#! /usr/bin/env python
# -*- coding: utf_8 -*-

from rclpy.node import Node
import rclpy
from pyModbusTCP.client import ModbusClient
import yaml
from ros_modbus_msgs.msg import Modbus


class modbus_node(Node):

    def __init__(self):
        super().__init__('modbus_node')

        self.m_timeout = self.declare_parameter('timeout', 30).value
        self.m_sub_queue_size = self.declare_parameter('sub_queue_size',10).value
        self.m_pub_queue_size = self.declare_parameter('pub_queue_size', 1).value
        self.m_name = self.declare_parameter('name', 'test_device').value
        self.m_YAML_config_file = self.declare_parameter('YAML_config_file', '/home/ecn/ros2/src/modbus_ros2/ros_modbus/map.yaml').value
        self.m_debug = self.declare_parameter('debug', True).value

        #member varaiables declarations in case of faulty configuration
        self.m_address = None
        self.m_port = None
        self.m_publish_on_timer = None
        self.m_msg_on_timer = Modbus()
        self.m_msg_on_timer.header.frame_id = 'Modbus_report'
        self.m_msg_on_timer.coils = []
        self.m_msg_on_timer.values = []
        self.m_publish_on_event = None
        self.m_msg_on_event = Modbus()
        self.m_msg_on_event.header.frame_id = 'Modbus_report'
        self.m_msg_on_event.coils = []
        self.m_msg_on_event.values = []
        self.m_IO = None
        self.m_clientMaster = None
       
        self.m_configOK = False

        try:
            self.configure()
        except:
            self.get_logger().error("Configuration file not valid, please provide a valid configuration file")

        #ROS related components
        self.m_subscriber = self.create_subscription(Modbus, "/ros_modbus/command",self.subscriber_callback, self.m_sub_queue_size)
        self.m_publisher = self.create_publisher(Modbus, "/ros_modbus/report", self.m_pub_queue_size)
        

    def configure(self):
        with open(rf'{self.m_YAML_config_file}') as file:
            conf_dic = yaml.safe_load(file)
            self.m_address = conf_dic[self.m_name]['address']
            self.m_port = conf_dic[self.m_name]['port']
            self.create_timer(1/conf_dic[self.m_name]['publish_frequency'], self.publish_timer_callback)
            self.m_publish_on_timer = dict(zip(conf_dic[self.m_name]['publish_on_timer'],[0]*len(conf_dic[self.m_name]['publish_on_timer'])))
            self.m_publish_on_event = dict(zip(conf_dic[self.m_name]['publish_on_event'],[0]*len(conf_dic[self.m_name]['publish_on_event'])))
            self.m_IO = {}

            for type, data_type in conf_dic[self.m_name]['input'].items():
                for name, address in data_type.items():
                    self.m_IO.update({name: ['input',type, address]})
            for type, data_type in conf_dic[self.m_name]['output'].items():
                for name, address in data_type.items():
                    self.m_IO.update({name: ['output',type, address]})

            self.m_clientMaster = ModbusClient(host=self.m_address, port=self.m_port, debug=self.m_debug, timeout=2)
            self.get_logger().info(f'Configured automat {self.m_name} with address {self.m_address} and port {self.m_port}')
            self.get_logger().info(f'Configured automat {self.m_name} with IO {[k for k in self.m_IO.keys()]}')
            self.m_configOK = True


    def now(self):
            return self.get_clock().now().to_msg()


    def publish_timer_callback(self):
        
        if self.m_configOK :
            self.m_clientMaster.open()
            if self.m_clientMaster.is_open():
                for key in self.m_publish_on_timer.keys():
                    if self.m_IO[key][0] == 'input':
                        if self.m_IO[key][1] == 'digital':
                            self.m_publish_on_timer.update({key: int(self.m_clientMaster.read_coils(self.m_IO[key][2])[0])})
                            print(self.m_publish_on_timer[key])
            self.m_msg_on_timer.coils = list(self.m_publish_on_timer.keys())
            print(self.m_publish_on_timer.values())
            self.m_msg_on_timer.values = list(self.m_publish_on_timer.values())
            self.m_msg_on_timer.header.stamp = self.now()
            self.m_clientMaster.close()
            self.m_publisher.publish(self.m_msg_on_timer)

        else:
            self.get_logger().warn(f'Timer callback but configuration is not valid, skipping')


    def subscriber_callback(self, msg):   
        if self.m_configOK:    
            command = dict(zip(msg.coils, msg.values))
            for coil, value in command.items():
                if coil in self.m_IO.keys():
                    if self.m_IO[coil][0] == 'output':
                        if self.m_IO[coil][1] == 'digital':
                            try:
                                self.m_clientMaster.write_single_coil(self.m_IO[coil][2], bool(value))
                            except:
                                self.get_logger().error(f'Tried to set coil {coil} at value {bool(value)} (value given was {value}) and failed, skipping')
                        elif self.m_IO[coil][1] == 'analog':
                            try:
                                self.m_clientMaster.write_single_coil(self.m_IO[coil][2], value)
                            except:
                                self.get_logger().error(f'Tried to set coil {coil} at value {value} and failed, skipping')
                        else:
                            self.get_logger().warn(f'Unsupported output type for coil {coil}, skipping')
                    else:
                        self.get_logger().warn(f'Coil {coil} is not set as output, skipping')
                else:
                    self.get_logger().warn(f'Coil {coil} not declared, skipping')
        else:
            self.get_logger().warn(f'Messaged received but configuration is not valid, skipping')


    
def main():
    try:
        rclpy.init()
    except:
        pass
    node = modbus_node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
