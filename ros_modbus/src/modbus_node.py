#! /usr/bin/env python
# -*- coding: utf_8 -*-

import rclpy
import yaml
from pyModbusTCP.client import ModbusClient
from rclpy.node import Node
from ros_modbus_msgs.msg import Modbus


class modbus_node(Node):

    def __init__(self):
        super().__init__('modbus_node')

        self.m_timeout = self.declare_parameter('timeout', 30).value
        self.m_sub_queue_size = self.declare_parameter('sub_queue_size',10).value
        self.m_pub_queue_size = self.declare_parameter('pub_queue_size', 5).value
        self.m_name = self.declare_parameter('name', 'test_device').value
        self.m_YAML_config_file = self.declare_parameter('YAML_config_file', '/home/ecn/ros2/src/modbus_ros2/ros_modbus/map.yaml').value
        self.m_debug = self.declare_parameter('debug', True).value

        #member varaiables declarations in case of faulty configuration
        self.m_address = None
        self.m_port = None
        self.m_publish_on_timer = None
        self.m_msg_on_timer = Modbus()
        self.m_msg_on_timer.header.frame_id = 'Modbus_report'
        self.m_msg_on_timer.in_out = []
        self.m_msg_on_timer.values = []
        self.m_publish_on_event = None
        self.m_msg_on_event = Modbus()
        self.m_msg_on_event.header.frame_id = 'Modbus_report'
        self.m_msg_on_event.in_out = []
        self.m_msg_on_event.values = []
        self.m_IO = None
        self.m_clientMaster = None
        self.m_temp_value = None
        self.m_publish = False

        self.m_connected = False
        self.m_configOK = False
        self.m_reconnection_timer = None

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
            self.m_publisher_timer = self.create_timer(1/conf_dic[self.m_name]['publish_frequency'], self.publish_timer_callback)
            self.m_checker_timer = self.create_timer(0.001, self.check_timer_callback)
            self.m_publish_on_timer = dict(zip(conf_dic[self.m_name]['publish_on_timer'],[0]*len(conf_dic[self.m_name]['publish_on_timer'])))
            self.m_publish_on_event = dict(zip(conf_dic[self.m_name]['publish_on_event'],[0]*len(conf_dic[self.m_name]['publish_on_event'])))
            self.m_IO = {}

            for type, data_type in conf_dic[self.m_name]['input'].items():
                for name, address in data_type.items():
                    if address:
                        self.m_IO.update({name: ['input',type, address-1]}) # address starts at 1 while at 0 on python
                    else:
                        self.m_IO.update({name: ['input',type, address]})
            for type, data_type in conf_dic[self.m_name]['output'].items():
                for name, address in data_type.items():
                    if address: 
                        self.m_IO.update({name: ['output',type, address-1]}) # address starts at 1 while at 0 on python
                    else:
                        self.m_IO.update({name: ['input',type, address]})

            self.m_clientMaster = ModbusClient(host=self.m_address, port=self.m_port, timeout=2, auto_open=True, auto_close=True)
            self.get_logger().info(f'Configuring device {self.m_name} with address {self.m_address} and port {self.m_port}')
            self.get_logger().info(f'Configuring device {self.m_name} with IO {[k for k in self.m_IO.keys()]}')
            self.m_configOK = self.verify()
            if self.m_configOK:
                self.get_logger().info(f'Connected to {self.m_address}:{self.m_port}')
                self.get_logger().info(f'Configured {self.m_name} successfully')
                self.m_connected = True

            

    def verify(self):
        if not self.m_clientMaster.open():
            return False
        else:
            for key in self.m_publish_on_event.keys():
                if key not in self.m_IO.keys():
                    self.get_logger().error(f'I/O {key} is not provided in configuration file')
                    return False
                elif self.m_IO[key][1] not in ['digital', 'analog']:
                    self.get_logger().error(f'I/O {key} is provided with incorrect I/O type (given {self.m_IO[key][1]} expected digital or analog) in configuration file')
                    return False
                elif self.m_IO[key][2] == None:
                    self.get_logger().error(f'I/O {key} is provided with no address in configuration file')
                    return False
        return True


    def restart_connection(self):
        self.m_clientMaster = ModbusClient(host=self.m_address, port=self.m_port, timeout=2, auto_open=True, auto_close=True)
        test = self.m_clientMaster.open() 
        if test and not self.m_connected:
            self.get_logger().info(f'Reconnected to {self.m_address}:{self.m_port}')
            self.m_connected = True
            self.m_reconnection_timer = None


    def now(self):
        return self.get_clock().now().to_msg()


    def check_timer_callback(self):

        if self.m_configOK and self.m_connected and self.m_clientMaster.open():

            for key in self.m_publish_on_event.keys():

                if self.m_IO[key][0] == 'input':

                    if self.m_IO[key][1] == 'digital':
                        try:
                            self.m_temp_value = int(self.m_clientMaster.read_discrete_inputs(self.m_IO[key][2])[0])
                        except:
                            self.get_logger().error(f'Tried to read I/O {key} and failed, skipping')

                    elif self.m_IO[key][1] == 'analog':
                        try:
                            self.m_temp_value = int(self.m_clientMaster.read_input_registers(self.m_IO[key][2])[0])
                        except:
                            self.get_logger().error(f'Tried to read I/O {key} and failed, skipping')

                    else:
                        self.get_logger().warn(f'Unsupported output type for I/O {key}, skipping')

                elif self.m_IO[key][0] == 'output':

                    if self.m_IO[key][1] == 'digital':
                        try:
                            self.m_temp_value = int(self.m_clientMaster.read_coils(self.m_IO[key][2])[0])
                        except:
                            self.get_logger().error(f'Tried to read I/O {key} and failed, skipping')

                    elif self.m_IO[key][1] == 'analog':
                        try:
                            self.m_temp_value = int(self.m_clientMaster.read_holding_registers(self.m_IO[key][2])[0])
                        except:
                            self.get_logger().error(f'Tried to read I/O {key} and failed, skipping')

                    else:
                        self.get_logger().warn(f'Unsupported output type for I/O {key}, skipping')

                else:
                    self.get_logger().warn(f'I/O {key} is not set as input nor output, skipping')

                if not self.m_publish and self.m_publish_on_event[key] != self.m_temp_value :
                    self.m_publish_on_event[key] = self.m_temp_value
                    self.m_publish = True

            if self.m_publish:
                self.m_msg_on_event.in_out = list(self.m_publish_on_event.keys())
                self.m_msg_on_event.values = list(self.m_publish_on_event.values())
                self.m_msg_on_event.header.stamp = self.now()
                self.m_clientMaster.close()
                self.m_publisher.publish(self.m_msg_on_event)
                self.m_publish = False
        else:
            if not self.m_configOK:
                self.get_logger().warn(f'Timer callback but configuration is not valid, reconfiguring')
                self.configure()
            else:
                self.get_logger().warn(f'Timer callback but connection to {self.m_address}:{self.m_port} lost, reconnecting')
                self.reconnection_timer = self.create_timer(0.0001, self.restart_connection)


    def publish_timer_callback(self):
        if self.m_configOK and self.m_connected and self.m_clientMaster.open():
            for key in self.m_publish_on_timer.keys():
                if self.m_IO[key][0] == 'input':
                    if self.m_IO[key][1] == 'digital':
                        try:
                            self.m_publish_on_timer.update({key: int(self.m_clientMaster.read_discrete_inputs(self.m_IO[key][2])[0])})
                        except:
                            self.get_logger().error(f'Tried to read I/O {key} and failed, skipping')
                    elif self.m_IO[key][1] == 'analog':
                        try:
                            self.m_publish_on_timer.update({key: int(self.m_clientMaster.read_input_registers(self.m_IO[key][2])[0])})
                        except:
                            self.get_logger().error(f'Tried to read I/O {key} and failed, skipping')
                    else:
                        self.get_logger().warn(f'Unsupported output type for I/O {key}, skipping')
                elif self.m_IO[key][0] == 'output':
                    if self.m_IO[key][1] == 'digital':
                        try:
                            self.m_publish_on_timer.update({key: int(self.m_clientMaster.read_coils(self.m_IO[key][2])[0])})
                        except:
                            self.get_logger().error(f'Tried to read I/O {key} and failed, skipping')
                    elif self.m_IO[key][1] == 'analog':
                        try:
                            self.m_publish_on_timer.update({key: int(self.m_clientMaster.read_holding_registers(self.m_IO[key][2])[0])})
                        except:
                            self.get_logger().error(f'Tried to read I/O {key} and failed, skipping')
                    else:
                        self.get_logger().warn(f'Unsupported output type for I/O {key}, skipping')
                else:
                    self.get_logger().warn(f'I/O {key} is not set as input nor output, skipping')
            self.m_msg_on_timer.in_out = list(self.m_publish_on_timer.keys())
            self.m_msg_on_timer.values = list(self.m_publish_on_timer.values())
            self.m_msg_on_timer.header.stamp = self.now()
            self.m_clientMaster.close()
            self.m_publisher.publish(self.m_msg_on_timer)
        else:
            if not self.m_configOK:
                self.get_logger().warn(f'Timer callback but configuration is not valid, reconfiguring')
                self.configure()
            else:
                self.get_logger().warn(f'Timer callback but connection to {self.m_address}:{self.m_port} lost, reconnecting')
                self.reconnection_timer = self.create_timer(0.0001, self.restart_connection)


    def subscriber_callback(self, msg):   

        if self.m_configOK and self.m_connected and self.m_clientMaster.open():   
            command = dict(zip(msg.in_out, msg.values))
            for key, value in command.items():
                if key in self.m_IO.keys():
                    if self.m_IO[key][0] == 'output':
                        if self.m_IO[key][1] == 'digital':
                            try:
                                self.m_clientMaster.write_single_coil(self.m_IO[key][2], bool(value))
                            except:
                                self.get_logger().error(f'Tried to set I/O {key} at value {bool(value)} (value given was {value}) and failed, skipping')
                        elif self.m_IO[key][1] == 'analog':
                            try:
                                self.m_clientMaster.write_single_register(self.m_IO[key][2], value)
                            except:
                                self.get_logger().error(f'Tried to set I/O {key} at value {value} and failed, skipping')
                        else:
                            self.get_logger().warn(f'Unsupported output type for I/O {key}, skipping')
                    else:
                        self.get_logger().warn(f'I/O {key} is not set as output, skipping')
                else:
                    self.get_logger().warn(f'I/O {key} not declared, skipping')
        else:
            if not self.m_configOK:
                self.get_logger().warn(f'Timer callback but configuration is not valid, reconfiguring')
                self.configure()
            else:
                self.get_logger().warn(f'Timer callback but connection to {self.m_address}:{self.m_port} lost, reconnecting')
                self.reconnection_timer = self.create_timer(0.0001, self.restart_connection)


    
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
