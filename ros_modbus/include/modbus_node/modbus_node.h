#ifndef MODBUS_NODE_H
#define MODBUS_NODE_H

//ROS include
#include <rclcpp/rclcpp.hpp>
#include <ros_modbus_msgs/msg/modbus.hpp>

//Modbus include
#include <modbusRequest.hpp>
#include <modbusResponse.hpp>
#include <TCP/connection.hpp>

#include <yaml-cpp/yaml.h>

//Standard includes
#include <string>
#include <vector>
#include <map>
#include <sys/socket.h>

using namespace std::chrono_literals;

namespace modbus_node {

class ModbusNode : public rclcpp::Node
{
public:

    ModbusNode(rclcpp::NodeOptions options = rclcpp::NodeOptions());

private:

    void configure();
    bool verify();
    void reconnection();
    void check_timer_callback();
    void publish_timer_callback();
    void subscriber_callback();


//IO structure definition
    struct m_IO_struct{
        std::string type;
        std::string data_type;
        int address;
    };

//ROS parameters
    int m_timeout = declare_parameter<int>("timeout", 30);
    int m_sub_queue_size = declare_parameter<int>("sub_queue_size",10);
    int m_pub_queue_size = declare_parameter<int>("pub_queue_size", 5);
    std::string m_name = declare_parameter<std::string>("name", "test_device");
    std::string m_YAML_config_file = declare_parameter<std::string>("YAML_config_file", "/home/ecn/ros2/src/modbus_ros2/ros_modbus/map.yaml");
    bool m_debug = declare_parameter<bool>("debug", false);

//Node member variables
    std::string m_address;
    int m_port;
    std::map<std::string, uint16_t> m_publish_on_timer;
    ros_modbus_msgs::msg::Modbus m_msg_on_timer;
    std::map<std::string, uint16_t> m_publish_on_event;
    ros_modbus_msgs::msg::Modbus m_msg_on_event;
    std::map<std::string, m_IO_struct> m_IO;
    int m_sock{socket(AF_INET, SOCK_STREAM,0)};
    sockaddr_in m_server;
    int m_temp_value;
    bool m_publish;
    bool m_connected;
    bool m_configOK;

    m_IO_struct m_IO_temp;


//ROS components
    rclcpp::TimerBase::SharedPtr m_reconnection_timer;
    rclcpp::TimerBase::SharedPtr m_publisher_timer;
    rclcpp::TimerBase::SharedPtr m_checker_timer;

};
}

#endif
