#ifndef MODBUS_NODE_H
#define MODBUS_NODE_H

//ROS include
#include <rclcpp/rclcpp.hpp>
#include <ros_modbus_msgs/msg/modbus.hpp>
#include <ros_modbus_msgs/msg/state.hpp>

//Modbus include
#include <modbus/modbus-tcp.h>

//YAML include
#include <yaml-cpp/yaml.h>

//Standard includes
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <set>

using namespace std::chrono_literals;

namespace modbus_node {

class ModbusNode : public rclcpp::Node
{
public:

    ModbusNode(rclcpp::NodeOptions options = rclcpp::NodeOptions());

private:

    void configure();
    bool verify_IO();
    bool verify_connection();
    void restart_connection();
    void update_timer_callback();
    void check_timer_callback();
    void publish_timer_callback();
    void subscriber_callback(ros_modbus_msgs::msg::Modbus::SharedPtr msg);
    void publish_state(bool, int);

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

    std::map<std::string, uint16_t> m_IO;
    std::map<std::string, m_IO_struct> m_IO_map;
    std::set<std::string> m_IO_list;

    modbus_t *m_ctx;
    std::mutex m_IO_guard;
    std::mutex m_IO_map_guard;
    std::mutex m_ctx_guard;
    int m_cout = 0;

    uint8_t m_temp_digit_value;
    uint16_t m_update_temp_value;
    uint16_t m_checker_temp_value;
    m_IO_struct m_IO_update_temp;
    m_IO_struct m_IO_sub_temp;

    bool m_publish;
    bool m_connected{false};
    bool m_configOK;

    m_IO_struct m_IO_temp;
    int m_buffer_size;

//ROS components
    rclcpp::Subscription<ros_modbus_msgs::msg::Modbus>::SharedPtr m_subscriber;
    rclcpp::Publisher<ros_modbus_msgs::msg::Modbus>::SharedPtr m_timer_publisher;
    rclcpp::Publisher<ros_modbus_msgs::msg::Modbus>::SharedPtr m_event_publisher;
    rclcpp::Publisher<ros_modbus_msgs::msg::State>::SharedPtr m_state_publisher;

    rclcpp::TimerBase::SharedPtr m_publisher_timer;
    rclcpp::TimerBase::SharedPtr m_checker_timer;
    rclcpp::TimerBase::SharedPtr m_update_timer;

   };
}

#endif
