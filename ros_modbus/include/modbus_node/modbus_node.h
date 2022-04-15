// -*- lsst-c++ -*-
/**
 * @file modbus_node.h
 *
 * @brief modbus_node header
 */

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

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif


using namespace std::chrono_literals;

namespace modbus_node {

class ModbusNode : public rclcpp::Node
{
public:

    ModbusNode(rclcpp::NodeOptions options = rclcpp::NodeOptions());

private:

    /**
     * @brief Load the YAML configuration file provided at the node start.
     *
     * Initialize node with the given YAML.
     */
    void configure();

    /**
     * @brief Verify IO structure.
     *
     * Checks that each IO provided has a type "input" or "output".
     * Checks that each IO provided has a data type "digital" or "analog"
     * Checks that each IO provided has an address
     *
     * @return true if the configuration appears valid
     * @return false if an IO is provided with a non conform parameter
     */
    bool verify_IO();

    /**
     * @brief Tries to open the connection to configured device
     *
     * @return true if the connection is established
     * @return false otherwise
     */
    bool verify_connection();

    /**
     * @brief Restart the connection to the configured device
     *
     * Close the modbus m_ctx context
     */
    void restart_connection();

    /**
     * @brief update the values of all interest IO one by one
     *
     * Restart reconnection
     */
    void update_timer_callback();

    /**
     * @brief
     *
     *
     *
     * @param
     * @return
     */
    void check_timer_callback();

    /**
     * @brief
     *
     *
     *
     * @param
     * @return
     */
    void publish_timer_callback();

    /**
     * @brief
     *
     *
     *
     * @param
     * @return
     */
    void subscriber_callback(ros_modbus_msgs::msg::Modbus::SharedPtr msg);

    /**
     * @brief
     *
     *
     *
     * @param
     * @return
     */
    void publish_state(bool, int);    

    /**
     * @struct m_IO_struct
     * @brief Structure defining an IO
     *
     * @var m_IO_struct::type
     * IO type, can be input or output
     * @var m_IO_struct::data_type
     * IO data type, can be digital or analog
     * @var m_IO_struct::address
     * IO address
     */
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
    rclcpp::CallbackGroup::SharedPtr mp_callback_group_subscriber = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::CallbackGroup::SharedPtr mp_callback_group_publisher = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::CallbackGroup::SharedPtr mp_callback_group_update = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::CallbackGroup::SharedPtr mp_callback_group_checker = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::CallbackGroup::SharedPtr mp_callback_group_reconnection = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions m_sub_option;

    rclcpp::Subscription<ros_modbus_msgs::msg::Modbus>::SharedPtr mp_subscriber;
    rclcpp::Publisher<ros_modbus_msgs::msg::Modbus>::SharedPtr mp_timer_publisher = this->create_publisher<ros_modbus_msgs::msg::Modbus>("/ros_modbus/report_timer", rclcpp::QoS(m_pub_queue_size));
    rclcpp::Publisher<ros_modbus_msgs::msg::Modbus>::SharedPtr mp_event_publisher = this->create_publisher<ros_modbus_msgs::msg::Modbus>("/ros_modbus/report_event", rclcpp::QoS(m_pub_queue_size));
    rclcpp::Publisher<ros_modbus_msgs::msg::State>::SharedPtr mp_state_publisher = this->create_publisher<ros_modbus_msgs::msg::State>("/ros_modbus/state", rclcpp::QoS(m_pub_queue_size));

    rclcpp::TimerBase::SharedPtr mp_publisher_timer;
    rclcpp::TimerBase::SharedPtr mp_checker_timer;
    rclcpp::TimerBase::SharedPtr mp_update_timer;
    rclcpp::TimerBase::SharedPtr mp_reconnection_timer;

   };
}

#endif
