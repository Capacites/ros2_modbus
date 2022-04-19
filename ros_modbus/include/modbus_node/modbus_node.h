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
#include "modbus_interface.h"

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
using namespace Modbus;

namespace modbus_node {

/**
 * @brief The ModbusNode class is a basic class to run a node communicating with a modbus device via TCP
 */

class ModbusNode : public rclcpp::Node
{
public:

    /**
     * @brief Node constructor.
     *
     * @param options The node options
     */
    ModbusNode(rclcpp::NodeOptions options = rclcpp::NodeOptions());

private:

    /**
     * @brief Load the YAML configuration file provided at the node start.
     *
     * Initialize node with the given YAML.
     * Calls verify_connection().
     * Calls verify_IO().
     */
    void configure();

    /**
     * @brief Restart the connection to the configured device.
     *
     * Close the modbus m_ctx context.
     * Tries to restart the connection
     * Publish a state message if connection restablished.
     */
    void restart_connection();

    /**
     * @brief Update the values of all interest IO one by one.
     *
     * Update the values of all interest IO one by one.
     * Publish a state message with not connected state.
     * Calls restart_connection() in it's thread.
     */
    void update_timer_callback();

    /**
     * @brief Look for any IO state change for IO flagged to be published over on event.
     *
     * Compares the current IO and the temp ones stored in m_checker_temp_value.
     * If any change is observed, prepares and publishes a Modbus message on /ros_modbus/report_event with all IO flagged to be published over on event.
     */
    void check_timer_callback();

    /**
     * @brief Publish the state of all IO flagged to be published over on timer.
     */
    void publish_timer_callback();

    /**
     * @brief Send a received command to the device
     *
     * @param msg The received message with the command
     */
    void subscriber_callback(ros_modbus_msgs::msg::Modbus::SharedPtr msg);

    /**
     * @brief Publish a state message
     *
     * @param state The state to publish, true for a valid state, false if an error occurs
     * @param state code The state code, to help troubleshoot
     */
    void publish_state(bool, int);    

//ROS parameters
    int m_timeout = declare_parameter<int>("timeout", 30);                                                                                        /*!< Timeout for connection to the modbus device */
    int m_sub_queue_size = declare_parameter<int>("sub_queue_size",10);                                                                           /*!< Queue size for subscribers                  */
    int m_pub_queue_size = declare_parameter<int>("pub_queue_size", 5);                                                                           /*!< Queue size for publishers                   */
    std::string m_name = declare_parameter<std::string>("name", "test_device");                                                                   /*!< Device name                                 */
    std::string m_YAML_config_file = declare_parameter<std::string>("YAML_config_file", "/home/ecn/ros2/src/modbus_ros2/ros_modbus/map.yaml");    /*!< YAML device desription file full path       */
    bool m_debug = declare_parameter<bool>("debug", false);                                                                                       /*!< Debug mode                                  */

//Node member variables
    std::map<std::string, uint16_t> m_publish_on_timer;       /*!< Map of IO and their values to publish on timer callback */
    ros_modbus_msgs::msg::Modbus m_msg_on_timer;              /*!< Modbus message for values to publish on timer callback  */

    std::map<std::string, uint16_t> m_publish_on_event;       /*!< Map of IO and their values to publish on event          */
    ros_modbus_msgs::msg::Modbus m_msg_on_event;              /*!< Modbus message for values to publish on event           */

    ModbusInterface m_modbus_device;

    uint8_t m_temp_digit_value;    /*!< Temporary value for digital reading      */
    uint16_t m_update_temp_value;  /*!< Temporary value for reading              */
    uint16_t m_checker_temp_value; /*!< Temporary value for checking events      */

    bool m_publish;                /*!< Control to publish event message         */
    bool m_configOK;               /*!< Configuration state to the modbus device */

//ROS components
    rclcpp::CallbackGroup::SharedPtr mp_callback_group_subscriber = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);          /*!< Subscriber callback group    */
    rclcpp::CallbackGroup::SharedPtr mp_callback_group_publisher = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);           /*!< Publisher callback group     */
    rclcpp::CallbackGroup::SharedPtr mp_callback_group_update = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);              /*!< Update callback group        */
    rclcpp::CallbackGroup::SharedPtr mp_callback_group_checker = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);             /*!< Event checker callback group */
    rclcpp::CallbackGroup::SharedPtr mp_callback_group_reconnection = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);        /*!< Reconnection callback group  */

    rclcpp::SubscriptionOptions m_sub_option;                  /*!< Subscription options */

    rclcpp::Subscription<ros_modbus_msgs::msg::Modbus>::SharedPtr mp_subscriber;                                                                                                                          /*!< Command subscriber */
    rclcpp::Publisher<ros_modbus_msgs::msg::Modbus>::SharedPtr mp_timer_publisher = this->create_publisher<ros_modbus_msgs::msg::Modbus>("/ros_modbus/report_timer", rclcpp::QoS(m_pub_queue_size));      /*!< On timer publisher */
    rclcpp::Publisher<ros_modbus_msgs::msg::Modbus>::SharedPtr mp_event_publisher = this->create_publisher<ros_modbus_msgs::msg::Modbus>("/ros_modbus/report_event", rclcpp::QoS(m_pub_queue_size));      /*!< On event publisher */
    rclcpp::Publisher<ros_modbus_msgs::msg::State>::SharedPtr mp_state_publisher = this->create_publisher<ros_modbus_msgs::msg::State>("/ros_modbus/state", rclcpp::QoS(m_pub_queue_size));               /*!< State publisher    */

    /**
     * @note creating timers to assign them to several callback groups effectivelly assigning them several threads
     */
    rclcpp::TimerBase::SharedPtr mp_publisher_timer;           /*!< On timer publisher timer */
    rclcpp::TimerBase::SharedPtr mp_checker_timer;             /*!< Checker timer            */
    rclcpp::TimerBase::SharedPtr mp_update_timer;              /*!< IO value update timer    */
    rclcpp::TimerBase::SharedPtr mp_reconnection_timer;        /*!< Reconnection timer       */

   };
}

#endif
