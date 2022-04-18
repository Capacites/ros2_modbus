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

/**
 * @brief The ModbusNode class is a basic class to run a node communicating with a modbus device via TCP
 */

class ModbusNode : public rclcpp::Node
{
public:

    enum STATE_CODE
    {
     NO_ISSUE = 0,                       /*!< The node is running without encountering any issue                          */
     INITIALIZING = 1,                   /*!< The node is initializing itself                                             */
     NOT_CONNECTED = 2,                  /*!< The node could not establish, or lost the connection with the Modbus device */
     INVALID_CONFIGURATION_FILE = 3,     /*!< The provided configuration file is not valid                                */
     INVALID_IO_TYPE = 4,                /*!< An IO was given a type other than input or output                           */
     INVALID_IO_DATA_TYPE = 5,           /*!< An IO was given a data type other than digital or analog                    */
     INVALID_IO_KEY = 6,                 /*!< A provided key to publish, or write on is not given a configuration         */
     INVALID_IO_TO_WRITE = 7             /*!< Tried to write on a non output IO                                           */
    };

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
     * @brief Verify IO structure.
     *
     * Checks that each IO provided has a type "input" or "output".
     * Checks that each IO provided has a data type "digital" or "analog".
     * Checks that each IO provided has an address.
     *
     * @return true if the configuration appears valid
     * @return false if an IO is provided with a non conform parameter
     */
    bool verify_IO();

    /**
     * @brief Tries to open the connection to configured device.
     *
     * @return true if the connection is established
     * @return false otherwise
     */
    bool verify_connection();

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
        std::string type;        /*!< The IO type, can be input or output                                                                                      */
        std::string data_type;   /*!< The IO data type, can be digital or analog                                                                               */
        int address;             /*!< The IO address in its register, starting at 1 on configuration file to match reference device Bechkoff KL1889 and EL2809 */
    };

//ROS parameters
    int m_timeout = declare_parameter<int>("timeout", 30);                                                                                        /*!< Timeout for connection to the modbus device */
    int m_sub_queue_size = declare_parameter<int>("sub_queue_size",10);                                                                           /*!< Queue size for subscribers                  */
    int m_pub_queue_size = declare_parameter<int>("pub_queue_size", 5);                                                                           /*!< Queue size for publishers                   */
    std::string m_name = declare_parameter<std::string>("name", "test_device");                                                                   /*!< Device name                                 */
    std::string m_YAML_config_file = declare_parameter<std::string>("YAML_config_file", "/home/ecn/ros2/src/modbus_ros2/ros_modbus/map.yaml");    /*!< YAML device desription file full path       */
    bool m_debug = declare_parameter<bool>("debug", false);                                                                                       /*!< Debug mode                                  */

//Node member variables

    std::string m_address;         /*!< IP address of the modbus device          */
    int m_port;                    /*!< Listening port of the modbus device      */
    modbus_t *m_ctx;               /*!< Modbus context                           */

    std::map<std::string, uint16_t> m_publish_on_timer;       /*!< Map of IO and their values to publish on timer callback */
    ros_modbus_msgs::msg::Modbus m_msg_on_timer;              /*!< Modbus message for values to publish on timer callback  */

    std::map<std::string, uint16_t> m_publish_on_event;       /*!< Map of IO and their values to publish on event          */
    ros_modbus_msgs::msg::Modbus m_msg_on_event;              /*!< Modbus message for values to publish on event           */

    std::map<std::string, uint16_t> m_IO;                     /*!< Map of all declared IO to publish and their values      */
    std::map<std::string, m_IO_struct> m_IO_map;              /*!< Map of all declared IO and their definition             */
    std::set<std::string> m_IO_list;                          /*!< List of all declared IO to publish                      */

    std::mutex m_ctx_guard;        /*!< Modbus context guard                     */
    std::mutex m_IO_map_guard;     /*!< IO map guard                             */
    std::mutex m_IO_guard;         /*!< IO definition guard                      */

    uint8_t m_temp_digit_value;    /*!< Temporary value for digital reading      */
    uint16_t m_update_temp_value;  /*!< Temporary value for reading              */
    uint16_t m_checker_temp_value; /*!< Temporary value for checking events      */
    m_IO_struct m_IO_update_temp;  /*!< Temporary value for updating values      */
    m_IO_struct m_IO_sub_temp;     /*!< Temporary value for sending command      */
    m_IO_struct m_IO_temp;         /*!< Temporary value for configuration        */

    bool m_publish;                /*!< Control to publish event message         */
    bool m_connected{false};       /*!< Connection state to the modbus device    */
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
