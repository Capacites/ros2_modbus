// -*- lsst-c++ -*-
/*
 * This file is part of modbus_ros2.
 *
 * MIT License
 *
 * Copyright (c) 2022 CAPACITES
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file modbus_node.cpp
 *
 * @brief modbus_node functions definition
 */

#include <modbus_node/modbus_node.h>
#include <string>
#include <fstream>
#include <vector>
#include <utility>

using namespace std::chrono_literals;
using namespace modbus_node;
using namespace Modbus;

/**
 * @brief Node constructor.
 *
 * @param options The node options
 */
ModbusNode::ModbusNode(rclcpp::NodeOptions options)
    : Node("modbus_node", options)
{
    // configuration of subscriber with it's callback group
    m_sub_option.callback_group = mp_callback_group_publisher;
    mp_subscriber = this->create_subscription<ros_modbus_msgs::msg::Modbus>("command", rclcpp::QoS(m_pub_queue_size), [this](ros_modbus_msgs::msg::Modbus::SharedPtr msg){subscriber_callback(msg);}, m_sub_option);

    // initializing messages
    m_msg_state.header.set__frame_id(m_name);

    m_msg_on_timer.header.set__frame_id(m_name);
    m_msg_on_timer.set__in_out(std::vector<std::string>());
    m_msg_on_timer.set__values(std::vector<uint16_t>());

    m_msg_on_event.header.set__frame_id(m_name);
    m_msg_on_event.set__in_out(std::vector<std::string>());
    m_msg_on_event.set__values(std::vector<uint16_t>());

    publish_state(false, ModbusInterface::INITIALIZING); // Currently not working, it appears that publisher is not fully initialized at this staged

    try
    {
        configure(); //attempt to configure
    }
    catch (...) {
        RCLCPP_ERROR(get_logger(), "Configuration file %s not valid, please provide a valid configuration file", m_YAML_config_file.c_str());
        publish_state(false, ModbusInterface::INVALID_CONFIGURATION_FILE);
    }

    // assigning timers to threads and put them to sleep
    mp_checker_timer = this->create_wall_timer(0s, [this](){check_timer_callback();}, mp_callback_group_checker);
    mp_checker_timer->cancel();
    mp_reconnection_timer = this->create_wall_timer(0s, [this](){restart_connection();}, mp_callback_group_reconnection);
    mp_reconnection_timer->cancel();
}

/**
 * @brief Publish a state message
 *
 * @param state The state to publish, true for a valid state, false if an error occurs
 * @param state code The state code, to help troubleshoot
 */
void ModbusNode::publish_state(bool state, int state_code)
{
    auto msg_state = ros_modbus_msgs::msg::State();
    msg_state.header.set__frame_id(m_name);
    msg_state.header.set__stamp(now());
    msg_state.set__state(state);
    msg_state.set__error(state_code);
    mp_state_publisher->publish(msg_state);
}

/**
 * @brief Publish a state message
 */
void ModbusNode::publish_state()
{
    int errorCode;
    bool state = m_modbus_device.getConnectionState();
    m_msg_state.header.set__stamp(now());
    m_msg_state.set__state(m_configOK && state);
    if(!m_configOK)
    {
        errorCode = ModbusInterface::INVALID_CONFIGURATION_FILE;
    }
    else if (!state)
    {
        errorCode = ModbusInterface::NOT_CONNECTED;
    }
    else
    {
        errorCode = ModbusInterface::NO_ISSUE;
    }
    m_msg_state.set__error(errorCode);
    mp_state_publisher->publish(m_msg_state);
}

/**
 * @brief Load the YAML configuration file provided at the node start.
 *
 * Initialize Modbus device with the given YAML.
 * Initiate connection.
 * Simple IO verification.
 */
void ModbusNode::configure()
{
    YAML::Node config = YAML::LoadFile(m_YAML_config_file);
    if(config[m_name]) // We need our device description
    {
        int temp_address;
        // configuring Modbus device context
        m_modbus_device.setContext(config[m_name]["address"].as<std::string>(),
                                   config[m_name]["port"].as<int>());

        m_modbus_device.initiateConnection();
        auto temp_context = m_modbus_device.getContext();
        RCLCPP_INFO(get_logger(),"Configuring device %s with address %s and port %d", m_name.c_str(), temp_context.first.c_str(), temp_context.second);

        m_modbus_device.setDevice(config[m_name]["connected_IO"]["digital_input"].as<int>(),
                                  config[m_name]["connected_IO"]["digital_output"].as<int>(),
                                  config[m_name]["connected_IO"]["analog_input"].as<int>(),
                                  config[m_name]["connected_IO"]["analog_output"].as<int>());

        // Creating map of IO to publish on timer
        if(config[m_name]["publish_on_timer"].size() != 0)
        {
            for (const auto &iter : config[m_name]["publish_on_timer"].as<std::vector<std::string>>())
            {

                RCLCPP_INFO(get_logger(),"%s", iter.c_str());
                m_publish_on_timer.insert({iter, 0});
            }
        }


        // Creating map of IO to publish on event
        if(config[m_name]["publish_on_event"].size() != 0)
        {
            for (const auto &iter : config[m_name]["publish_on_event"].as<std::vector<std::string>>())
            {
                m_publish_on_event.insert({iter, 0});
            }
        }

        // Construct IO structure for inputs, update map with IO name as key for it's structure
        for(YAML::const_iterator element=config[m_name]["input"].begin();element!=config[m_name]["input"].end();++element)
        {
            for(YAML::const_iterator element2=element->second.begin();element2!=element->second.end();++element2)
            {
                if(element2->second.IsScalar()) // Prevent substracting one to empty address
                {
                    temp_address = element2->second.as<int>()-1;
                }
                else
                {
                    temp_address = -1;
                }
                m_modbus_device.addIO(element2->first.as<std::string>(), "input", element->first.as<std::string>(), temp_address);
            }
        }

        // Construct IO structure for outputs, update map with IO name as key for it's structure
        for(YAML::const_iterator element=config[m_name]["output"].begin();element!=config[m_name]["output"].end();++element)
        {
            for(YAML::const_iterator element2=element->second.begin();element2!=element->second.end();++element2)
            {
                if(element2->second.IsScalar()) // Prevent substracting one to empty address
                {
                    temp_address = element2->second.as<int>()-1;
                }
                else
                {
                   temp_address = -1;
                }
                m_modbus_device.addIO(element2->first.as<std::string>(), "output", element->first.as<std::string>(), temp_address);
            }
        }

        RCLCPP_INFO(get_logger(),"Verifying device %s's IO", m_name.c_str());
        m_configOK = m_modbus_device.verifyIO();

        // Configure timers with desired rate
        mp_publisher_timer = this->create_wall_timer(std::chrono::milliseconds(int(1000./config[m_name]["publish_rate"].as<int>())),
                                                     [this](){publish_timer_callback();},
                                                     mp_callback_group_publisher);

        mp_update_timer = this->create_wall_timer(std::chrono::milliseconds(int(1000./config[m_name]["refresh_rate"].as<int>())),
                                                  [this](){update_timer_callback();},
                                                  mp_callback_group_update);

        mp_state_timer = this->create_wall_timer(std::chrono::milliseconds(int(1000./config[m_name]["state_rate"].as<int>())),
                                                  [this](){publish_state();},
                                                  mp_callback_group_state);


        // publish updated state
        if(!m_configOK)
        {
            RCLCPP_INFO(get_logger(), "Configuration appears invalid");
            publish_state(false, ModbusInterface::INVALID_CONFIGURATION_FILE);
        }
        else if (!m_modbus_device.getConnectionState())
        {
            RCLCPP_INFO(get_logger(), "Can not connect");
            publish_state(false, ModbusInterface::NOT_CONNECTED);
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Configuration appears valid, connected");
            publish_state(true, ModbusInterface::NO_ISSUE);
        }

    }
}

/**
 * @brief Restart the connection to the configured device.
 *
 * Tries to restart the connection.
 * Publish a state message if connection restablished.
 */
void ModbusNode::restart_connection()
{
    m_modbus_device.restartConnection();
    auto temp_context = m_modbus_device.getContext();
    RCLCPP_INFO(get_logger(), "Reconnected to %s:%d", temp_context.first.c_str(), temp_context.second);
    publish_state(true, ModbusInterface::NO_ISSUE);
    mp_reconnection_timer->cancel();
}

/**
 * @brief Update the modbus device memory.
 *
 * Update the modbus device memory.
 * Publish a state message with not connected state if needed.
 * Calls restart_connection() in it's thread if disconnected.
 */
void ModbusNode::update_timer_callback()
{
    mp_update_timer->cancel(); // avoid flooding the queue
    try
    {
        m_modbus_device.updateMemory();
    }
    catch(...) // If error occurs
    {
        if(!m_configOK) // And configuration is invalid
        {
            RCLCPP_WARN(get_logger(), "Timer callback but configuration is not valid, reconfiguring");
            publish_state(false, ModbusInterface::INVALID_CONFIGURATION_FILE);
            configure(); // Try to reconfigure
        }
        else
        {
            m_modbus_device.setConnectionState(false); // Default to a connection issue
            if(mp_reconnection_timer->is_canceled()) // Only wake the reconnection timer up if it's sleeping
            {
                auto temp_context = m_modbus_device.getContext();
                RCLCPP_WARN(get_logger(), "Connection to %s:%d lost, reconnecting", temp_context.first.c_str(), temp_context.second);
                publish_state(false, ModbusInterface::NOT_CONNECTED);
                mp_reconnection_timer->reset();
            }
        }
    }
    mp_checker_timer->reset(); // Look for an event
    mp_update_timer->reset();
}

/**
 * @brief Publish the state of all IO flagged to be published over on timer.
 */
void ModbusNode::publish_timer_callback()
{
    m_msg_on_timer.header.set__stamp(now());
    m_msg_on_timer.set__values(std::vector<uint16_t>());
    m_msg_on_timer.set__in_out(std::vector<std::string>());

    for(auto &[key, value] : m_publish_on_event)
    {
        m_publish_on_timer[key] = m_modbus_device.getIOvalue(key);
        m_msg_on_timer.in_out.push_back(key);
        m_msg_on_timer.values.push_back(m_publish_on_timer[key]);
    }
    if(m_msg_on_timer.in_out.size() !=0)
    {
        mp_timer_publisher->publish(m_msg_on_timer);
    }
}

/**
 * @brief Look for any IO state change for IO flagged to be published over on event.
 *
 * Compares the current IO and the temp ones stored in m_checker_temp_value.
 * If any change is observed, prepares and publishes a Modbus message on /ros_modbus/report_event with all IO flagged to be published over on event.
 */
void ModbusNode::check_timer_callback()
{
    mp_checker_timer->cancel();
    m_publish = false; // Default to not publish
    for(auto &[key, value] : m_publish_on_event)
    {
        m_checker_temp_value = m_modbus_device.getIOvalue(key);
        if(value != m_checker_temp_value) // A change has occured
        {
            m_publish_on_event[key] = m_checker_temp_value; // Update value

            if(!m_publish)
            {
                m_publish = true;
            }
        }
    }
    if (m_publish) // Publish if a change occured
    {
        m_msg_on_event.header.set__stamp(now());
        m_msg_on_event.set__values(std::vector<uint16_t>());
        m_msg_on_event.set__in_out(std::vector<std::string>());
        for (auto &[key, value] : m_publish_on_event)
        {
            m_msg_on_event.in_out.push_back(key);
            m_msg_on_event.values.push_back(value);
        }
        if(m_msg_on_event.in_out.size() !=0)
        {
            mp_event_publisher->publish(m_msg_on_event);
        }

    }
}

/**
 * @brief Send a received command to the device
 *
 * Get current outpu state and write desired output values on their address.
 * send the command as a vector of outputs.
 *
 * @param msg The received message with the command
 */
void ModbusNode::subscriber_callback(ros_modbus_msgs::msg::Modbus::SharedPtr p_msg)
{
    auto temp_digital = m_modbus_device.getMultipleOutputCoils();
    auto temp_analog = m_modbus_device.getMultipleOutputRegisters();
    for(int iter=0; iter < int(p_msg->in_out.size()); iter++) // iterate on the IO and value list of the message
    {
        auto m_IO_sub_temp = m_modbus_device.getIOMap();
        m_analog = false;
        m_digital = false;

        m_analog = false;
        m_digital = false;

        try {
            auto key = p_msg->in_out[iter];
            auto value = p_msg->values[iter];
            if(m_IO_sub_temp.find(key) == m_IO_sub_temp.end())
            {
                RCLCPP_WARN(get_logger(), "IO %s not declared, skipping", key.c_str());
                publish_state(false, ModbusInterface::INVALID_IO_TO_WRITE);
            }
            else
            {
                if(m_IO_sub_temp.at(key).type == "input") // Write on each output in command with received value
                {
                    RCLCPP_WARN(get_logger(), "I/O %s is configured as input, can't write here, skipping", key.c_str());
                    publish_state(false, ModbusInterface::INVALID_IO_TO_WRITE);
                }
                else if(m_IO_sub_temp.at(key).type == "output")
                {
                    if (m_IO_sub_temp.at(key).data_type == "digital")
                    {
                        temp_digital[m_IO_sub_temp.at(key).address] = value;
                        m_digital = true;
                    }
                    else if (m_IO_sub_temp.at(key).data_type == "analog")
                    {
                        temp_analog[m_IO_sub_temp.at(key).address] = value;
                        m_analog = true;
                    }
                    else
                    {
                        RCLCPP_WARN(get_logger(), "Unsupported output type for I/O %s, skipping", key.c_str());
                        publish_state(false, ModbusInterface::INVALID_IO_DATA_TYPE);
                        break;
                    }
                }
            }
        }
        catch (...) {
            RCLCPP_WARN(get_logger(), "Cannot write, skipping");
            publish_state(false, ModbusInterface::INVALID_IO_TO_WRITE);
        }
    }

    int count = 0;
    m_testd = true;
    m_testa = true;

    do // tries up to 3 times to send commands
    {
        count ++;
        if(m_digital)
        {
            m_testd = false;
            try {
                m_testd = m_modbus_device.setMultipleOutputCoils(temp_digital);
            } catch (...) {
            }
        }
    }
    while(count < 4 && !m_testd);
    count = 0;
    do
    {
        count ++;
        if(m_analog)
        {
            m_testa = false;
            try {
                m_testa = m_modbus_device.setMultipleOutputRegisters(temp_analog);
            } catch (...) {
            }
        }
    }
    while(count < 4 && !m_testa);

    if(!m_testa || !m_testd) // if we didn't managed to send the command
    {

        RCLCPP_WARN(get_logger(), "Cannot write, skipping");
        publish_state(false, ModbusInterface::INVALID_IO_TO_WRITE);
    }
 }

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node{std::make_shared<modbus_node::ModbusNode>()};

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
