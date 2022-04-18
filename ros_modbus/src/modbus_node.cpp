// -*- lsst-c++ -*-

#include <modbus_node/modbus_node.h>

using namespace std::chrono_literals;
using namespace modbus_node;

ModbusNode::ModbusNode(rclcpp::NodeOptions options)
    : Node("modbus_node", options)
{
    // configuration of subscriber with it's callback group
    m_sub_option.callback_group = mp_callback_group_publisher;
    mp_subscriber = this->create_subscription<ros_modbus_msgs::msg::Modbus>("/ros_modbus/command", rclcpp::QoS(m_pub_queue_size), [this](ros_modbus_msgs::msg::Modbus::SharedPtr msg){subscriber_callback(msg);}, m_sub_option);

    // initializing messages
    m_msg_on_timer.header.set__frame_id(m_name);
    m_msg_on_timer.set__in_out(std::vector<std::string>());
    m_msg_on_timer.set__values(std::vector<uint16_t>());

    m_msg_on_event.header.set__frame_id(m_name);
    m_msg_on_event.set__in_out(std::vector<std::string>());
    m_msg_on_event.set__values(std::vector<uint16_t>());

    publish_state(false, INITIALIZING); // Currently not working, it appears that publisher is not fully initialized at this staged

    try
    {
        configure();
    }
    catch (...) {
        RCLCPP_ERROR(get_logger(), "Configuration file not valid, please provide a valid configuration file");
        publish_state(false, INVALID_CONFIGURATION_FILE);
    }

    mp_checker_timer = this->create_wall_timer(0s, [this](){check_timer_callback();}, mp_callback_group_checker);
    mp_checker_timer->cancel();
    mp_reconnection_timer = this->create_wall_timer(0s, [this](){restart_connection();}, mp_callback_group_reconnection);
    mp_reconnection_timer->cancel();
}

void ModbusNode::publish_state(bool state, int state_code)
{
    auto m_msg_state = ros_modbus_msgs::msg::State();
    m_msg_state.header.set__frame_id(m_name);
    m_msg_state.header.set__stamp(now());
    m_msg_state.set__state(state);
    m_msg_state.set__error(state_code);
    mp_state_publisher->publish(m_msg_state);
}

void ModbusNode::configure()
{
    YAML::Node config = YAML::LoadFile(m_YAML_config_file);
    if(config[m_name])
    {
        m_address = config[m_name]["address"].as<std::string>();
        m_port = config[m_name]["port"].as<int>();

        for (const auto &iter : config[m_name]["publish_on_timer"].as<std::vector<std::string>>())
        {
            m_publish_on_timer.insert({iter, 0});
            m_IO_list.insert(iter);
        }

        for (const auto &iter : config[m_name]["publish_on_event"].as<std::vector<std::string>>())
        {
            m_publish_on_event.insert({iter, 0});
            m_IO_list.insert(iter);
        }

        for(YAML::const_iterator element=config[m_name]["input"].begin();element!=config[m_name]["input"].end();++element)
        {
            for(YAML::const_iterator element2=element->second.begin();element2!=element->second.end();++element2)
            {
                m_IO_temp.type = "input";
                m_IO_temp.data_type = element->first.as<std::string>();
                if(element2->second.IsScalar())
                {
                    m_IO_temp.address = element2->second.as<int>()-1;
                }
                else
                {
                    m_IO_temp.address = -1;
                }
                m_IO_map.insert(std::pair(element2->first.as<std::string>(), m_IO_temp));
            }
        }

        for(YAML::const_iterator element=config[m_name]["output"].begin();element!=config[m_name]["output"].end();++element)
        {
            for(YAML::const_iterator element2=element->second.begin();element2!=element->second.end();++element2)
            {
                m_IO_temp.type = "output";
                m_IO_temp.data_type = element->first.as<std::string>();
                if(element2->second.IsScalar())
                {
                    m_IO_temp.address = element2->second.as<int>()-1;
                }
                else
                {
                    m_IO_temp.address = -1;
                }
                m_IO_map.insert(std::pair(element2->first.as<std::string>(), m_IO_temp));
            }
        }

        m_ctx = modbus_new_tcp(m_address.c_str(), m_port);

        RCLCPP_INFO(get_logger(),"Configuring device %s with address %s and port %d", m_name.c_str(), m_address.c_str(), m_port);
        m_connected = verify_connection();

        RCLCPP_INFO(get_logger(),"Verifying device %s's IO", m_name.c_str());
        m_configOK = verify_IO();
        if(!m_configOK)
        {
            publish_state(false, INVALID_CONFIGURATION_FILE);
        }
        else if (!m_connected)
        {
            publish_state(false, NOT_CONNECTED);
        }
        else
        {
            publish_state(true, NO_ISSUE);
        }

        mp_publisher_timer = this->create_wall_timer(std::chrono::milliseconds(int(1000./config[m_name]["publish_rate"].as<int>())), [this](){publish_timer_callback();}, mp_callback_group_publisher);
        mp_update_timer = this->create_wall_timer(std::chrono::milliseconds(int(1000./config[m_name]["refresh_rate"].as<int>())), [this](){update_timer_callback();}, mp_callback_group_update);

    }
}

bool ModbusNode::verify_connection()
{
    if (modbus_connect(m_ctx) == 0)
    {
        RCLCPP_INFO(get_logger(),"Connected to %s:%d successfully", m_address.c_str(), m_port);
        return true;
    }
    else
    {
        RCLCPP_WARN(get_logger(), "Connection to %s:%d failed", m_address.c_str(), m_port);
        return false;
    }


}

bool ModbusNode::verify_IO()
{
    for(auto &[key, value] : m_publish_on_event)
    {
        if(m_IO_map.find(key) == m_IO_map.end())
        {
            RCLCPP_INFO(get_logger(), "I/O %s is not provided in configuration file", key.c_str());
            return false;
        }
        else if(m_IO_map[key].data_type != "digital" && m_IO_map[key].type != "analog")
        {
            RCLCPP_INFO(get_logger(), "I/O %s is provided with incorrect I/O type (given %s expected digital or analog) in configuration file", key.c_str(), m_IO_map[key].type.c_str());
            return false;
        }
        else if(m_IO_map[key].address == -1)
        {
            RCLCPP_INFO(get_logger(), "I/O %s is provided with no address in configuration file", key.c_str());
            return false;
        }
    }
    for(auto &[key, value] : m_publish_on_timer)
    {
        if(m_IO_map.find(key) == m_IO_map.end())
        {
            RCLCPP_INFO(get_logger(), "I/O %s is not provided in configuration file", key.c_str());
            return false;
        }
        else if(m_IO_map[key].data_type != "digital" && m_IO_map[key].type != "analog")
        {
            RCLCPP_INFO(get_logger(), "I/O %s is provided with incorrect I/O type (given %s expected digital or analog) in configuration file", key.c_str(), m_IO_map[key].type.c_str());
            return false;
        }
        else if(m_IO_map[key].address == -1)
        {
            RCLCPP_INFO(get_logger(), "I/O %s is provided with no address in configuration file", key.c_str());
            return false;
        }
    }

    RCLCPP_INFO(get_logger(),"Configuration of %s appears valid successfully", m_name.c_str());
    return true;
}

void ModbusNode::restart_connection()
{
    m_ctx_guard.lock();
    modbus_close(m_ctx);
    while(modbus_connect(m_ctx) == -1)
    {
        m_ctx_guard.unlock();
        sleep(1);
        /**
         * @TODO find a way to make only this thread sleep
         */
        m_ctx_guard.lock();
    }
    m_ctx_guard.unlock();
    RCLCPP_INFO(get_logger(), "Reconnected to %s:%d", m_address.c_str(), m_port);
    m_connected = true;  
    publish_state(true, NO_ISSUE);
    mp_reconnection_timer->cancel();
}

void ModbusNode::update_timer_callback()
{
    mp_update_timer->cancel();
    try
    {
        for(auto key : m_IO_list)
        {
            bool result;
            m_IO_map_guard.lock();
            m_IO_update_temp = m_IO_map[key];
            m_IO_map_guard.unlock();
            if(m_IO_update_temp.type == "input")
            {
                if (m_IO_update_temp.data_type == "digital")
                {
                    m_ctx_guard.lock();
                    result = modbus_read_input_bits(m_ctx, m_IO_update_temp.address, 1, &m_temp_digit_value) == -1;
                    m_ctx_guard.unlock();
                    if (result)
                    {
                        throw MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE;
                    }
                }
                else if (m_IO_update_temp.data_type == "analog")
                {
                    m_ctx_guard.lock();
                    result = modbus_read_input_registers(m_ctx, m_IO_update_temp.address, 1, &m_update_temp_value) == -1;
                    m_ctx_guard.unlock();
                    if (result)
                    {
                            throw MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE;
                        }
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "Unsupported input type for I/O %s, skipping", key.c_str());
                    publish_state(false, INVALID_IO_DATA_TYPE);
                }
            }
            else if(m_IO_update_temp.type == "output")
            {
                if (m_IO_update_temp.data_type == "digital")
                {
                    m_ctx_guard.lock();
                    result = modbus_read_input_bits(m_ctx, m_IO_update_temp.address, 1, &m_temp_digit_value) == -1;
                    m_ctx_guard.unlock();
                    if (result)
                    {
                        throw MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE;
                    }
                    m_update_temp_value = m_temp_digit_value;
                }
                else if (m_IO_update_temp.data_type == "analog")
                {
                    m_ctx_guard.lock();
                    result = modbus_read_input_registers(m_ctx, m_IO_update_temp.address, 1, &m_update_temp_value) == -1;
                    m_ctx_guard.unlock();
                    if (result)
                    {
                        throw MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE;
                    }
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "Unsupported output type for I/O %s, skipping", key.c_str());
                    publish_state(false, INVALID_IO_DATA_TYPE);
                }
            }
            else
            {
                RCLCPP_WARN(get_logger(), "I/O %s is not set as input nor output, skipping", key.c_str());
                publish_state(false, INVALID_IO_TYPE);
            }

            m_IO_guard.lock();
            m_IO[key] = m_update_temp_value;
            m_IO_guard.unlock();
        }
    }
    catch(...)
    {
        if(!m_configOK)
        {
            RCLCPP_WARN(get_logger(), "Timer callback but configuration is not valid, reconfiguring");
            publish_state(false, INVALID_CONFIGURATION_FILE);
            configure();
        }
        else
        {
            m_connected = false;
            if(mp_reconnection_timer->is_canceled())
            {
                RCLCPP_WARN(get_logger(), "Connection to %s:%d lost, reconnecting", m_address.c_str(), m_port);
                publish_state(false, NOT_CONNECTED);
                mp_reconnection_timer->reset();
            }
        }
    }
    mp_checker_timer->reset();
    mp_update_timer->reset();
}

void ModbusNode::publish_timer_callback()
{
    m_msg_on_timer.header.set__stamp(now());
    m_msg_on_timer.set__values(std::vector<uint16_t>());
    m_msg_on_timer.set__in_out(std::vector<std::string>());

    for(auto &[key, value] : m_publish_on_event)
    {
        m_IO_guard.lock();
        m_publish_on_timer[key] = m_IO[key];
        m_IO_guard.unlock();
        m_msg_on_timer.in_out.push_back(key);
        m_msg_on_timer.values.push_back(m_publish_on_timer[key]);
    }
    mp_timer_publisher->publish(m_msg_on_timer);
}

void ModbusNode::check_timer_callback()
{
    mp_checker_timer->cancel();
    m_publish = false;
    for(auto &[key, value] : m_publish_on_event)
    {
        m_IO_guard.lock();
        m_checker_temp_value = m_IO[key];
        m_IO_guard.unlock();

        if(value != m_checker_temp_value)
        {
            m_publish_on_event[key] = m_checker_temp_value;

            if(!m_publish)
            {
                m_publish = true;
            }
        }
    }
    if (m_publish)
    {
        m_msg_on_event.header.set__stamp(now());
        m_msg_on_event.set__values(std::vector<uint16_t>());
        m_msg_on_event.set__in_out(std::vector<std::string>());
        for (auto &[key, value] : m_publish_on_event)
        {
            m_msg_on_event.in_out.push_back(key);
            m_msg_on_event.values.push_back(value);
        }
        mp_event_publisher->publish(m_msg_on_event);
    }
}

void ModbusNode::subscriber_callback(ros_modbus_msgs::msg::Modbus::SharedPtr p_msg)
{
    for(int iter=0; iter < p_msg->in_out.size(); iter++)
    {
        bool result;
        try {
            auto key = p_msg->in_out[iter];
            auto value = p_msg->values[iter];
            m_IO_map_guard.lock();
            m_IO_sub_temp = m_IO_map[key];
            m_IO_map_guard.unlock();
            if(m_IO_sub_temp.type == "input")
            {
                RCLCPP_WARN(get_logger(), "I/O %s is configured as input, can't write here, skipping", key.c_str());
                publish_state(false, INVALID_IO_TO_WRITE);
            }
            else if(m_IO_sub_temp.type == "output")
            {
                if (m_IO_sub_temp.data_type == "digital")
                {
                    m_ctx_guard.lock();
                    result = modbus_write_bit(m_ctx, m_IO_sub_temp.address, value) == -1;
                    m_ctx_guard.unlock();
                    if (result)
                    {
                        throw MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE;
                    }
                    m_update_temp_value = m_temp_digit_value;
                }
                else if (m_IO_sub_temp.data_type == "analog")
                {
                    m_ctx_guard.lock();
                    result = modbus_write_register(m_ctx, m_IO_sub_temp.address, value) == -1;
                    m_ctx_guard.unlock();
                    if (result)
                    {
                        throw MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE;
                    }
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "Unsupported output type for I/O %s, skipping", key.c_str());
                    publish_state(false, INVALID_IO_DATA_TYPE);
                }
            }

        } catch (...) {

            RCLCPP_WARN(get_logger(), "Unsupported output type for I/O , skipping");
            publish_state(false, NOT_CONNECTED);
        }
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
