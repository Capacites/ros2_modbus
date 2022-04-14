#include <modbus_node/modbus_node.h>

#define NO_ISSUE 0
#define INITIALIZING 1
#define NOT_CONNECTED 2
#define INVALID_CONFIGURATION_FILE 3
#define INVALID_IO_TYPE 4
#define INVALID_IO_DATA_TYPE 5
#define INVALID_IO_KEY 6
#define INVALID_IO_TO_WRITE 7

using namespace std::chrono_literals;
using namespace modbus_node;

ModbusNode::ModbusNode(rclcpp::NodeOptions options)
    : Node("modbus_node", options)
{
    m_msg_on_timer.header.set__frame_id(m_name);
    m_msg_on_timer.set__in_out(std::vector<std::string>());
    m_msg_on_timer.set__values(std::vector<uint16_t>());

    m_msg_on_event.header.set__frame_id(m_name);
    m_msg_on_event.set__in_out(std::vector<std::string>());
    m_msg_on_event.set__values(std::vector<uint16_t>());

    m_checker_timer = this->create_wall_timer(1s, [this](){check_timer_callback();});
    m_checker_timer->cancel();

    m_timer_publisher = this->create_publisher<ros_modbus_msgs::msg::Modbus>("/ros_modbus/report_timer", rclcpp::QoS(m_pub_queue_size));
    m_event_publisher = this->create_publisher<ros_modbus_msgs::msg::Modbus>("/ros_modbus/report_event", rclcpp::QoS(m_pub_queue_size));
    m_state_publisher = this->create_publisher<ros_modbus_msgs::msg::State>("/ros_modbus/state", rclcpp::QoS(m_pub_queue_size));
    m_subscriber = this->create_subscription<ros_modbus_msgs::msg::Modbus>("/ros_modbus/command", rclcpp::QoS(m_pub_queue_size), [this](ros_modbus_msgs::msg::Modbus::SharedPtr msg){subscriber_callback(msg);});

    try {
        publish_state(false, INITIALIZING);
        configure();
    }
    catch (...) {
        RCLCPP_ERROR(get_logger(), "Configuration file not valid, please provide a valid configuration file");
        publish_state(false, INVALID_CONFIGURATION_FILE);
    }
}

void ModbusNode::publish_state(bool state, int state_code)
{
    auto m_msg_state = ros_modbus_msgs::msg::State();
    m_msg_state.header.set__frame_id(m_name);
    m_msg_state.header.set__stamp(now());
    m_msg_state.set__state(state);
    m_msg_state.set__error(state_code);
    m_state_publisher->publish(m_msg_state);
}

void ModbusNode::configure()
{
    YAML::Node config = YAML::LoadFile(m_YAML_config_file);
    if(config[m_name])
    {
        m_address = config[m_name]["address"].as<std::string>();
        m_port = config[m_name]["port"].as<int>();

        m_publisher_timer = this->create_wall_timer(std::chrono::seconds(int(1./config[m_name]["publish_rate"].as<int>())), [this](){publish_timer_callback();});
        m_update_timer = this->create_wall_timer(std::chrono::seconds(int(1./config[m_name]["refresh_rate"].as<int>())), [this](){update_timer_callback();});

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

        m_checker_timer->reset();

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
    modbus_close(m_ctx);
    while(modbus_connect(m_ctx) == -1)
    {
    }
    RCLCPP_INFO(get_logger(), "Reconnected to %s:%d", m_address.c_str(), m_port);
    m_connected = true;  
    publish_state(true, NO_ISSUE);
}

void ModbusNode::update_timer_callback()
{
    m_checker_timer->cancel();
    try
    {
        for(auto key : m_IO_list)
        {
            m_IO_map_guard.lock();
            m_IO_update_temp = m_IO_map[key];
            m_IO_map_guard.unlock();

            if(m_IO_update_temp.type == "input")
            {
                if (m_IO_update_temp.data_type == "digital")
                {
                        if (modbus_read_input_bits(m_ctx, m_IO_update_temp.address, 1, &m_temp_digit_value) == -1)
                        {
                            throw MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE;
                        }

                }
                else if (m_IO_update_temp.data_type == "analog")
                {
                        if (modbus_read_input_registers(m_ctx, m_IO_update_temp.address, 1, &m_update_temp_value) == -1)
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
                        if (modbus_read_input_bits(m_ctx, m_IO_update_temp.address, 1, &m_temp_digit_value) == -1)
                        {
                            throw MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE;
                        }
                        m_update_temp_value = m_temp_digit_value;
                }
                else if (m_IO_update_temp.data_type == "analog")
                {
                        if (modbus_read_input_registers(m_ctx, m_IO_update_temp.address, 1, &m_update_temp_value) == -1)
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
        m_checker_timer->execute_callback();
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
            RCLCPP_WARN(get_logger(), "Connection to %s:%d lost, reconnecting", m_address.c_str(), m_port);
            publish_state(false, NOT_CONNECTED);
            restart_connection();
        }
    }
    m_checker_timer->reset();
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
    m_timer_publisher->publish(m_msg_on_timer);
}

void ModbusNode::check_timer_callback()
{
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
        m_event_publisher->publish(m_msg_on_event);
    }
}


void ModbusNode::subscriber_callback(ros_modbus_msgs::msg::Modbus::SharedPtr msg)
{
        for(int iter=0; iter <= int(size(m_msg_recv_temp.in_out)); iter++)
        {
            //try {
                auto key = msg->in_out[iter];
                auto value = msg->values[iter];
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
                            if (modbus_write_bit(m_ctx, m_IO_sub_temp.address, value) == -1)
                            {
                                throw MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE;
                            }
                            m_update_temp_value = m_temp_digit_value;
                    }
                    else if (m_IO_sub_temp.data_type == "analog")
                    {
                            if (modbus_write_register(m_ctx, m_IO_sub_temp.address, value) == -1)
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

            //} catch (...) {

            //}
        }
    }


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<modbus_node::ModbusNode>());
  rclcpp::shutdown();
  return 0;
}
