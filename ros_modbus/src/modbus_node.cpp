#include <modbus_node/modbus_node.h>

using namespace std::chrono_literals;
using namespace modbus_node;
using ros_modbus_msgs::msg::Modbus;

ModbusNode::ModbusNode(rclcpp::NodeOptions options)
    : Node("modbus_node", options)
{
    m_msg_on_timer.header.set__frame_id("Modbus_report");
    m_msg_on_timer.set__in_out(std::vector<std::string>());
    m_msg_on_timer.set__values(std::vector<uint16_t>());

    m_msg_on_event.header.set__frame_id("Modbus_report");
    m_msg_on_event.set__in_out(std::vector<std::string>());
    m_msg_on_event.set__values(std::vector<uint16_t>());

    m_checker_timer = this->create_wall_timer(100ms, std::bind(&ModbusNode::check_timer_callback, this));

    m_publisher = this->create_publisher<Modbus>("/ros_modbus/report", rclcpp::QoS(m_pub_queue_size));
    m_subscriber = this->create_subscription<Modbus>("/ros_modbus/command", rclcpp::QoS(m_pub_queue_size), std::bind(&ModbusNode::subscriber_callback, this, std::placeholders::_1));

    //try {
       configure();
    //}
    //catch (...) {
    //    RCLCPP_ERROR(get_logger(), "Configuration file not valid, please provide a valid configuration file"no);
    //}
}


void ModbusNode::configure()
{
    YAML::Node config = YAML::LoadFile(m_YAML_config_file);
    if(config[m_name])
    {
        m_address = config[m_name]["address"].as<std::string>();
        m_port = config[m_name]["port"].as<int>();

        m_publisher_timer = this->create_wall_timer(std::chrono::seconds(1/config[m_name]["publish_frequency"].as<int>()), std::bind(&ModbusNode::publish_timer_callback, this));

        for (const auto &iter : config[m_name]["publish_on_timer"].as<std::vector<std::string>>())
        {
            m_publish_on_timer.insert({iter, 0});
        }

        for (const auto &iter : config[m_name]["publish_on_event"].as<std::vector<std::string>>())
        {
            m_publish_on_event.insert({iter, 0});
        }

        m_IO_as_str = "";
        for(YAML::const_iterator element=config[m_name]["input"].begin();element!=config[m_name]["input"].end();++element)
        {
            for(YAML::const_iterator element2=element->second.begin();element2!=element->second.end();++element2)
            {
                m_IO_temp.type = "input";
                m_IO_temp.data_type = element->first.as<std::string>();
                if(element2->second.IsScalar())
                {
                    m_IO_temp.address = element2->second.as<int>();
                }
                else
                {
                    m_IO_temp.address = -1;
                }
                m_IO.insert(std::pair(element2->first.as<std::string>(), m_IO_temp));
                m_IO_as_str += element2->first.as<std::string>() + " ";

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
                    m_IO_temp.address = element2->second.as<int>();
                }
                else
                {
                    m_IO_temp.address = -1;
                }
                m_IO.insert(std::pair(element2->first.as<std::string>(), m_IO_temp));
                m_IO_as_str += element2->first.as<std::string>() + " ";
            }
        }

        m_ctx = modbus_new_tcp(m_address.c_str(), m_port);

        m_checker_timer->reset();

        RCLCPP_INFO(get_logger(),"Configuring device %s with address %s and port %d", m_name.c_str(), m_address.c_str(), m_port);
        m_connected = verify_connection();

        RCLCPP_INFO(get_logger(),"Configuring device %s with IO %s", m_name.c_str(), m_IO_as_str.c_str());
        m_configOK = verify_IO();
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
        RCLCPP_WARN(get_logger(), "Connection to %s:%d failed, reconnecting", m_address.c_str(), m_port);
        return false;
    }


}

bool ModbusNode::verify_IO()
{
    for(auto &[key, value] : m_publish_on_event)
    {
        if(m_IO.find(key) == m_IO.end())
        {
            RCLCPP_INFO(get_logger(), "I/O %s is not provided in configuration file", key.c_str());
            return false;
        }
        else if(m_IO[key].data_type != "digital" && m_IO[key].type != "analog")
        {
            RCLCPP_INFO(get_logger(), "I/O %s is provided with incorrect I/O type (given %s expected digital or analog) in configuration file", key.c_str(), m_IO[key].type.c_str());
            return false;
        }
        else if(m_IO[key].address == -1)
        {
            RCLCPP_INFO(get_logger(), "I/O %s is provided with no address in configuration file", key.c_str());
            return false;
        }
    }
    for(auto &[key, value] : m_publish_on_timer)
    {
        if(m_IO.find(key) == m_IO.end())
        {
            RCLCPP_INFO(get_logger(), "I/O %s is not provided in configuration file", key.c_str());
            return false;
        }
        else if(m_IO[key].data_type != "digital" && m_IO[key].type != "analog")
        {
            RCLCPP_INFO(get_logger(), "I/O %s is provided with incorrect I/O type (given %s expected digital or analog) in configuration file", key.c_str(), m_IO[key].type.c_str());
            return false;
        }
        else if(m_IO[key].address == -1)
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
    while(modbus_connect(m_ctx) != 0)
    {
    }
    RCLCPP_INFO(get_logger(), "Reconnected to %s:%d", m_address.c_str(), m_port);
    m_connected = true;
}

void ModbusNode::check_timer_callback()
{
    m_checker_timer->cancel();
    try
    {
        m_IO_map_guard.lock();
        for(auto &[key, value] : m_IO_map)
        {
            if(m_IO[key].type == "input")
            {
                if (m_IO[key].data_type == "digital")
                {
                        if (modbus_read_input_bits(m_ctx, m_IO[key].address, 1, &m_temp_digit_value) == -1)
                        {
                            throw MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE;
                        }

                }
                else if (m_IO[key].data_type == "analog")
                {
                        if (modbus_read_input_registers(m_ctx, m_IO[key].address, 1, &m_temp_value) == -1)
                        {
                            throw MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE;
                        }
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "Unsupported input type for I/O %s, skipping", key.c_str());
                }
            }
            else if(m_IO[key].type == "outpu")
            {
                if (m_IO[key].data_type == "digital")
                {
                        if (modbus_read_input_bits(m_ctx, m_IO[key].address, 1, &m_temp_digit_value) == -1)
                        {
                            throw MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE;
                        }
                        m_temp_value = m_temp_digit_value;
                }
                else if (m_IO[key].data_type == "analog")
                {
                        if (modbus_read_input_registers(m_ctx, m_IO[key].address, 1, &m_temp_value) == -1)
                        {
                            throw MODBUS_EXCEPTION_SLAVE_OR_SERVER_FAILURE;
                        }
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "Unsupported output type for I/O %s, skipping", key.c_str());
                }
            }
            else
            {
                RCLCPP_WARN(get_logger(), "I/O %s is not set as input nor output, skipping", key.c_str());
            }
            if(m_publish_on_event[key] != m_temp_value)
            {
                m_publish_on_event[key] = m_temp_value;
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
            m_publisher->publish(m_msg_on_event);
            m_publish = false;
        }
        m_IO_map_guard.unlock();
    }
    catch(...)
    {
        if(!m_configOK)
        {
            RCLCPP_WARN(get_logger(), "Timer callback but configuration is not valid, reconfiguring");
            configure();
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Connection to %s:%d lost, reconnecting", m_address.c_str(), m_port);
            restart_connection();
        }
    }
    m_checker_timer->reset();
}

void ModbusNode::publish_timer_callback()
{

}

void ModbusNode::subscriber_callback(ros_modbus_msgs::msg::Modbus::SharedPtr msg)
{

}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<modbus_node::ModbusNode>());
  rclcpp::shutdown();
  return 0;
}
