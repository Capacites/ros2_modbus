#include <modbus_node/modbus_node.h>

using namespace std::chrono_literals;
using namespace modbus_node;

ModbusNode::ModbusNode(rclcpp::NodeOptions options)
    : Node("modbus_node", options)
{
    m_msg_on_timer.header.set__frame_id("Modbus_report");
    m_msg_on_timer.set__in_out(std::vector<std::string>());
    m_msg_on_timer.set__values(std::vector<uint16_t>());

    m_msg_on_event.header.set__frame_id("Modbus_report");
    m_msg_on_event.set__in_out(std::vector<std::string>());
    m_msg_on_event.set__values(std::vector<uint16_t>());

    m_checker_timer = this->create_wall_timer(1ms, std::bind(&ModbusNode::check_timer_callback, this));

    //try {
       configure();
    //}
    //catch (...) {
    //    RCLCPP_ERROR(get_logger(), "Configuration file not valid, please provide a valid configuration file");
    //}
}


void ModbusNode::configure()
{
    YAML::Node config = YAML::LoadFile(m_YAML_config_file);
    if(config[m_name])
    {
        if(m_sock)
        {
            close(m_sock);
        }
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
                    m_IO_temp.address = element2->second.as<int>()-1;
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
                    m_IO_temp.address = element2->second.as<int>()-1;
                }
                else
                {
                    m_IO_temp.address = -1;
                }
                m_IO.insert(std::pair(element2->first.as<std::string>(), m_IO_temp));
                m_IO_as_str += element2->first.as<std::string>() + " ";
            }
        }

        m_server.sin_addr.s_addr = inet_addr(m_address.c_str());
        m_server.sin_family = AF_INET;
        m_server.sin_port = htons(m_port);

        RCLCPP_INFO(get_logger(),"Configuring device %s with address %s and port %d", m_name.c_str(), m_address.c_str(), m_port);
        RCLCPP_INFO(get_logger(),"Configuring device %s with IO %s", m_name.c_str(), m_IO_as_str.c_str());
        m_configOK = verify();
        if(m_configOK)
        {
            RCLCPP_INFO(get_logger(),"Connected to %s:%d", m_address.c_str(), m_port);
            RCLCPP_INFO(get_logger(),"Configured %s successfully", m_name.c_str());
            m_connected = true;
        }




    }
}

bool ModbusNode::verify()
{
    if( connect(m_sock, (struct sockaddr*) &m_server, sizeof(m_server)) !=0 )
    {
           RCLCPP_INFO(get_logger(),"%s:%d is not a valid server address or server is down", m_address.c_str(), m_port);
           return false;
    }
    else
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
    }
    return true;
}

void ModbusNode::check_timer_callback()
{
    configure();
}

void ModbusNode::publish_timer_callback()
{

}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<modbus_node::ModbusNode>());
  rclcpp::shutdown();
  return 0;
}
