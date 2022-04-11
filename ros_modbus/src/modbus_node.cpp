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

    try {
       configure();
    }
    catch (...) {
        RCLCPP_ERROR(get_logger(), "Configuration file not valid, please provide a valid configuration file");
    }
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

        for(std::pair<std::string, std::map<std::string, int>> element : config[m_name]["input"].as<std::map<std::string, std::map<std::string, int>>>())
        {
            for(std::pair<std::string, int> element2 : element.second)
            {
                m_IO_temp.type = "input";
                m_IO_temp.data_type = element2.first;
                if(element2.second)
                {
                    m_IO_temp.address = element2.second-1;
                }
                else
                {
                    m_IO_temp.address = NAN;
                }
                m_IO.insert(std::pair(element2.first, m_IO_temp));
            }
        }

        for(std::pair<std::string, std::map<std::string, int>> element : config[m_name]["output"].as<std::map<std::string, std::map<std::string, int>>>())
        {
            for(std::pair<std::string, int> element2 : element.second)
            {
                m_IO_temp.type = "output";
                m_IO_temp.data_type = element2.first;
                if(element2.second)
                {
                    m_IO_temp.address = element2.second-1;
                }
                else
                {
                    m_IO_temp.address = NAN;
                }
                m_IO.insert(std::pair(element2.first, m_IO_temp));
            }
        }
    }

}

void ModbusNode::check_timer_callback()
{

};

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
