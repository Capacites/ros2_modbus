#ifndef MODBUS_NODE_H
#define MODBUS_NODE_H

#include <rclcpp/rclcpp.hpp>

namespace modbus {

class ModbusNode : public rclcpp::Node
{
public:
    ModbusNode(rclcpp::NodeOptions options = rclcpp::NodeOptions());

};
}

#endif
