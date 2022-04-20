# Modbus_ROS2

## Overview

A ROS 2 package for Modbus with TCP

**Keywords:** ROS 2, Modbus, Modbus TCP

### License

The source code is released under a [MIT license](LICENSE).

## Installation

#### Dependencies

- [Robot Operating System (ROS) 2](http://wiki.ros.org) (middleware for robotics),
- [yaml-cpp](https://github.com/jbeder/yaml-cpp) as submodule,
- [libmodbus](https://github.com/stephane/libmodbus) (Modbus library)


#### Building

Clone with `--recursive` in order to get the necessary `yaml-cpp` library:

	cd ros2_workspace/src
	git clone TODO -b main --recursive
	cd ../..
	colcon build --symlink-install --packages-select ros_modbus

## Node

The package provides a C++ node: `ros2 run ros_modbus modbus_node`

While continuously updating the state of the Modbus device's IOs, the node may publish two types of messages:
- The IO's state (on timer or on event depending on the configuration file)
- The node state (indicating wether published data should be considered valid or not, and a debug code)
