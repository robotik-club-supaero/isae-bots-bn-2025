#ifndef _ROS_LOAD_HPP_
#define _ROS_LOAD_HPP_

#include <micro_ros_platformio.h>

#include "ros2/Node.hpp"
#include "ros2/std_msgs/Empty.hpp"
#include "ros2/std_msgs/Int16.hpp"
#include "ros2/std_msgs/String.hpp"

/// Basic C++ interface over micro_ros and RCLC.
/// Only the stuff needed by this project is implemented.
namespace ros2 {

inline void init(Stream& comm_instance) { set_microros_serial_transports(comm_instance); }
inline void spin(Node& node) { node.spin(); }

}  // namespace ros2

#endif