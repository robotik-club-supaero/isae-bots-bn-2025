#ifndef _ROS_IMPL_MICRO_ROS_HPP_
#define _ROS_IMPL_MICRO_ROS_HPP_

#include "ros/micro_ros/Node.hpp"

#include "ros/micro_ros/std_msgs/Bool.hpp"
#include "ros/micro_ros/std_msgs/Empty.hpp"
#include "ros/micro_ros/std_msgs/Int16.hpp"

#include "ros/micro_ros/br_messages/Command.hpp"
#include "ros/micro_ros/br_messages/DisplacementOrder.hpp"
#include "ros/micro_ros/br_messages/GainsPid.hpp"
#include "ros/micro_ros/br_messages/LogEntry.hpp"
#include "ros/micro_ros/br_messages/OdosCount.hpp"
#include "ros/micro_ros/br_messages/Point.hpp"
#include "ros/micro_ros/br_messages/Position.hpp"

#include <micro_ros_platformio.h>

namespace ros2 {
inline void init(Stream &comm_instance) {
    set_microros_serial_transports(comm_instance);
}
} // namespace ros2

#endif