#ifndef _ROS_IMPL_TYPE_SUPPORT_HPP_
#define _ROS_IMPL_TYPE_SUPPORT_HPP_

#include <rcl_interfaces/msg/log.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

namespace ros_rclc {

using support_t = const ::rosidl_message_type_support_t *const;

template <typename T>
class type_support_t;

} // namespace ros_rclc

#endif