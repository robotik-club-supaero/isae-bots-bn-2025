#ifndef _ROS_IMPL_HPP_
#define _ROS_IMPL_HPP_

#include "ros/rclc/Node.hpp"
#include "ros/rclc/type_support.hpp"

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/empty.h>
#include <std_msgs/msg/int16.h>

#include "br_messages/msg/command.h"
#include "br_messages/msg/displacement_order.h"
#include "br_messages/msg/gains_pid.h"
#include "br_messages/msg/log_entry.h"
#include "br_messages/msg/odos_count.h"
#include "br_messages/msg/point.h"
#include "br_messages/msg/position.h"

#include <span>

namespace ros_impl {
using namespace ros_rclc;

using node_t = Node;

template <typename T>
using publisher_t = Publisher<T>;

template <typename T>
using subscription_t = Subscription<T>;
} // namespace ros_impl

#define DEFINE_ROS_MESSAGE(NAME, PACKAGE, MSG)                                                                                                       \
    namespace ros_impl {                                                                                                                             \
    namespace messages {                                                                                                                             \
    using NAME = PACKAGE##__msg__##MSG;                                                                                                              \
    }                                                                                                                                                \
    }                                                                                                                                                \
    namespace ros_rclc {                                                                                                                             \
    template <>                                                                                                                                      \
    class type_support_t<::ros_impl::messages::NAME> {                                                                                               \
      public:                                                                                                                                        \
        static support_t get() {                                                                                                                     \
            return ROSIDL_GET_MSG_TYPE_SUPPORT(PACKAGE, msg, MSG);                                                                                   \
        }                                                                                                                                            \
    };                                                                                                                                               \
    } // namespace ros_rclc

template <>
inline std::span<const br_messages__msg__Point>
message_cast<std::span<const br_messages__msg__Point>, br_messages__msg__Point__Sequence>(const br_messages__msg__Point__Sequence &msg) {
    return std::span(msg.data, msg.size);
}

#endif