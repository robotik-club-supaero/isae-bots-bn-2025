#ifndef _ROS_MSG_CAST_HPP_
#define _ROS_MSG_CAST_HPP_

/// Conversion between ROS messages types and the implementation-agnostic types used in this project
template <typename O, typename I>
O message_cast(const I &msg) {
    return msg;
}

#endif