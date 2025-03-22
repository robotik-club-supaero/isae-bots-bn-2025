#ifndef _ROS_MSG_CAST_HPP_
#define _ROS_MSG_CAST_HPP_

template <typename O, typename I>
O message_cast(const I &msg) {
    return msg;
}

#endif