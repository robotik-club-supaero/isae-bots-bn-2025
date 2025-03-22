#ifndef _ROS_IMPL_HPP_

#ifdef ARDUINO
#include "ros/rclc/rclc_impl.hpp"
#else
#include "ros/rclcpp/rclcpp_impl.hpp"
#endif // #ifdef ARDUINO

DEFINE_ROS_MESSAGE(bool_t, std_msgs, Bool);
DEFINE_ROS_MESSAGE(empty_t, std_msgs, Empty);
DEFINE_ROS_MESSAGE(msg_int16_t, std_msgs, Int16);

DEFINE_ROS_MESSAGE(command_t, br_messages, Command);
DEFINE_ROS_MESSAGE(displacement_order_t, br_messages, DisplacementOrder);
DEFINE_ROS_MESSAGE(gains_t, br_messages, GainsPid);
DEFINE_ROS_MESSAGE(log_entry_t, br_messages, LogEntry);
DEFINE_ROS_MESSAGE(odos_count_t, br_messages, OdosCount);
DEFINE_ROS_MESSAGE(point_t, br_messages, Point);
DEFINE_ROS_MESSAGE(position_t, br_messages, Position);

#include "geometry/Position2D.hpp"
#include "ros/Callbacks.hpp"
#include "ros/message_cast.hpp"

#define CAST_POSITION(UNIT)                                                                                                                          \
    template <>                                                                                                                                      \
    inline Position2D<UNIT> message_cast<Position2D<UNIT>, ros_impl::messages::position_t>(const ros_impl::messages::position_t &msg) {              \
        return Position2D<UNIT>(static_cast<double_t>(msg.x), static_cast<double_t>(msg.y), static_cast<double_t>(msg.theta));                       \
    }                                                                                                                                                \
    template <>                                                                                                                                      \
    inline ros_impl::messages::position_t message_cast<ros_impl::messages::position_t, Position2D<UNIT>>(const Position2D<UNIT> &value) {            \
        ros_impl::messages::position_t msg;                                                                                                          \
        msg.x = static_cast<float>(value.x);                                                                                                         \
        msg.y = static_cast<float>(value.y);                                                                                                         \
        msg.theta = static_cast<float>(value.theta);                                                                                                 \
        return msg;                                                                                                                                  \
    }

#define CAST_POINT(UNIT)                                                                                                                             \
    template <>                                                                                                                                      \
    inline Point2D<UNIT> message_cast<Point2D<UNIT>, ros_impl::messages::point_t>(const ros_impl::messages::point_t &msg) {                          \
        return Point2D<UNIT>(static_cast<double_t>(msg.x), static_cast<double_t>(msg.y));                                                            \
    }                                                                                                                                                \
    template <>                                                                                                                                      \
    inline ros_impl::messages::point_t message_cast<ros_impl::messages::point_t, Point2D<UNIT>>(const Point2D<UNIT> &value) {                        \
        ros_impl::messages::point_t msg;                                                                                                          \
        msg.x = static_cast<float>(value.x);                                                                                                         \
        msg.y = static_cast<float>(value.y);                                                                                                         \
        return msg;                                                                                                                                  \
    }

#define CAST_WRAPPER(I, O)                                                                                                                           \
    template <>                                                                                                                                      \
    inline const O &message_cast<const O &, I>(const I &msg) {                                                                                       \
        return msg.data;                                                                                                                             \
    }                                                                                                                                                \
    template <>                                                                                                                                      \
    inline I message_cast<I, O>(const O &data) {                                                                                                     \
        I msg;                                                                                                                                       \
        msg.data = data;                                                                                                                             \
        return msg;                                                                                                                                  \
    }

CAST_POSITION(Meter);
CAST_POSITION(Millimeter);
CAST_POINT(Meter);
CAST_POINT(Millimeter);

template <>
inline Speeds message_cast<Speeds, ros_impl::messages::command_t>(const ros_impl::messages::command_t &msg) {
    return Speeds(static_cast<double_t>(msg.linear), static_cast<double_t>(msg.angular));
}
template <>
inline ros_impl::messages::command_t message_cast<ros_impl::messages::command_t, Speeds>(const Speeds &value) {
    ros_impl::messages::command_t msg;
    msg.linear = static_cast<float>(value.linear);
    msg.angular = static_cast<float>(value.angular);
    return msg;
}

CAST_WRAPPER(ros_impl::messages::msg_int16_t, int16_t);
CAST_WRAPPER(ros_impl::messages::bool_t, bool);

template <>
inline ros_impl::messages::msg_int16_t message_cast<ros_impl::messages::msg_int16_t, AsservCallback>(const AsservCallback &callback) {
    return message_cast<ros_impl::messages::msg_int16_t>(static_cast<int16_t>(callback));
}

#endif