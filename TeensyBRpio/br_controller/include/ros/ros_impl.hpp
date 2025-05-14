#ifndef _ROS_IMPL_HPP_
#define _ROS_IMPL_HPP_

#ifdef ARDUINO
#include "ros/micro_ros/ros_impl.hpp"
#else
#include "ros/rclcpp/ros_impl.hpp"
#endif

namespace std_msgs {
inline msg::Bool make_message(bool data) {
    msg::Bool message;
    message.data = data;
    return message;
}
inline msg::Int16 make_message(int16_t data) {
    msg::Int16 message;
    message.data = data;
    return message;
}
} // namespace std_msgs

namespace br_messages {

template <typename Unit>
inline Position2D<Unit> position_cast(const msg::Position &msg) {
    return Position2D<Millimeter>(msg.x, msg.y, msg.theta).convert<Unit>();
}

template <typename Unit>
inline msg::Position position_cast(const Position2D<Unit> &position) {
    Position2D<Millimeter> position_millis = position.toMillimeters();

    msg::Position message;
    message.x = position_millis.x;
    message.y = position_millis.y;
    message.theta = position_millis.theta;
    return message;
}

template <typename Unit>
inline Point2D<Unit> point_cast(const msg::Point &msg) {
    return Point2D<Millimeter>(msg.x, msg.y).convert<Unit>();
}

template <typename Unit>
inline msg::Point point_cast(const Point2D<Unit> &point) {
    Point2D<Millimeter> point_millis = point.toMillimeters();

    msg::Point message;
    message.x = point_millis.x;
    message.y = point_millis.y;
    return message;
}

inline Speeds command_cast(const msg::Command &msg) {
    return Speeds(msg.linear, msg.angular);
}

inline msg::Command command_cast(const Speeds &speed) {
    msg::Command message;
    message.linear = speed.linear;
    message.angular = speed.angular;
    return message;
}

inline msg::OdosCount make_odos_count(int32_t left, int32_t right) {
    msg::OdosCount message;
    message.left = left;
    message.right = right;
    return message;
}

class PathView {
    const __detail::__PathView m_inner;

  public:
    class Iterator {
        __detail::__PathView::iterator m_inner;
        friend class PathView;

        Iterator(__detail::__PathView::iterator &&inner) : m_inner(std::move(inner)) {}

      public:
        Point2D<Meter> operator*() const { return point_cast<Meter>(*m_inner); }
        Iterator &operator++() {
            ++m_inner;
            return *this;
        }
        Iterator operator++(int) { return Iterator(m_inner++); }
        bool operator==(const Iterator &other) const { return m_inner == other.m_inner; }
        bool operator!=(const Iterator &other) const { return m_inner != other.m_inner; }
    };

    using iterator = Iterator;

    PathView(__detail::__PathView &&path) : m_inner(std::move(path)) {}

    iterator begin() const { return m_inner.begin(); }
    iterator end() const { return m_inner.end(); }
    std::size_t size() const { return m_inner.size(); }
    bool empty() const { return m_inner.empty(); }
    Point2D<Meter> operator[](std::size_t i) const { return point_cast<Meter>(m_inner[i]); }
};

inline PathView getPathView(const msg::DisplacementOrder &order) {
    return PathView(__detail::__PathView(order.path));
}

} // namespace br_messages

namespace ros2 {}

#endif