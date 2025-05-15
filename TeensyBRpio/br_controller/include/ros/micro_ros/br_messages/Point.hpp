#ifndef _BR_MESSAGES_MSG_POINT_HPP_
#define _BR_MESSAGES_MSG_POINT_HPP_

#include "br_messages/msg/point.h"
#include "ros/micro_ros/type_support.hpp"

namespace br_messages {
namespace msg {
class Point : public br_messages__msg__Point {
  public:
    Point() = default;
    Point(const br_messages__msg__Point &point) : Point(point.x, point.y) {}
    Point(number_t x, number_t y) {
        this->x = x;
        this->y = y;
    }

    static ros2::support_t type_support() { return ROSIDL_GET_MSG_TYPE_SUPPORT(br_messages, msg, Point); }
};

} // namespace msg

} // namespace br_messages

#endif