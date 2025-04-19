#ifndef _ROS_MSG_STRING_HPP_
#define _ROS_MSG_STRING_HPP_

#include <Arduino.h>
#include <build/mcu/install/include/rosidl_runtime_c/rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/string.h>

#include "ros2/std_msgs/__defines.hpp"

namespace std_msgs {

struct String : public std_msgs__msg__String {
   public:
    String() : std_msgs__msg__String() { rosidl_runtime_c__String__init(&this->data); }
    String(const ::String data) : String() { setData(data); }
    ~String() { rosidl_runtime_c__String__fini(&this->data); }

    const char* getData() const { return this->data.data; }
    size_t length() const { return this->data.size; }
    void setData(const ::String& data) { rosidl_runtime_c__String__assignn(&this->data, data.c_str(), data.length()); }

    static support_t type_support() { return ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String); }
};

}  // namespace std_msgs

#endif