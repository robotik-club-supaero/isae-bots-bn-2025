#ifndef _SPEC_ROS_HPP_
#define _SPEC_ROS_HPP_

#include "specializations/actuators.hpp"
#include "specializations/feedback.hpp"

#include "ros/ROS.hpp"

#ifdef ARDUINO

#include <micro_ros_platformio.h>

#endif // #ifdef ARDUINO

using ros_t = ROS<actuators_t, feedback_t>;

#endif