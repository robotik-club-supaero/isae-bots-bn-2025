#ifndef _BUMPER_HPP_
#define _BUMPER_HPP_

#include "LowPassFilter.hpp"
#include "ros2/ros2.hpp"

/// Bumper with filtered state
class Bumper {
   public:
    Bumper(int pin, double filter_tau, unsigned long int update_interval);

    void loop();
    void reset();

    bool isPressed() const;
    /// 0 (not pressed) -> 1 (pressed for a long time)
    double getFilteredState() const;

   private:
    int m_pin;
    LowPassFilter m_filter;

    unsigned long int m_updateInterval;
    unsigned long int m_lastUpdate;
};

/// Connects ROS to the bumpers
class Bumpers {
   public:
    Bumpers(ros2::Node &node);
    Bumpers(ros2::Node &node, int pin1, int pin2, double filter_tau, unsigned long int update_interval, double detection_threshold);

    void loop();
    void reset();

    /// ((result & 0b01) != 0) => bumper 1 triggered
    /// ((result & 0b10) != 0) => bumper 2 triggered
    uint16_t getState() const;

   private:
    Bumper m_bump_1;
    Bumper m_bump_2;
    double m_threshold;
    uint16_t m_lastState;
    ros2::Publisher<std_msgs::Int16> m_publisher;
};

#endif