#ifndef _ACTUATORS_HPP_
#define _ACTUATORS_HPP_

#include <memory>

#include "Servo.h"
#include "ros2arduino.h"

template <uint16_t N = 2>
class Actuator;

template <uint16_t N>
class ActuatorHandle {
   public:
    ActuatorHandle(int servo_pin, const char *name, std::array<int, N> positions, uint16_t initial_state);
    void setState(uint16_t state);

    const char *m_name;
    std::array<int, N> m_positions;

    uint16_t m_current_state;
    unsigned long m_last_change_time;
    bool m_need_notify;

    Servo m_servo;
};

template <uint16_t N>
class Actuator {
   public:
    Actuator(int servo_pin, ros2::Node &node, const char *order_topic, const char *callback_topic, std::array<int, N> positions,
             uint16_t initial_state = 0);
    void loop();

   protected:
   private:
    static void orderCallback(void *msg, void *arg);

    std::shared_ptr<ActuatorHandle<N>> m_handle;

    ros2::Subscriber<std_msgs::Int16> *m_sub;
    ros2::Publisher<std_msgs::Int16> *m_pub;
    std_msgs::Int16 m_msg;
};

#endif