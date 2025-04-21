#ifndef _ACTUATOR_SERVO_HPP_
#define _ACTUATOR_SERVO_HPP_

#include <Servo.h>

#include <array>

/// Converts between states (CLOSED/OPEN, etc.) and Servo positions.
/// `N` is the number of states (it is always 2 currently).
template <uint16_t N>
class ActuatorServo {
   public:
    ActuatorServo(int pin, std::array<int, N> positions, uint16_t initial_state = 0);

    uint16_t getState() const;
    void setState(uint16_t state);
    void toggle();

    bool needsNotify() const;
    unsigned long getLastChangeTime() const;
    void markNotified();

   private:
    uint16_t m_state;
    unsigned long m_last_change;
    bool m_need_notify;

    std::array<int, N> m_positions;

    Servo m_servo;
};

#endif