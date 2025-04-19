#include "ActuatorServo.hpp"

#include <Arduino.h>

template <uint16_t N>
ActuatorServo<N>::ActuatorServo(int pin, std::array<int, N> positions, uint16_t initial_state)
    : m_state(), m_last_change(), m_need_notify(false), m_positions(positions), m_servo() {
    m_servo.attach(pin);
    setState(initial_state);
    markNotified();
}

template <uint16_t N>
uint16_t ActuatorServo<N>::getState() const {
    return m_state;
}

template <uint16_t N>
void ActuatorServo<N>::setState(uint16_t state) {
    m_last_change = micros();
    m_need_notify = true;

    m_state = state;
    m_servo.write(m_positions[state]);
}

template <uint16_t N>
bool ActuatorServo<N>::needsNotify() const {
    return m_need_notify;
}

template <uint16_t N>
unsigned long ActuatorServo<N>::getLastChangeTime() const {
    return m_last_change;
}

template <uint16_t N>
void ActuatorServo<N>::markNotified() {
    m_need_notify = false;
}

template class ActuatorServo<2>;