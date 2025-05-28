#include "Bumper.hpp"

#include <Arduino.h>

#include "configuration.hpp"

Bumper::Bumper(int pin, double filter_tau, unsigned long int update_interval)
    : m_pin(pin), m_filter(filter_tau), m_updateInterval(update_interval), m_lastUpdate(millis()) {
    pinMode(pin, INPUT);
}

void Bumper::loop() {
    // `millis` overflows after ~50 days - this definitely won't happen
    unsigned long int now = millis();
    unsigned int num_of_steps = (now - m_lastUpdate) / m_updateInterval;

    for (unsigned int i = 0; i < num_of_steps; i++) {
        m_lastUpdate += m_updateInterval;
        m_filter.update(isPressed() ? 1. : 0., m_updateInterval);
    }
}
void Bumper::reset() {
    m_lastUpdate = millis();
    m_filter.reset();
}

bool Bumper::isPressed() const { return digitalRead(m_pin) == HIGH; }

double Bumper::getFilteredState() const { return m_filter.value(); }

Bumpers::Bumpers(ros2::Node &node)
    : Bumpers(node, BUMPER_1_PIN, BUMPER_2_PIN, BUMPER_FILTER_TAU, BUMPER_UPDATE_INTERVAL, BUMPER_DETECTION_THRESHOLD) {}
Bumpers::Bumpers(ros2::Node &node, int pin1, int pin2, double filter_tau, unsigned long int update_interval, double detection_threshold)
    : m_bump_1(pin1, filter_tau, update_interval),
      m_bump_2(pin2, filter_tau, update_interval),
      m_threshold(detection_threshold),
      m_lastState(0),
      m_publisher(node.createPublisher<std_msgs::Int16>("/act/bumpers")) {}

void Bumpers::loop() {
    m_bump_1.loop();
    m_bump_2.loop();

    uint16_t state = getState();
    if (state != m_lastState) {
        m_lastState = state;
        m_publisher.publish(std_msgs::Int16(state));
    }
}

void Bumpers::reset() {
    m_bump_1.reset();
    m_bump_2.reset();
}

uint16_t Bumpers::getState() const {
    uint16_t result = 0;
    if (m_bump_1.getFilteredState() > m_threshold) {
        result |= 0b01;
    }
    if (m_bump_2.getFilteredState() > m_threshold) {
        result |= 0b10;
    }
    return result;
}