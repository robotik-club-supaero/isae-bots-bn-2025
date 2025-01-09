#include "motors/MotorStub.hpp"

MotorStub::MotorStub(std::shared_ptr<Speeds> speedsPtr) : m_speeds(std::move(speedsPtr)), m_isReady(false) {}

void MotorStub::sendCommand(Speeds speeds) {    
    m_speeds->linear = speeds.linear;
    m_speeds->angular = speeds.angular;
}

void MotorStub::switchOn() {
    m_isReady = true;
}

void MotorStub::switchOff() {
    m_isReady = false;
}

void MotorStub::update(double_t interval) {
    // Nothing to do
}

bool MotorStub::isReady() const {
    return m_isReady;
}
bool MotorStub::isIdle() const {
    return !m_isReady;
}
