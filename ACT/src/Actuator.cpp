#include "Actuator.hpp"

#include "configuration.hpp"
#include "logging.hpp"

template <uint16_t N>
ActuatorHandle<N>::ActuatorHandle(int servo_pin, const char *name, std::array<int, N> positions, uint16_t initial_state)
    : m_name(name), m_positions(positions), m_current_state(initial_state), m_last_change_time(), m_need_notify(), m_servo() {
    m_servo.attach(servo_pin);
    m_servo.write(m_positions[initial_state]);
}

template <uint16_t N>
uint16_t ActuatorHandle<N>::getState() const {
    return m_current_state;
}
template <uint16_t N>
bool ActuatorHandle<N>::setState(uint16_t state) {
    if (state < N) {
        log(INFO, String("Received order ").concat(state).concat(" for ").concat(m_name));

        if (state != m_current_state) {
            m_current_state = state;
            m_last_change_time = micros();
            m_need_notify = true;
        }

        m_servo.write(m_positions[state]);
        return true;
    } else {
        log(WARN, String("Invalid order received for ").concat(m_name));
        return false;
    }
}
template <uint16_t N>
unsigned long ActuatorHandle<N>::lastChangeTime() const {
    return m_last_change_time;
}

template <uint16_t N>
bool ActuatorHandle<N>::needsNotify() const {
    return m_need_notify;
}
template <uint16_t N>
void ActuatorHandle<N>::markNotified() {
    m_need_notify = false;
}

template <uint16_t N>
void ActuatorHandle<N>::setNeedNotify(bool need_notify) {
    m_need_notify = need_notify;
}

ActuatorTopics::ActuatorTopics(ros2::Node &node, const char *order_topic, const char *callback_topic, std::function<void(uint16_t)> callback)
    : m_callback(std::make_shared<std::function<void(uint16_t)>>(std::move(callback))),
      m_sub(node.createSubscriber<std_msgs::Int16>(order_topic, ActuatorTopics::orderCallback, m_callback.get())),
      m_pub(node.createPublisher<std_msgs::Int16>(callback_topic)),
      m_msg() {}

ActuatorTopics::~ActuatorTopics() { m_sub->destroy(); }

void ActuatorTopics::sendCallback(uint16_t data) {
    m_msg.data = data;
    m_pub->publish(&m_msg);
}
void ActuatorTopics::orderCallback(void *msg, void *arg) {
    std_msgs::Int16 *msg_casted = (std_msgs::Int16 *)msg;
    std::function<void(uint16_t)> *callback = (std::function<void(uint16_t)> *)arg;

    callback->operator()(msg_casted->data);
}

template class ActuatorHandle<2>;