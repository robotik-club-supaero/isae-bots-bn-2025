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
void ActuatorHandle<N>::setState(uint16_t state) {
    if (state < N) {
        log(INFO, String("Received order ").concat(state).concat(" on ").concat(m_name));

        m_current_state = state;
        m_last_change_time = micros();
        m_need_notify = true;

        m_servo.write(m_positions[state]);
    } else {
        log(WARN, String("Invalid order received on ").concat(m_name));
    }
}

template <uint16_t N>
Actuator<N>::Actuator(int servo_pin, ros2::Node &node, const char *order_topic, const char *callback_topic, std::array<int, N> positions,
                      uint16_t initial_state)
    : m_handle(std::make_shared<ActuatorHandle<N>>(servo_pin, callback_topic, positions, initial_state)),
      // The pointer allocated with `new` is never freed, but here that's actually what we want
      m_sub(node.createSubscriber<std_msgs::Int16>(order_topic, Actuator<N>::orderCallback, new std::weak_ptr<ActuatorHandle<N>>(m_handle))),
      m_pub(node.createPublisher<std_msgs::Int16>(callback_topic)),
      m_msg() {}

template <uint16_t N>
void Actuator<N>::loop() {
    if (m_handle->m_need_notify && micros() - m_handle->m_last_change_time > CALLBACK_INTERVAL) {
        m_msg.data = m_handle->m_current_state;
        m_pub->publish(&m_msg);
        m_handle->m_need_notify = false;
    }
}

template <uint16_t N>
void Actuator<N>::orderCallback(void *msg, void *arg) {
    std_msgs::Int16 *msg_casted = (std_msgs::Int16 *)msg;
    std::weak_ptr<ActuatorHandle<N>> *arg_casted = (std::weak_ptr<ActuatorHandle<N>> *)arg;

    if (auto handle = arg_casted->lock()) {
        handle->setState(msg_casted->data);
    }
}

template class Actuator<2>;