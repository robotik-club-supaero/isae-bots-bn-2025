#include "Elevator.hpp"

#include "configuration.hpp"
#include "logging.hpp"

// TODO what should be the initial state?
ElevatorStepper::ElevatorStepper(int number_of_steps, int pin1, int pin2, int level, long speed, int move_steps)
    : m_stepper(number_of_steps, pin1, pin2), m_level(level), m_steps(move_steps), m_state(DOWN) {
    m_stepper.setSpeed(speed);
}

ElevatorCallback ElevatorStepper::getState() const { return m_state; }
void ElevatorStepper::setState(uint16_t state) {
    switch (state) {
        case UP:
            log(INFO, String("Moving elevator ").concat(m_level).concat(" UP"));
            m_stepper.step(m_steps);
            m_state = UP;
            break;
        case DOWN:

            log(INFO, String("Moving elevator ").concat(m_level).concat(" DOWN"));
            m_stepper.step(-m_steps);
            m_state = DOWN;
            break;
        default:
            log(WARN, String("Invalid order received for elevator ").concat(m_level));
    }
}

Elevators::Elevators(ros2::Node &node)
    : m_stepper_1(ELEVATOR_STEP_PER_REV, ELEVATOR_1_STEP_PIN, ELEVATOR_1_DIR_PIN, 1, ELEVATOR_SPEED, ELEVATOR_POS_OFFSET),
      m_stepper_2(ELEVATOR_STEP_PER_REV, ELEVATOR_2_STEP_PIN, ELEVATOR_2_DIR_PIN, 2, ELEVATOR_SPEED, ELEVATOR_POS_OFFSET),
      m_elevator_1(node, "/act/order/elevator_1", "/act/callback/elevator_1"),
      m_elevator_2(node, "/act/order/elevator_2", "/act/callback/elevator_2") {}

void Elevators::loop() {
    if (auto state = m_elevator_1.getRequestedState()) {
        if (state == UP && m_stepper_2.getState() == DOWN) {
            log(WARN, "Refusing to move elevator 1 UP while elevator 2 is DOWN");
        } else {
            m_stepper_1.setState(*state);
        }

        m_elevator_1.sendCallback(m_stepper_1.getState());
        m_elevator_1.clearRequestedState();
    }

    if (auto state = m_elevator_2.getRequestedState()) {
        if (state == DOWN && m_stepper_1.getState() == UP) {
            log(WARN, "Refusing to move elevator 2 DOWN while elevator 1 is UP");
        } else {
            m_stepper_2.setState(*state);
        }

        m_elevator_2.sendCallback(m_stepper_1.getState());
        m_elevator_2.clearRequestedState();
    }
}