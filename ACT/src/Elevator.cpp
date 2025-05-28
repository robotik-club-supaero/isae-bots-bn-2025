#include "Elevator.hpp"

#include "configuration.hpp"
#include "logging.hpp"

// TODO what should be the initial state?
ElevatorStepper::ElevatorStepper(int number_of_steps, int pin1, int pin2, int level, long speed, int move_steps_up, int move_steps_down)
    : m_stepper(number_of_steps, pin1, pin2),
      m_level(level),
      m_steps_up(move_steps_up),
      m_steps_down(move_steps_down),
      m_remaining_steps(),
      m_max_steps(max(1, STEPPER_YIELD_TIMEOUT * speed * number_of_steps / 60 / 1000)),
      m_state(DOWN) {
    m_stepper.setSpeed(speed);
}

ElevatorStepper1::ElevatorStepper1()
    : ElevatorStepper(ELEVATOR_1_STEP_PER_REV, ELEVATOR_1_STEP_PIN, ELEVATOR_1_DIR_PIN, 1, ELEVATOR_1_SPEED, ELEVATOR_1_POS_OFFSET_UP, ELEVATOR_1_POS_OFFSET_DOWN) {}

ElevatorStepper2::ElevatorStepper2()
    : ElevatorStepper(ELEVATOR_2_STEP_PER_REV, ELEVATOR_2_STEP_PIN, ELEVATOR_2_DIR_PIN, 2, ELEVATOR_2_SPEED, ELEVATOR_2_POS_OFFSET_UP, ELEVATOR_2_POS_OFFSET_DOWN) {}

ElevatorCallback ElevatorStepper::getState() const { return m_state; }
bool ElevatorStepper::setState(uint16_t state) {
    if (state == m_state && m_remaining_steps == 0) {
        return false;
    }
    if (state != m_state) {
        switch (state) {
            case UP:
                log(INFO, String("Moving elevator ").concat(m_level).concat(" UP"));
                m_remaining_steps += m_steps_up;
                m_state = UP;
                break;
            case DOWN:

                log(INFO, String("Moving elevator ").concat(m_level).concat(" DOWN"));
                m_remaining_steps -= m_steps_down;
                m_state = DOWN;
                break;
            default:
                log(WARN, String("Invalid order received for elevator ").concat(m_level));
        }
    }
    return true;
}

bool ElevatorStepper::loop() {
    if (m_remaining_steps < -m_max_steps) {
        m_stepper.step(-m_max_steps);
        m_remaining_steps += m_max_steps;
    } else if (m_remaining_steps > m_max_steps) {
        m_stepper.step(m_max_steps);
        m_remaining_steps -= m_max_steps;
    } else if (m_remaining_steps != 0) {
        m_stepper.step(m_remaining_steps);
        m_remaining_steps = 0;
        return true;
    }
    return false;
}

Elevators::Elevators(ros2::Node &node)
    : m_stepper_1(),
      m_stepper_2(),
      m_elevator_1(node, "/act/order/elevator_1", "/act/callback/elevator_1"),
      m_elevator_2(node, "/act/order/elevator_2", "/act/callback/elevator_2") {}

void Elevators::loop() {
    if (auto state = m_elevator_1.getRequestedState()) {
        if (state == UP && m_stepper_2.getState() == DOWN) {
            log(WARN, "Refusing to move elevator 1 UP while elevator 2 is DOWN");
            m_elevator_1.sendCallback(m_stepper_1.getState());
        } else {
            if (!m_stepper_1.setState(*state)) {
                m_elevator_1.sendCallback(m_stepper_1.getState());
            }
        }
        m_elevator_1.clearRequestedState();
    }

    if (auto state = m_elevator_2.getRequestedState()) {
        if (state == DOWN && m_stepper_1.getState() == UP) {
            log(WARN, "Refusing to move elevator 2 DOWN while elevator 1 is UP");
            m_elevator_2.sendCallback(m_stepper_2.getState());
        } else {
            if (!m_stepper_2.setState(*state)) {
                m_elevator_2.sendCallback(m_stepper_2.getState());
            } 
        }
        m_elevator_2.clearRequestedState();
    }

    if (m_stepper_1.loop()) {
        m_elevator_1.sendCallback(m_stepper_1.getState());
    }
    if (m_stepper_2.loop()) {
        m_elevator_2.sendCallback(m_stepper_2.getState());
    }
}