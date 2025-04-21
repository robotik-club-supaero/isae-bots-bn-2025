#ifdef TEST_OSCILLATE

#include "Banner.hpp"
#include "Clamp.hpp"
#include "Elevator.hpp"
#include "Led.hpp"
#include "configuration.hpp"
#include "logging.hpp"

class TestServo {
   public:
    TestServo() : banner1(), banner2(), clamp11(), clamp12(), clamp21(), clamp22() {}

    void loop() {
        if (millis() - m_last_change > TEST_OSCILLATE_INTERVAL) {
            m_last_change = millis();

            banner1.toggle();
            banner2.toggle();

            clamp11.toggle();
            clamp12.toggle();
            clamp21.toggle();
            clamp22.toggle();
        }
    }

   private:
    unsigned long m_last_change;

    BannerServo1 banner1;
    BannerServo2 banner2;

    ClampServo1_1 clamp11;
    ClampServo1_2 clamp12;
    ClampServo2_1 clamp21;
    ClampServo2_2 clamp22;
};

class TestElevator {
   public:
    TestElevator(ElevatorStepper stepper) : m_stepper(stepper) {}

    void loop() {
        if (millis() - m_last_change > TEST_OSCILLATE_INTERVAL) {
            m_last_change = millis();

            m_stepper.setState((m_stepper.getState() + 1) % 2);
        }
        m_stepper.loop();
    }

   private:
    unsigned long m_last_change;

    ElevatorStepper m_stepper;
};

std::optional<BlinkLED> led;

std::optional<TestServo> test_servo;
std::optional<TestElevator> test_elevator;

void setup() {
    led.emplace();

#if TEST_OSCILLATE == TEST_SERVO
    test_servo.emplace();
#endif

#if TEST_OSCILLATE == TEST_STEPPER_1
    test_elevator.emplace(ElevatorStepper1());
#endif
#if TEST_OSCILLATE == TEST_STEPPER_2
    test_elevator.emplace(ElevatorStepper2());
#endif
}

void loop() {
    led->loop();

    if (test_servo) {
        test_servo->loop();
    }
    if (test_elevator) {
        test_elevator->loop();
    }
}

void log(LogSeverity _severity, String _message) {}

#endif