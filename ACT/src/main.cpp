#ifndef TEST_OSCILLATE

#include <optional>

#include "Banner.hpp"
#include "Bumper.hpp"
#include "Clamp.hpp"
#include "Elevator.hpp"
#include "Led.hpp"
#include "logging.hpp"
#include "ros2/ros2.hpp"

std::optional<BlinkLED> led;

std::optional<ros2::Node> node;
std::optional<ros2::Publisher<std_msgs::String>> logger;

std::optional<Elevators> elevators;
std::optional<Clamps1> clamp1;
std::optional<Clamps2> clamp2;
std::optional<Banner> banner;
std::optional<Bumpers> bumpers;
std::optional<ros2::Subscriber<std_msgs::Empty>> resetSub;

void setup() {
    Serial.begin(115200);
    while (!Serial);

    led.emplace();

    ros2::init(Serial);

    node.emplace("ACT");
    logger.emplace(node->createPublisher<std_msgs::String>("/act/logging"));

    elevators.emplace(*node);
    clamp1.emplace(*node);
    clamp2.emplace(*node);
    banner.emplace(*node);
    bumpers.emplace(*node);

    resetSub.emplace(node->createSubscriber<std_msgs::Empty>("/act/reset", [](const std_msgs::Empty &) {
        // WARNING: use ->reset instead of .reset, otherwise you will reset the "optional" and not the actuators' state!
        elevators->reset();
        clamp1->reset();
        clamp2->reset();
        banner->reset();
        bumpers->reset();
    }));
}

void loop() {
    led->loop();

    elevators->loop();
    clamp1->loop();
    clamp2->loop();
    banner->loop();
    bumpers->loop();

    ros2::spin(*node);
}

inline const char *severityToString(LogSeverity severity) {
    switch (severity) {
        case DEBUG:
            return "DEBUG";
        case INFO:
            return "INFO";
        case WARN:
            return "WARN";
        case ERROR:
            return "ERROR";
        case FATAL:
            return "FATAL";
        default:
            return "INVALID";
    }
}

void log(LogSeverity severity, String message) {
    if (logger) {
        std_msgs::String msg(String("[").concat(severityToString(severity)).concat("] ").concat(message));
        logger->publish(msg);
    }
}

#endif
