
#include <optional>

#include "Banner.hpp"
#include "Clamp.hpp"
#include "Led.hpp"
#include "ros2/ros2.hpp"
// #include "Elevator.hpp"
#include "logging.hpp"

std::optional<BlinkLED> led;

std::optional<ros2::Node> node("ACT");
std::optional<ros2::Publisher<std_msgs::String>> logger;

// std::optional<Elevators> elevators;
std::optional<Clamp1> clamp1;
std::optional<Clamp2> clamp2;
std::optional<Banner> banner;

void setup() {
    Serial.begin(115200);
    while (!Serial);

    led.emplace();

    ros2::init(Serial);

    node.emplace("ACT");
    logger.emplace(node->createPublisher<std_msgs::String>("/act/logging"));

    //   elevators.emplace(node);
    clamp1.emplace(*node);
    clamp2.emplace(*node);
    banner.emplace(*node);
}

void loop() {
    led->loop();

    // elevators->loop();
    clamp1->loop();
    clamp2->loop();
    banner->loop();

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
    std_msgs::String msg(String("[").concat(severityToString(severity)).concat("] ").concat(message));
    logger->publish(msg);
}