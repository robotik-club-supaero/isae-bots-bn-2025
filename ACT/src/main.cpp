#include <ros2arduino.h>

#include <optional>

#include "Banner.hpp"
#include "Clamp.hpp"
#include "Elevator.hpp"
#include "logging.hpp"

ros2::Node node("ACT");
ros2::Publisher<std_msgs::String> *logger(node.createPublisher<std_msgs::String>("/act/logging"));

std::optional<Elevators> elevators;
std::optional<Clamp1> clamp1;
std::optional<Clamp2> clamp2;
std::optional<Banner> banner;

void setup() {
    Serial.begin(115200);
    while (!Serial);

    ros2::init(&Serial);

    elevators.emplace(node);
    clamp1.emplace(node);
    clamp2.emplace(node);
    banner.emplace(node);
}

void loop() {
    elevators->loop();
    clamp1->loop();
    clamp2->loop();
    banner->loop();

    ros2::spin(&node);
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
        std_msgs::String msg;
        constexpr size_t max_len = sizeof(msg.data);

        String severityPrefix = String("[").concat(severityToString(severity)).concat("] ");
        size_t max_msg_len = max_len - 1 - severityPrefix.length();

        while (message.length() > max_msg_len) {
            String partialMessage = severityPrefix.concat(message.substring(0, max_msg_len));
            message = message.substring(max_msg_len);

            memcpy(msg.data, partialMessage.c_str(), partialMessage.length() + 1);
            logger->publish(&msg);
        }

        String _message = severityPrefix.concat(message);
        memcpy(msg.data, _message.c_str(), _message.length() + 1);
        logger->publish(&msg);
    }
}

extern "C" {
__attribute__((weak)) int _write(int file, char *ptr, int len) {
    ((class Print *)file)->write((uint8_t *)ptr, len);
    return len;
}
}