#include "specializations/ros.hpp"
#include <Led.hpp>

std::optional<BlinkLED> blinkingLed = std::nullopt;
std::optional<ros_t> rosInstance = std::nullopt;

void setup() {
    Serial.begin(115200);
    ros2::init(Serial);

    blinkingLed.emplace();

    rosInstance.emplace();
    rosInstance->attachManager();
}

void loop() {
    rosInstance->loop();
    blinkingLed->loop();
}

void log(LogSeverity severity, const char *message) {
    if (rosInstance) {
        rosInstance->sendLog(severity, message);
    }
}