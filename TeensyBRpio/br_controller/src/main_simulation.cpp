#ifndef ARDUINO

#include "logging.hpp"
#include "specializations/manager.hpp"
#include "specializations/ros.hpp"

#include <atomic>
#include <memory>
#include <optional>
#include <signal.h>
#include <stdlib.h>

std::atomic<bool> quit(false);

std::optional<ros_t> ros;

void got_signal(int) {
    quit.store(true);
}

int main(int argc, char **argv) {
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = got_signal;
    sigfillset(&sa.sa_mask);
    sigaction(SIGINT, &sa, NULL);

    ros2::init(argc, argv);

    ros.emplace();
    ros->attachManager(createManager());

    while (!quit.load()) {
        ros->loop();
    }

    ros.reset();
    ros2::shutdown();
}

void log(LogSeverity severity, const char *message) {
    if (ros) {
        ros->sendLog(severity, message);
    }
}

#endif