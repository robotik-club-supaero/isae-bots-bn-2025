#include "Window.hpp"
#include "drawables/Joystick.hpp"
#include "drawables/PointDrawable.hpp"
#include "drawables/RobotDrawable.hpp"

#include "specializations/manager.hpp"

int main() {
    const Position2D<Meter> DEFAULT_POS = Position2D<Meter>(1, 1.5, Angle::Pi / 2);
    Window window("Speed control");
    manager_t manager = createManager(DEFAULT_POS);
    manager.setActive(true);

    RobotDrawable robotDrawable;
    Joystick joystick({0, 0});

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (joystick.processEvent(event)) {
                manager.sendOrder([&](controller_t &controller, Position2D<Meter> robotPosition) {
                    sf::Vector2f value = joystick.getValue();
                    if (abs(value.x) < 0.1) {
                        value.x = 0;
                    }
                    if (abs(value.y) < 0.1) {
                        value.y = 0;
                    }
                    Speeds speeds(-value.y * controller.getMaxSpeeds().linear, -value.x * controller.getMaxSpeeds().angular);
                    controller.setSetpointSpeed(speeds /*, enforceMaxSpeeds = true */);
                });
            } else {
                switch (event.type) {
                    case sf::Event::Closed:
                        window.close();
                        break;
                    case sf::Event::KeyPressed:
                        if (event.key.code == sf::Keyboard::Key::R) {
                            manager.resetPosition(DEFAULT_POS);
                        } else if (event.key.code == sf::Keyboard::Key::Space) {
                            manager.sendOrder([&](controller_t &controller, Position2D<Meter> robotPosition) { controller.brakeToStop(); });
                        }
                        break;
                }
            }
        }

        window.clear(sf::Color(220, 220, 220));
        window.draw(joystick);
        robotDrawable.setPosition(window, manager.getPositionFeedback().getRobotPosition());
        window.draw(robotDrawable);
        manager.update();
        window.display();
    }
}
