#include "Window.hpp"
#include "drawables/BezierTrajectoryDrawable.hpp"
#include "drawables/PointDrawable.hpp"
#include "drawables/RobotDrawable.hpp"
#include "test_logging.hpp"
#include "specializations/manager.hpp"
#include "trajectories/PathTrajectory.hpp"

#include <iostream>

constexpr controller::ControllerStatus Still = controller::ControllerStatus::Still;

manager_t startTrajectorySimulation(std::unique_ptr<Trajectory> trajectory) {
    manager_t manager = createManager(trajectory->getCurrentPosition());
    manager.setActive(true);
    while (!manager.isActive()) {
        manager.update(UPDATE_INTERVAL / 1e6);
    }

    manager.sendOrder(
        [&](controller_t &controller, Position2D<Meter> position) { controller.startTrajectory(FORWARD, std::move(trajectory), std::nullopt); });
    return manager;
}

int main() {
    constexpr double_t STEP = 0.001;
    std::vector<Point2D<Meter>> points;

    Window window("Bezier Curve");

    std::vector<PointDrawable> pointDrawables;

    std::optional<BezierTrajectoryDrawable> trajectoryDrawable;
    RobotDrawable robotDrawable;

    bool paused;
    std::optional<manager_t> manager;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            switch (event.type) {
                case sf::Event::Closed:
                    window.close();
                    break;

                case sf::Event::MouseButtonPressed:
                    if (event.mouseButton.button == sf::Mouse::Left) {
                        // -- ADD A POINT --
                        Point2D<Meter> coords = window.toPhysicalCoordinates(event.mouseButton.x, event.mouseButton.y);
                        points.push_back(coords);
                        pointDrawables.push_back(PointDrawable(sf::Vector2f(event.mouseButton.x, event.mouseButton.y)));
                        std::cout << "Added point at " << std::string(coords) << std::endl;
                    } else if (event.mouseButton.button == sf::Mouse::Right) {
                        // -- REMOVE LAST POINT --
                        if (!points.empty()) {
                            points.pop_back();
                            pointDrawables.pop_back();
                        }
                    }
                    if (points.size() > 1) {
                        PathTrajectory trajectory(std::nullopt, points);
                        trajectoryDrawable.emplace(BezierTrajectoryDrawable(window, trajectory, STEP));
                    } else {
                        trajectoryDrawable.reset();
                    }
                    break;

                case sf::Event::KeyPressed:
                    if (event.key.code == sf::Keyboard::Key::R && points.size() > 1) {
                        // -- SIMULATE TRAJECTORY --
                        paused = false;
                        manager.emplace(startTrajectorySimulation(std::make_unique<PathTrajectory>(std::nullopt, points)));

                    } else if (event.key.code == sf::Keyboard::Key::P) {
                        paused = !paused;
                        if (!paused) {
                            if (manager) {
                            manager->resyncClock();
                            }
                        }
                    } else if (event.key.code == sf::Keyboard::Key::Space) {
                        auto brakeOrder = [&](controller_t &controller, Position2D<Meter> robotPosition) { controller.brakeToStop(); };
                        if (manager) {
                            manager->sendOrder(brakeOrder);
                        }
                    }
                    break;
                default:
                    break;
            }
        }

        window.clear(sf::Color(220, 220, 220));
        for (const auto &point : pointDrawables) {
            window.draw(point);
        }
        if (trajectoryDrawable) {
            window.draw(*trajectoryDrawable);
        }

        if (manager) {
            robotDrawable.setPosition(window, manager->getPositionFeedback().getRobotPosition());
            window.draw(robotDrawable);           

            if (!paused) {
                manager->update();
            }
            if (manager->getController().getStatus() == Still && !manager->getController().isMoving()) {
                manager.reset();
            }
            }
        window.display();
    }
}
