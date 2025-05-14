#include "Window.hpp"
#include "drawables/BezierTrajectoryDrawable.hpp"
#include "drawables/PointDrawable.hpp"
#include "drawables/RobotDrawable.hpp"
#include "optim/CurveParameters.hpp"
#include "optim/optim.hpp"
#include "test_logging.hpp"

#include "specializations/manager.hpp"
#include "trajectories/PathTrajectory.hpp"

#include <iostream>
#include <limits>

constexpr controller::ControllerStatus Still = controller::ControllerStatus::Still;

template <Derived<Trajectory> T, typename... Args>
manager_t startTrajectorySimulation(Args &&...args) {
    T trajectory(std::forward<Args>(args)...);
    manager_t manager = createManager(trajectory.getCurrentPosition());
    manager.sendOrder(
        [&](controller_t &controller, Position2D<Meter> position) { controller.startTrajectory<T>(FORWARD, std::nullopt, std::move(trajectory)); });
    return manager;
}

template <Derived<Trajectory> T>
double measureTime(T &&trajectory, double limit) {
    DisableLoggingGuard _guard;

    duration_t limit_micros = limit > 0 ? (duration_t)(limit * 1e6) : std::numeric_limits<duration_t>::max();

    manager_t manager = startTrajectorySimulation<T>(std::forward<T>(trajectory));
    duration_t time = 0;
    while (manager.getController().getStatus() != Still || manager.getController().isMoving()) {
        manager.update(UPDATE_INTERVAL / 1e6);
        time += UPDATE_INTERVAL;
        if (time > limit_micros) {
            return std::numeric_limits<double>::max();
        }
    }
    return time / 1e6;
}

int main() {
    constexpr double_t STEP = 0.001;
    SmallDeque<Point2D<Meter>> points;

    Window window("Bezier Curve");

    std::vector<PointDrawable> pointDrawables;

    std::optional<BezierTrajectoryDrawable> trajectoryDrawable;
    std::optional<BezierTrajectoryDrawable> trajectoryDrawable2;

    RobotDrawable robotDrawable;
    RobotDrawable robotDrawable2(sf::Color(0, 150, 0));

    bool paused;
    std::optional<manager_t> manager;
    std::optional<manager_t> manager2;

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
                        PathTrajectory trajectory(std::nullopt, std::nullopt, points);
                        trajectoryDrawable.emplace(BezierTrajectoryDrawable(window, trajectory, STEP));
                    } else {
                        trajectoryDrawable.reset();
                    }
                    trajectoryDrawable2.reset();
                    break;

                case sf::Event::KeyPressed:
                    if (event.key.code == sf::Keyboard::Key::R && points.size() > 1) {
                        // -- SIMULATE TRAJECTORY --
                        paused = false;
                        manager.emplace(startTrajectorySimulation<PathTrajectory>(std::nullopt, std::nullopt, points));
                        manager2.reset();
                    } else if (event.key.code == sf::Keyboard::Key::O && points.size() > 1) {
                        // -- OPTIMIZE TRAJECTORY --
                        // This is still experimental

                        CurveParameters params(points);
                        std::cout << "Initial cost: " << params.cost() << std::endl;
                        params = optimize_local_search(params, 0.5, 100);
                        params = optimize_local_search(params, 0.1, 100);
                        params = optimize_local_search(params, 0.05, 100);
                        std::cout << "New cost: " << params.cost() << std::endl;

                        MultiCurveTrajectory<BezierCurve<4>> trajectory = params.generateTrajectory();
                        trajectoryDrawable2.emplace(
                            BezierTrajectoryDrawable(window, trajectory, STEP, sf::Color(150, 200, 50), sf::Color(50, 200, 0)));

                        paused = false;

                        manager.emplace(startTrajectorySimulation<PathTrajectory>(std::nullopt, std::nullopt, points));
                        manager2.emplace(startTrajectorySimulation<MultiCurveTrajectory<BezierCurve<4>>>(params.generateTrajectory()));
                    } else if (event.key.code == sf::Keyboard::Key::P) {
                        paused = !paused;
                        if (!paused) {
                            if (manager) {
                                manager->resyncClock();
                            }
                            if (manager2) {
                                manager2->resyncClock();
                            }
                        }
                    } else if (event.key.code == sf::Keyboard::Key::Space) {
                        auto brakeOrder = [&](controller_t &controller, Position2D<Meter> robotPosition) { controller.brakeToStop(); };
                        if (manager) {
                            manager->sendOrder(brakeOrder);
                        }
                        if (manager2) {
                            manager2->sendOrder(brakeOrder);
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
        if (trajectoryDrawable2) {
            window.draw(*trajectoryDrawable2);
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
        if (manager2) {
            robotDrawable2.setPosition(window, manager2->getPositionFeedback().getRobotPosition());
            window.draw(robotDrawable2);

            if (!paused) {
                manager2->update();
            }
            if (manager2->getController().getStatus() == Still && !manager2->getController().isMoving()) {
                manager2.reset();
            }
        }
        window.display();
    }
}
