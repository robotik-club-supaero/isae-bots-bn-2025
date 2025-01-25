#include "TestClock.hpp"
#include "Window.hpp"
#include "specializations/manager.hpp"
#include "trajectories/PathTrajectory.hpp"

#include <iostream>

int main() {
    double_t step = 0.001;
    std::vector<Point2D<Meter>> points;

    Window window("Bezier Curve");

    sf::VertexArray bezierCurve(sf::LinesStrip);
    std::vector<sf::CircleShape> sfCtrlPoints;
    std::vector<sf::CircleShape> sfPathPoints;

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
                        std::cout << "Added point at " << std::string(coords) << std::endl;

                        sf::CircleShape sfPoint(10); // Radius of 10 pixels
                        sfPoint.setFillColor(sf::Color::Blue);
                        sfPoint.setPosition(event.mouseButton.x, event.mouseButton.y);
                        sfPoint.move(-10, -10); // Center the circle
                        sfPathPoints.push_back(sfPoint);

                    } else if (event.mouseButton.button == sf::Mouse::Right && points.size() > 1) {
                        // -- CREATE TRAJECTORY --
                        bezierCurve.clear();
                        sfCtrlPoints.clear();

                        PathTrajectory trajectory(std::nullopt, points);
                        while (trajectory.advance(step)) {
                            bezierCurve.append(sf::Vertex(window.toScreenCoordinates(trajectory.getCurrentPosition()), sf::Color::Red));
                        }

                        // Intermediary control points for Bézier curves
                        trajectory = PathTrajectory(std::nullopt, points);
                        do {
                            const std::vector<Point2D<Meter>> &controlPoints = trajectory.getCurrentCurve().points();
                            for (unsigned int i = 1; i < controlPoints.size() - 1; i++) {
                                sf::CircleShape sfPoint(4); // Radius of 5 pixels
                                sfPoint.setFillColor(sf::Color(0, 100, 0));
                                sfPoint.setPosition(window.toScreenCoordinates(controlPoints[i]));
                                sfPoint.move(-4, -4); // Center the circle
                                sfCtrlPoints.push_back(sfPoint);
                            }
                        } while (trajectory.skipToNextCurve());

                    } else if (event.mouseButton.button == sf::Mouse::Middle) {
                        // -- REMOVE LAST POINT --
                        bezierCurve.clear();
                        sfCtrlPoints.clear();
                        if (!points.empty()) {
                            points.pop_back();
                            sfPathPoints.pop_back();
                        }
                    }
                    break;

                case sf::Event::KeyPressed:
                    if (event.key.code == sf::Keyboard::Key::R && points.size() > 1) {
                        // -- SIMULATE TRAJECTORY --
                        feedback_t feedback;
                        feedback.resetPosition(Position2D<Meter>(points[0], 0));
                        actuators_t motors = feedback.createMotorStub();
                        manager.emplace(std::move(motors), std::move(feedback));
                        manager->setActive(true);
                        while (!manager->isActive()) {
                            manager->loop();
                        }

                        manager->sendOrder([&](controller_t &controller, Position2D<Meter> position) {
                            std::unique_ptr<Trajectory> trajectory = std::make_unique<PathTrajectory>(std::nullopt, points);
                            controller.startTrajectory(FORWARD, std::move(trajectory), std::nullopt);
                        });
                    } else if (event.key.code == sf::Keyboard::Key::Space && manager) {
                        TestClock &clock = manager->getClock();
                        if (clock.paused()) {
                            clock.resume();
                        } else {
                            clock.pause();
                        }
                    }
                    break;
                default:
                    break;
            }
        }

        window.clear(sf::Color(220, 220, 220));
        for (const auto &point : sfPathPoints) {
            window.draw(point);
        }
        for (const auto &point : sfCtrlPoints) {
            window.draw(point);
        }
        window.draw(bezierCurve);

        if (manager) {
            Position2D<Meter> robotPosition = manager->getPositionFeedback().getRobotPosition();
            sf::Vector2f screenPosition = window.toScreenCoordinates(robotPosition);
            double_t rotation = 180. / Angle::Pi * (Angle::Pi / 2 - robotPosition.theta.value());

            const unsigned int DRAW_SIZE = 20;
            sf::RectangleShape robotMarker(sf::Vector2f(2 * DRAW_SIZE, 2 * DRAW_SIZE));
            robotMarker.setFillColor(sf::Color(128, 0, 128));
            robotMarker.setOrigin(DRAW_SIZE, DRAW_SIZE);
            robotMarker.setPosition(screenPosition);
            robotMarker.setRotation(rotation);

            const unsigned int HEAD_SIZE = 3;
            sf::RectangleShape headMarker(sf::Vector2f(2 * HEAD_SIZE, DRAW_SIZE));
            headMarker.setFillColor(sf::Color::White);
            headMarker.setPosition(screenPosition.x, screenPosition.y);
            headMarker.setRotation(rotation + 180);

            window.draw(robotMarker);
            window.draw(headMarker);

            if (!manager->getClock().paused()) {
                manager->loop();
            }
        }

        window.display();
    }
}
