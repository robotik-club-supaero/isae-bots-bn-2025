#include "Window.hpp"
#include "trajectories/PathTrajectory.hpp"

#include <iostream>

int main() {
    double_t step = 0.001;
    std::vector<Point2D<Meter>> points;

    Window window("Bezier Curve");

    sf::VertexArray bezierCurve(sf::LinesStrip);
    std::vector<sf::CircleShape> sfCtrlPoints;
    std::vector<sf::CircleShape> sfPathPoints;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            switch (event.type) {
                case sf::Event::Closed:
                    window.close();
                    break;
                case sf::Event::MouseButtonPressed:
                    if (event.mouseButton.button == sf::Mouse::Left) {
                        points.push_back(window.toPhysicalCoordinates(event.mouseButton.x, event.mouseButton.y));

                        sf::CircleShape sfPoint(10); // Radius of 10 pixels
                        sfPoint.setFillColor(sf::Color::Blue);
                        sfPoint.setPosition(event.mouseButton.x, event.mouseButton.y);
                        sfPoint.move(-10, -10); // Center the circle
                        sfPathPoints.push_back(sfPoint);
                    } else if (event.mouseButton.button == sf::Mouse::Right && points.size() > 1) {
                        bezierCurve.clear();
                        sfCtrlPoints.clear();

                        PathTrajectory trajectory(0, points);
                        while (trajectory.advance(step)) {
                            bezierCurve.append(sf::Vertex(window.toScreenCoordinates(trajectory.getCurrentPosition()), sf::Color::Red));
                        }

                        // Intermediary control points for Bézier curves
                        trajectory = PathTrajectory(0, points);
                        do {
                            const std::vector<Point2D<Meter>> &controlPoints = trajectory.getCurrentBezierArc().points();
                            for (int i = 1; i < controlPoints.size() - 1; i++) {
                                sf::CircleShape sfPoint(4); // Radius of 5 pixels
                                sfPoint.setFillColor(sf::Color::Green);
                                sfPoint.setPosition(window.toScreenCoordinates(controlPoints[i]));
                                sfPoint.move(-4, -4); // Center the circle
                                sfCtrlPoints.push_back(sfPoint);
                            }
                        } while (trajectory.skipToNextArc());
                    } else if (event.mouseButton.button == sf::Mouse::Middle) {
                        bezierCurve.clear();
                        sfCtrlPoints.clear();
                        if (!points.empty()) {
                            points.pop_back();
                            sfPathPoints.pop_back();
                        }
                    }
                    break;
                default:
                    break;
            }
        }

        window.clear();
        for (const auto &point : sfPathPoints) {
            window.draw(point);
        }
        for (const auto &point : sfCtrlPoints) {
            window.draw(point);
        }
        window.draw(bezierCurve);
        window.display();
    }
}
