#include "Window.hpp"
#include "trajectories/PathTrajectory.hpp"

#include <iostream>

int main() {
    double_t step = 0.001;
    std::vector<Point2D<Meter>> points = {{0, 0}, {1.501, 0.796}, {1.766, 1.504}, {0.772, 2.790}};

    Window window("Bezier Curve");

    PathTrajectory trajectory(0, points);

    sf::VertexArray bezierCurve(sf::LinesStrip);
    while (trajectory.advance(step)) {
        bezierCurve.append(sf::Vertex(window.toScreenCoordinates(trajectory.getCurrentPosition()), sf::Color::Red));
    }

    // Intermediary control points for Bézier curves
    PathTrajectory trajectory2(0, points);
    std::vector<sf::CircleShape> sfCtrlPoints;
    do {
        const std::vector<Point2D<Meter>> &controlPoints = trajectory2.getCurrentBezierArc().points();
        for (int i = 1; i < controlPoints.size() - 1; i++) {
            sf::CircleShape sfPoint(4); // Radius of 5 pixels
            sfPoint.setFillColor(sf::Color::Green);
            sfPoint.setPosition(window.toScreenCoordinates(controlPoints[i])); 
            sfPoint.move(-4, -4); // Center the circle
            sfCtrlPoints.push_back(sfPoint);
        }
    } while (trajectory2.skipToNextArc());

    // Path cross points
    std::vector<sf::CircleShape> sfPathPoints;
    for (Point2D<Meter> point : points) {
        sf::CircleShape sfPoint(10); // Radius of 10 pixels
        sfPoint.setFillColor(sf::Color::Blue);
        sfPoint.setPosition(window.toScreenCoordinates(point)); 
        sfPoint.move(-10, -10); // Center the circle
        sfPathPoints.push_back(sfPoint);
    }

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
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
