
#include "trajectories/BezierTrajectory.hpp"
#include <cmath>
#include <functional>
#include <iostream>

// Functions

// Factorial function
double factorial(int n) {
    if (n == 0) {
        return 1;
    }
    return n * factorial(n - 1);
}

// Bnomial coefficient function
double binomial(int n, int i) {
    return factorial(n) / (factorial(i) * factorial(n - i));
}

// Bernstein polynomial function
double_t bernstein(int n, int i, double_t t) {
    if (t < 0.0 || t > 1.0) {
        throw std::invalid_argument("t must be between 0 and 1");
    }
    return binomial(n, i) * std::pow(t, i) * std::pow(1 - t, n - i);
}

BezierTrajectory::BezierTrajectory(std::vector<Point2D<Meter>> points) : m_points(points), m_position(0), m_totalLength(0), m_direction(points[0]) {
    Point2D<Meter> lastPoint = m_points[0];
    for (int i = 1; i < 1000; i++) {
        m_position = (double_t)i / 1000.0;
        Point2D<Meter> currentPoint = getCurrentPosition();
        if (i == 1) {
            m_direction.update(currentPoint);
        }
        m_totalLength += Vector2D<Meter>::distance(lastPoint, currentPoint);
        lastPoint = currentPoint;
    }
    m_position = 0;
}

bool BezierTrajectory::advance(double_t distance) {
    if (m_position == m_totalLength) {
        return false;
    }

    m_position += distance;
    if (m_position > m_totalLength) {
        m_position = m_totalLength;
    }
    return true;
}

std::optional<double_t> BezierTrajectory::getRemainingDistance() const {
    return std::make_optional<double_t>(m_totalLength - m_position);
}

Position2D<Meter> BezierTrajectory::getCurrentPosition() const {

    Point2D p = m_points[0];
    for (int i = 0; i < m_points.size(); i++) {
        double b = bernstein(m_points.size() - 1, i, m_position / m_totalLength);
        p += points[i] * b;
    }
    m_direction.update(p);
    return {p, m_direction.value().argument()};
}
