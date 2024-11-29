
#include "trajectories/BezierTrajectory.hpp"
#include <cmath>
#include <functional>
#include <iostream>

#define INTEGRAL_NUM_STEPS 1000
#define DIRECTION_EST_STEP 0.001

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

Point2D<Meter> bezier(const std::vector<Point2D<Meter>> &points, double_t t) {
    Point2D<Meter> position = points[0];
    for (int i = 0; i < points.size(); i++) {
        double b = bernstein(points.size() - 1, i, t);
        position += points[i] * b;
    }

    return position;
}

BezierTrajectory::BezierTrajectory(std::vector<Point2D<Meter>> points)
    : m_points(points), m_position(0), m_totalLength(0), m_direction(0), m_currentPosition(points[0]) {

    Point2D<Meter> lastPoint = points[0];
    for (int i = 0; i <= INTEGRAL_NUM_STEPS; i++) {
        Point2D<Meter> currentPoint = bezier(points, (double_t)i / INTEGRAL_NUM_STEPS);
        m_totalLength += Vector2D<Meter>::distance(lastPoint, currentPoint);
        lastPoint = currentPoint;
    }

    // Compute direction
    advance(0);
}

bool BezierTrajectory::advance(double_t distance) {
    if (m_position == m_totalLength) {
        return false;
    }

    m_position += distance;
    if (m_position > m_totalLength) {
        m_position = m_totalLength;
    }

    double_t t = m_position / m_totalLength;
    m_currentPosition = bezier(m_points, t);
    if (t + DIRECTION_EST_STEP <= 1) {
        m_direction = (bezier(m_points, t + DIRECTION_EST_STEP) - m_currentPosition).argument();
    }

    return true;
}

std::optional<double_t> BezierTrajectory::getRemainingDistance() const {
    return std::make_optional<double_t>(m_totalLength - m_position);
}

Position2D<Meter> BezierTrajectory::getCurrentPosition() const {
    return {m_currentPosition, m_direction};
}
