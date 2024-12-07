#include "trajectories/BezierTrajectory.hpp"

BezierTrajectory::BezierTrajectory(std::vector<Point2D<Meter>> points)
    : m_curve(std::move(points)), m_position(0), m_totalLength(m_curve.computeLength()) {}

bool BezierTrajectory::advance(double_t distance) {
    if (m_position == 1) {
        return false;
    }

    m_position += distance / m_curve.derivative(m_position).norm();
    if (m_position > 1) {
        m_position = 1;
    }
    return true;
}

std::optional<double_t> BezierTrajectory::getRemainingDistance() const {
    return m_totalLength * (1 - m_position);
}

Position2D<Meter> BezierTrajectory::getCurrentPosition() const {
    return {m_curve(m_position), m_curve.derivative(m_position).argument()};
}
