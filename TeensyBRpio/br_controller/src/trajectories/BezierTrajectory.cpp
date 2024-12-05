#include "trajectories/BezierTrajectory.hpp"

BezierTrajectory::BezierTrajectory(std::vector<Point2D<Meter>> points)
    : m_curve(std::move(points)), m_position(0), m_totalLength(m_curve.computeLength()) {}

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
    return m_curve.position(m_position / m_totalLength);
}
