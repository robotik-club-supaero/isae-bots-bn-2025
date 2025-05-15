#include "trajectories/LinearTrajectory.hpp"
#include <algorithm>

LinearTrajectory::LinearTrajectory(Point2D<Meter> origin, Point2D<Meter> destination)
    : m_origin(origin), m_destination(destination), m_direction((m_destination - m_origin).normalize()),
      m_totalLength(Point2D<Meter>::distance(m_origin, m_destination)), m_position(0) {}

bool LinearTrajectory::advance(number_t distance) {
    if (m_position == m_totalLength) {
        return false;
    }

    m_position += distance;
    if (m_position > m_totalLength) {
        m_position = m_totalLength;
    }
    return true;
}

Position2D<Meter> LinearTrajectory::getCurrentPosition() const {
    return Position2D<Meter>(m_origin + m_direction * m_position, m_direction.argument());
}

std::optional<number_t> LinearTrajectory::getRemainingDistance() const {
    return std::make_optional<number_t>(m_totalLength - m_position);
}

number_t LinearTrajectory::getMaxCurvature(number_t distance) {
    return 0;
}

bool LinearTrajectory::recompute(Position2D<Meter> newStartPosition) {
    *this = LinearTrajectory(newStartPosition, m_destination);
    return true;
}