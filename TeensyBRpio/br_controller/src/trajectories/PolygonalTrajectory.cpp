#include "trajectories/PolygonalTrajectory.hpp"

PolygonalTrajectory::PolygonalTrajectory(SmallDeque<Point2D<Meter>> path) : m_path(std::move(path)), m_currentLine(m_path[0], m_path[1]) {}

bool PolygonalTrajectory::advance(double_t distance) {
    double_t remainingOnCurrentLine = *m_currentLine.getRemainingDistance();
    while (distance >= remainingOnCurrentLine && setupNextLine()) {
        distance -= remainingOnCurrentLine;
        remainingOnCurrentLine = *m_currentLine.getRemainingDistance();
    }

    return advanceSmooth(distance);
}

bool PolygonalTrajectory::advanceSmooth(double_t distance) {
    return m_currentLine.advance(distance);
}

Position2D<Meter> PolygonalTrajectory::getCurrentPosition() const {
    return m_currentLine.getCurrentPosition();
}

std::optional<double_t> PolygonalTrajectory::getRemainingDistance() const {
    if (m_path.size() == 2) {
        return *m_currentLine.getRemainingDistance();
    } else {
        return std::nullopt;
    }
}

double_t PolygonalTrajectory::getMaxCurvature(double_t distance) {
    if (distance < *m_currentLine.getRemainingDistance()) {
        return 0;
    } else {
        return std::numeric_limits<double_t>::infinity();
    }
}

bool PolygonalTrajectory::recompute(Position2D<Meter> newStartPosition) {
    if (m_currentLine.recompute(newStartPosition)) {
        m_path[0] = newStartPosition;
        return true;
    } else {
        return false;
    }
}

bool PolygonalTrajectory::setupNextLine() {
    if (m_path.size() > 2) {
        m_path.pop_front();
        m_currentLine = LinearTrajectory(m_path[0], m_path[1]);
        return true;
    } else {
        return false;
    }
}
