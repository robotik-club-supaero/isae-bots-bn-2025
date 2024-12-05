#include "trajectories/PathTrajectory.hpp"
#include "logging.hpp"

/**
 * Generates a Bézier curve such that: 
 *  - The curve goes from `start` to `end`
 *  - The direction at the start point is `initialDirection`
 *  - If `next` is non-empty, the direction at the end point is the mean between [start, end] and [end, next].
 * 
 * @param next The next destination of the robot, *after* the generated Bézier curve, or `std::nullopt` if `end` is the final destination.
 */
BezierCurve generateBezier(Point2D<Meter> start, Point2D<Meter> end, Angle initialDirection, std::optional<Point2D<Meter>> next) {
    double_t distance = Point2D<Meter>::distance(start, end);
    double_t ctrlPtDistance = distance / 3;

    Point2D<Meter> controlPt1 = start + Point2D<Meter>(std::cos(initialDirection), std::sin(initialDirection)) * ctrlPtDistance;

    Vector2D<Meter> controlLine1 = start - end;
    Vector2D<Meter> controlLine2;
    if (next) {
        controlLine2 = end - *next;
    } else {
        controlLine2 = controlPt1 - end;
    }
    Vector2D<Meter> finalDirection = (controlLine1.normalize() + controlLine2.normalize()) / 2;
    Point2D<Meter> controlPt2 = end + finalDirection.normalize() * ctrlPtDistance;

    return {start, controlPt1, controlPt2, end};
}

PathTrajectory::PathTrajectory(Angle initialDirection, std::vector<Point2D<Meter>> points)
    : m_points(points), m_currentArcIndex(0), m_currentArc({}), m_currentArcLength(0), m_currentArcPosition(0), m_remainingLength(0) {
    if (points.size() < 2) {
        abort("Trajectory must have at least 2 points");
    }
    setupArc(initialDirection);
    updateRemainingLength();
}

void PathTrajectory::setupArc(Angle initialDirection) {
    std::optional<Point2D<Meter>> next = std::nullopt;
    if (m_currentArcIndex + 2 < m_points.size()) {
        next = std::make_optional(m_points[m_currentArcIndex + 2]);
    }
    m_currentArc = generateBezier(m_points[m_currentArcIndex + 0], m_points[m_currentArcIndex + 1], initialDirection, next);
    m_currentArcLength = m_currentArc.computeLength();
}

void PathTrajectory::updateRemainingLength() {
    m_remainingLength = 0;
    for (size_type i = m_currentArcIndex + 1; i < numberOfArcs(); i++) {
        m_remainingLength += Point2D<Meter>::distance(m_points[i], m_points[i + 1]);
    }
}

bool PathTrajectory::advance(double_t distance) {
    if (m_currentArcPosition >= m_currentArcLength && m_currentArcIndex >= numberOfArcs() - 1) {
        return false;
    }
    m_currentArcPosition += distance;

    while (m_currentArcPosition >= m_currentArcLength && m_currentArcIndex < numberOfArcs() - 1) {
        m_currentArcIndex++;
        m_currentArcPosition -= m_currentArcLength;

        const std::vector<Point2D<Meter>> &points = m_currentArc.points();
        Angle direction = (points[points.size() - 1] - points[points.size() - 2]).argument();
        setupArc(direction);
        updateRemainingLength();
    }

    if (m_currentArcPosition >= m_currentArcLength) {
        m_currentArcPosition = m_currentArcLength;
    }

    return true;
}

Position2D<Meter> PathTrajectory::getCurrentPosition() const {
    return m_currentArc.position(m_currentArcPosition / m_currentArcLength);
}

std::optional<double_t> PathTrajectory::getRemainingDistance() const {
    return m_remainingLength + m_currentArcLength - m_currentArcPosition;
}

const std::vector<Point2D<Meter>> &PathTrajectory::getPathPoints() const {
    return m_points;
}
const BezierCurve &PathTrajectory::getCurrentBezierArc() const {
    return m_currentArc;
}
PathTrajectory::size_type PathTrajectory::getCurrentArcIndex() const {
    return m_currentArcIndex;
}

bool PathTrajectory::skipToNextArc() {
    m_currentArcPosition = m_currentArcLength;
    return advance(0);
}