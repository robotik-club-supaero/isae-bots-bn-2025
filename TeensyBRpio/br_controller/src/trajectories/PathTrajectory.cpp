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
    : m_points(points), m_currentArcIndex(0), m_currentArc(), m_currentArcLength(0), m_currentArcPosition(0), m_currentDerivative(),
      m_remainingLength(0) {
    if (points.size() < 2) {
        abort("Trajectory must have at least 2 points");
    }

    Angle preferredDirection = (points[1] - points[0]).argument();
    if (points.size() > 2) {
        preferredDirection += Vector2D<Meter>::angle(points[2] - points[1], points[1] - points[0]).value() / 2;
    }
    if (abs(initialDirection - preferredDirection) > Angle::PI / 3) {
        // Force the controller to enter state "initial rotation" before starting the trajectory
        // As we are not moving yet, it is more efficient (and safer) to turn now than starting to move in the wrong direction.
        initialDirection = preferredDirection;
    }
    setupArc(initialDirection);
    updateRemainingLength();

    m_currentDerivative = m_currentArc->derivative(0);
}

void PathTrajectory::setupArc(Angle initialDirection) {
    std::optional<Point2D<Meter>> next = std::nullopt;
    if (m_currentArcIndex + 2 < m_points.size()) {
        next = std::make_optional(m_points[m_currentArcIndex + 2]);
    }
    m_currentArc.emplace(generateBezier(m_points[m_currentArcIndex + 0], m_points[m_currentArcIndex + 1], initialDirection, next));
    m_currentArcLength = m_currentArc->computeLength();
}

void PathTrajectory::updateRemainingLength() {
    m_remainingLength = 0;
    for (size_type i = m_currentArcIndex + 1; i < numberOfArcs(); i++) {
        m_remainingLength += Point2D<Meter>::distance(m_points[i], m_points[i + 1]);
    }
}

bool PathTrajectory::advance(double_t distance) {
    if (m_currentArcPosition >= 1 && m_currentArcIndex >= numberOfArcs() - 1) {
        return false;
    }

    do {
        double_t remainingOnArc = m_currentArcLength * (1 - m_currentArcPosition);
        if (distance < remainingOnArc) {
            double_t increment = distance / m_currentDerivative.norm();
            m_currentArcPosition += increment;
            if (m_currentArcPosition > 1) {
                m_currentArcPosition = 1;
            }
            distance = 0;
        } else {
            m_currentArcPosition = 1;
            distance -= remainingOnArc;
        }
        if (m_currentArcPosition >= 1 && m_currentArcIndex < numberOfArcs() - 1) {
            m_currentArcIndex++;
            m_currentArcPosition = 0;

            setupArc(m_currentArc->derivative(1).argument());
            updateRemainingLength();
        }
    } while (distance > 0 && m_currentArcIndex < numberOfArcs() - 1);

    m_currentDerivative = m_currentArc->derivative(m_currentArcPosition);
    return true;
}

Position2D<Meter> PathTrajectory::getCurrentPosition() const {
    return {m_currentArc->operator()(m_currentArcPosition), m_currentDerivative.argument()};
}

std::optional<double_t> PathTrajectory::getRemainingDistance() const {
    return m_remainingLength + m_currentArcLength * (1 - m_currentArcPosition);
}

const std::vector<Point2D<Meter>> &PathTrajectory::getPathPoints() const {
    return m_points;
}
const BezierCurve &PathTrajectory::getCurrentBezierArc() const {
    return *m_currentArc;
}
PathTrajectory::size_type PathTrajectory::getCurrentArcIndex() const {
    return m_currentArcIndex;
}

bool PathTrajectory::skipToNextArc() {
    m_currentArcPosition = 1;
    return advance(0);
}