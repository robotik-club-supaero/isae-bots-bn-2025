#include "trajectories/PathTrajectory.hpp"
#include "logging.hpp"

/**
 * Returns the unnormalized direction of the angle bisector between lineA and lineB.
 */
inline Vector2D<Meter> angleBisector(Vector2D<Meter> lineA, Vector2D<Meter> lineB) {
    return lineA.normalize() + lineB.normalize();
}

/**
 * Generates a Bézier curve such that:
 *  - The curve goes from `start` to `end`
 *  -  The direction at the start point is that of `initialDirection`
 *  - If `next` is non-empty, the direction at the end point is the angle bisector between [start, end] and [end, next].
 *
 * @param next The next destination of the robot, *after* the generated Bézier curve, or `std::nullopt` if `end` is the final destination.
 */
BezierCurve generateBezier(Point2D<Meter> start, Point2D<Meter> end, Vector2D<Meter> initialDirection, std::optional<Point2D<Meter>> next) {
    double_t distance = Point2D<Meter>::distance(start, end);
    double_t ctrlPtDistance = distance / 3;

    Point2D<Meter> controlPt1 = start + initialDirection.normalize() * ctrlPtDistance;

    Vector2D<Meter> finalDirection;
    if (next) {
        finalDirection = angleBisector(start - end, end - *next);
    } else {
        finalDirection = controlPt1 - end;
    }
    Point2D<Meter> controlPt2 = end + finalDirection.normalize() * ctrlPtDistance;

    return {start, controlPt1, controlPt2, end};
}

PathTrajectory::PathTrajectory(std::optional<Angle> initialDirection, std::vector<Point2D<Meter>> points)
    : m_points(points), m_currentArcIndex(0), m_currentArc(), m_currentArcPosition(0), m_currentArcDistance(0), m_currentDerivative(),
      m_remainingLength(0) {
    if (points.size() < 2) {
        abort("Trajectory must have at least 2 points");
    }

    Vector2D<Meter> preferredDirection;
    if (points.size() > 2) {
        preferredDirection = angleBisector(points[1] - points[0], 1.5 * points[1] - (points[0] + 0.5 * points[2]));
    } else {
        preferredDirection = points[1] - points[0];
    }
    // If the robot is already heading in the right direction, we bypass state "initial rotation" and immediately start the trajectory
    // Otherwise, as we are not moving yet, it is more efficient (and safer) to turn now than starting to move in the wrong direction.
    if (initialDirection) {
        if (abs(*initialDirection - preferredDirection.argument()) <= Angle::Pi / 3) {
            preferredDirection = {std::cos(*initialDirection), std::sin(*initialDirection)};
        }
    }

    m_currentArc.emplace(generateArc(0, preferredDirection));
    updateRemainingLength();

    m_currentDerivative = m_currentArc->curve.derivative(0);
}

std::optional<PathTrajectory::BezierArc> PathTrajectory::generateNextArc() const {
    size_type nextIndex = m_currentArcIndex + 1 + m_generatedArcs.size();
    if (nextIndex < numberOfArcs()) {
        const BezierArc &lastGeneratedArc = (m_generatedArcs.empty() ? *m_currentArc : m_generatedArcs.back());
        return generateArc(nextIndex, lastGeneratedArc.curve.derivative(1));
    } else {
        return std::nullopt;
    }
}

PathTrajectory::BezierArc PathTrajectory::generateArc(size_type index, Vector2D<Meter> initialDirection) const {
    std::optional<Point2D<Meter>> next = std::nullopt;
    if (index + 1 < numberOfArcs()) {
        next = std::make_optional(m_points[index + 2]);
    }
    BezierCurve curve = generateBezier(m_points[index], m_points[index + 1], initialDirection, next);

    return curve;
}

void PathTrajectory::setupNextArc() {
    if (m_generatedArcs.empty()) {
        m_currentArc.emplace(*generateNextArc());
    } else {
        m_currentArc.emplace(std::move(m_generatedArcs.front()));
        m_generatedArcs.pop_front();
    }
    m_currentArcPosition = 0;
    m_currentArcDistance = 0;
    m_currentArcIndex++;

    m_currentDerivative = m_currentArc->curve.derivative(0);

    updateRemainingLength();
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
        double_t remainingOnArc = m_currentArc->length - m_currentArcDistance;
        if (distance < remainingOnArc) {
            double_t increment = distance / m_currentDerivative.norm();
            m_currentArcDistance += distance;
            m_currentArcPosition += increment;
            distance = 0;
        } else {
            m_currentArcPosition = 1;
            distance -= remainingOnArc;
        }
        if (m_currentArcPosition >= 1 && m_currentArcIndex < numberOfArcs() - 1) {
            setupNextArc();
        }
    } while (distance > 0 && m_currentArcIndex < numberOfArcs() - 1);

    m_currentDerivative = m_currentArc->curve.derivative(m_currentArcPosition);

    return true;
}

Position2D<Meter> PathTrajectory::getCurrentPosition() const {
    return {m_currentArc->curve(m_currentArcPosition), m_currentDerivative.argument()};
}

std::optional<double_t> PathTrajectory::getRemainingDistance() const {
    return m_remainingLength + m_currentArc->length - m_currentArcDistance;
}

double_t PathTrajectory::getMaxCurvature(double_t distance) const {
    double_t end = m_currentArc->interpolatePosition(m_currentArcDistance + distance);
    double_t curvature = m_currentArc->curvature.getMaximum(m_currentArcPosition, end);
    distance -= m_currentArc->length - m_currentArcDistance;

    size_type anticipationCount = 0;
    while (distance >= 0) {
        if (anticipationCount >= m_generatedArcs.size()) {
            std::optional<BezierArc> arc = generateNextArc();
            if (arc) {
                m_generatedArcs.push_back(*std::move(arc));
            } else {
                break;
            }
        }
        const BezierArc &arc = m_generatedArcs[anticipationCount];
        end = arc.interpolatePosition(distance);
        curvature = std::max(curvature, arc.curvature.getMaximum(0, end));
        distance -= arc.length;
        anticipationCount++;
    }
    return curvature;
}

double_t PathTrajectory::BezierArc::interpolatePosition(double_t distance) const {
    if (distance > lengthSamples.domainEnd()) {
        return 1;
    }
    return lengthSamples.interpolate(distance);
}

const std::vector<Point2D<Meter>> &PathTrajectory::getPathPoints() const {
    return m_points;
}
const BezierCurve &PathTrajectory::getCurrentBezierArc() const {
    return m_currentArc->curve;
}
PathTrajectory::size_type PathTrajectory::getCurrentArcIndex() const {
    return m_currentArcIndex;
}

bool PathTrajectory::skipToNextArc() {
    m_currentArcPosition = 1;
    return advance(0);
}