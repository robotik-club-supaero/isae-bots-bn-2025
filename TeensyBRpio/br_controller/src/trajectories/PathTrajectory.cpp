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
 *  - The direction at the start point is that of `initialDirection`
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

BezierCurvesGenerator::BezierCurvesGenerator(std::optional<Angle> initialDirection, std::vector<Point2D<Meter>> points)
    : m_points(std::move(points)), m_initialDirection(), m_currentCurveIndex(0) {
    std::vector<size_type> duplicates;
    if (m_points.size() < 2) {
        abort("Trajectory must have at least 2 points");
    }

    Vector2D<Meter> preferredDirection;
    if (m_points.size() > 2) {
        preferredDirection = angleBisector(m_points[1] - m_points[0], 1.5 * m_points[1] - (m_points[0] + 0.5 * m_points[2]));
    } else {
        preferredDirection = m_points[1] - m_points[0];
    }
    // If the robot is already heading in the right direction, we bypass state "initial rotation" and immediately start the trajectory
    // Otherwise, as we are not moving yet, it is more efficient (and safer) to turn now than starting to move in the wrong direction.
    if (initialDirection) {
        if (abs(*initialDirection - preferredDirection.argument()) <= Angle::Pi / 3) {
            preferredDirection = {std::cos(*initialDirection), std::sin(*initialDirection)};
        }
    }
    m_initialDirection = preferredDirection;
}

BezierCurve BezierCurvesGenerator::next() {
    size_type index = m_currentCurveIndex++;

    std::optional<Point2D<Meter>> next = std::nullopt;
    if (index + 2 < m_points.size()) {
        next = m_points[index + 2];
    }
    BezierCurve curve = generateBezier(m_points[index], m_points[index + 1], m_initialDirection, next);
    m_initialDirection = curve.derivative(1);

    return curve;
}
bool BezierCurvesGenerator::hasNext() const {
    return m_currentCurveIndex + 1 < m_points.size();
}

const std::vector<Point2D<Meter>> &BezierCurvesGenerator::getPoints() const {
    return m_points;
}
PathTrajectory::PathTrajectory(std::optional<Angle> initialDirection, std::vector<Point2D<Meter>> points)
    : MultiCurveTrajectory(BezierCurvesGenerator(initialDirection, std::move(points))) {}

const std::vector<Point2D<Meter>> &PathTrajectory::getPathPoints() const {
    return getGenerator().getPoints();
}