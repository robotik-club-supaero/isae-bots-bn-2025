#include "trajectories/PathTrajectory.hpp"
#include "logging.hpp"

/**
 * Returns the unnormalized direction of the angle bisector between lineA and lineB.
 */
inline Vector2D<Meter> angleBisector(Vector2D<Meter> lineA, Vector2D<Meter> lineB) {
    return lineA.normalize() + lineB.normalize();
}

/**
 * Generates a Bézier curve of degree 3 such that:
 *  - The curve goes from `start` to `end`
 *  - The direction at the start point is that of `initialDirection`
 *  - The direction at the end point is that of `finalDirection` if it is non-empty. Otherwise, it is unspecified.
 */
BezierCurve generateBezier(Point2D<Meter> start, Point2D<Meter> end, Vector2D<Meter> initialDirection,
                           std::optional<Vector2D<Meter>> finalDirection) {
    double_t distance = Point2D<Meter>::distance(start, end);
    double_t ctrlPtDistance = distance / 3;

    Point2D<Meter> controlPt1 = start + initialDirection.normalize() * ctrlPtDistance;
    Point2D<Meter> controlPt2 = end + finalDirection.value_or(controlPt1 - end).normalize() * ctrlPtDistance;

    return {start, controlPt1, controlPt2, end};
}

BezierCurvesGenerator::BezierCurvesGenerator(std::optional<Angle> initialDirection, std::optional<Angle> finalDirection,
                                             std::vector<Point2D<Meter>> points)
    : m_points(std::move(points)), m_initialDirection(), m_finalDirection(finalDirection), m_currentCurveIndex(0) {
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

    std::optional<Vector2D<Meter>> finalDirection = std::nullopt;

    const Point2D<Meter> &start = m_points[index];
    const Point2D<Meter> &end = m_points[index + 1];

    if (index + 2 < m_points.size()) {
        const Point2D<Meter> &next = m_points[index + 2];
        finalDirection = angleBisector(start - end, end - next);
    } else if (m_finalDirection && abs(*m_finalDirection - (end - start).argument()) < Angle::Pi / 2) {
        finalDirection = m_finalDirection->toHeadingVector();
    }
    BezierCurve curve = generateBezier(m_points[index], m_points[index + 1], m_initialDirection, finalDirection);
    m_initialDirection = curve.derivative(1);

    return curve;
}
bool BezierCurvesGenerator::hasNext() const {
    return m_currentCurveIndex + 1 < m_points.size();
}

const std::vector<Point2D<Meter>> &BezierCurvesGenerator::getPoints() const {
    return m_points;
}

const std::optional<Angle> &BezierCurvesGenerator::getPreferredFinalDirection() const {
    return m_finalDirection;
}

PathTrajectory::PathTrajectory(std::optional<Angle> initialDirection, std::optional<Angle> finalDirection, std::vector<Point2D<Meter>> points)
    : MultiCurveTrajectory(BezierCurvesGenerator(initialDirection, finalDirection, std::move(points))) {}

const std::vector<Point2D<Meter>> &PathTrajectory::getPathPoints() const {
    return getGenerator().getPoints();
}

bool PathTrajectory::recompute(Position2D<Meter> newStartPosition) {
    const std::vector<Point2D<Meter>> &initialPath = getPathPoints();
    std::vector<Point2D<Meter>> newPoints;
    newPoints.push_back(newStartPosition);
    newPoints.insert(newPoints.end(), initialPath.begin() + getCurrentCurveIndex() + 1, initialPath.end());

    *this = PathTrajectory(newStartPosition.theta, getGenerator().getPreferredFinalDirection(), newPoints);

    return true;
}