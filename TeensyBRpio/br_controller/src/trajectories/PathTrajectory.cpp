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
        td::deque
 *  - The direction at the start point is that of `initialDirection`
 *  - The direction at the end point is that of `finalDirection` if it is non-empty. Otherwise, it is unspecified.
 */
BezierCurve<4> generateBezier(Point2D<Meter> start, Point2D<Meter> end, Vector2D<Meter> initialDirection,
                              std::optional<Vector2D<Meter>> finalDirection) {
    number_t distance = Point2D<Meter>::distance(start, end);
    number_t ctrlPtDistance = distance / 3;

    Point2D<Meter> controlPt1 = start + initialDirection.normalize() * ctrlPtDistance;
    Point2D<Meter> controlPt2 = end - finalDirection.value_or(end - controlPt1).normalize() * ctrlPtDistance;

    return BezierCurve<4>({start, controlPt1, controlPt2, end});
}

BezierCurvesGenerator::BezierCurvesGenerator(std::optional<Angle> initialDirection, std::optional<Angle> finalDirection,
                                             SmallDeque<Point2D<Meter>> &&points)
    : m_points(std::move(points)), m_initialDirection(), m_finalDirection(finalDirection) {
    if (m_points.size() < 2) {
        abort("Trajectory must have at least 2 points");
    }

    Vector2D<Meter> preferredDirection;
    if (m_points.size() > 2) {
        Angle theta1 = (m_points[1] - m_points[0]).argument();
        Angle theta2 = (m_points[2] - m_points[1]).argument();

        preferredDirection = (theta1 - Angle(0.5 * (theta2 - theta1))).toHeadingVector();
    } else {
        preferredDirection = m_points[1] - m_points[0];
    }
    // If the robot is already heading in the right direction, we bypass state "initial rotation" and immediately start the trajectory
    // Otherwise, as we are not moving yet, it is more efficient (and safer) to turn now than starting to move in the wrong direction.
    if (initialDirection) {
        if (std::abs(*initialDirection - preferredDirection.argument()) <= Angle::Pi / 3) {
            preferredDirection = initialDirection->toHeadingVector();
        }
    }
    m_initialDirection = preferredDirection;
}

BezierCurve<4> BezierCurvesGenerator::next() {

    std::optional<Vector2D<Meter>> finalDirection = std::nullopt;

    const Point2D<Meter> &start = m_points[0];
    const Point2D<Meter> &end = m_points[1];

    if (m_points.size() > 2) {
        const Point2D<Meter> &next = m_points[2];
        finalDirection = angleBisector(end - start, next - end);
    } else if (m_finalDirection && std::abs(*m_finalDirection - (end - start).argument()) < Angle::Pi / 2) {
        finalDirection = m_finalDirection->toHeadingVector();
    }
    BezierCurve curve = generateBezier(m_points[0], m_points[1], m_initialDirection, finalDirection);
    m_initialDirection = curve.derivative(1);

    m_points.pop_front();
    return curve;
}
bool BezierCurvesGenerator::hasNext() const {
    return m_points.size() > 1;
}

const SmallDeque<Point2D<Meter>> &BezierCurvesGenerator::getPoints() const {
    return m_points;
}
SmallDeque<Point2D<Meter>> &BezierCurvesGenerator::getPoints() {
    return m_points;
}

const std::optional<Angle> &BezierCurvesGenerator::getPreferredFinalDirection() const {
    return m_finalDirection;
}

PathTrajectory::PathTrajectory(std::optional<Angle> initialDirection, std::optional<Angle> finalDirection, SmallDeque<Point2D<Meter>> points)
    : MultiCurveTrajectory(BezierCurvesGenerator(initialDirection, finalDirection, std::move(points))) {}

const SmallDeque<Point2D<Meter>> &PathTrajectory::getPathPoints() const {
    return getGenerator().getPoints();
}

bool PathTrajectory::recompute(Position2D<Meter> newStartPosition) {
    SmallDeque<Point2D<Meter>> &path = getGenerator().getPoints();
    if (!path.push_front(newStartPosition)) {
        // This should not happen, because the first point is removed when the trajectory is created
        // (and everytime `BezierCurvesGenerator::next` is called), and the deque does not shrink automatically.
        return false;
    }
    *this = PathTrajectory(newStartPosition.theta, getGenerator().getPreferredFinalDirection(), std::move(path));
    return true;
}