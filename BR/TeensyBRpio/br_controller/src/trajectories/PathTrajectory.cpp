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
 *  - The direction at the start point is `initialDirection`
 *  - If `next` is non-empty, the direction at the end point is the mean between [start, end] and [end, next].
 *
 * @param next The next destination of the robot, *after* the generated Bézier curve, or `std::nullopt` if `end` is the final destination.
 */
BezierCurve generateBezier(Point2D<Meter> start, Point2D<Meter> end, Angle initialDirection, std::optional<Point2D<Meter>> next) {
    double_t distance = Point2D<Meter>::distance(start, end);
    double_t ctrlPtDistance = distance / 3;

    Point2D<Meter> controlPt1 = start + Point2D<Meter>(std::cos(initialDirection), std::sin(initialDirection)) * ctrlPtDistance;

    Vector2D<Meter> finalDirection;
    if (next) {
        finalDirection = angleBisector(start - end, end - *next);
    } else {
        finalDirection = controlPt1 - end;
    }
    Point2D<Meter> controlPt2 = end + finalDirection.normalize() * ctrlPtDistance;

    return {start, controlPt1, controlPt2, end};
}

PathTrajectory::PathTrajectory(Angle initialDirection, std::vector<Point2D<Meter>> points)
    : m_points(points), m_currentArcIndex(0), m_currentArc(), m_currentArcPosition(0), m_currentDerivative(), m_remainingLength(0) {
    if (points.size() < 2) {
        abort("Trajectory must have at least 2 points");
    }

    Angle preferredDirection = initialDirection;
    if (points.size() > 2) {
        preferredDirection = angleBisector(points[1] - points[0], 2 * points[1] - (points[0] + points[2])).argument();
    } else {
        preferredDirection = (points[1] - points[0]).argument();
    }
    if (abs(initialDirection - preferredDirection) > Angle::Pi / 3) {
        // Force the controller to enter state "initial rotation" before starting the trajectory
        // As we are not moving yet, it is more efficient (and safer) to turn now than starting to move in the wrong direction.
        initialDirection = preferredDirection;
    }

    m_currentArc.emplace(generateArc(0, initialDirection));
    updateRemainingLength();

    m_currentDerivative = m_currentArc->curve.derivative(0);
}

std::optional<PathTrajectory::BezierArc> PathTrajectory::generateNextArc() const {
    size_type nextIndex = m_currentArcIndex + 1 + m_generatedArcs.size();
    if (nextIndex < numberOfArcs()) {
        const BezierArc &lastGeneratedArc = (m_generatedArcs.empty() ? *m_currentArc : m_generatedArcs.back());
        return generateArc(nextIndex, lastGeneratedArc.curve.derivative(1).argument());
    } else {
        return std::nullopt;
    }
}

PathTrajectory::BezierArc PathTrajectory::generateArc(size_type index, Angle initialDirection) const {
    std::optional<Point2D<Meter>> next = std::nullopt;
    if (index + 1 < numberOfArcs()) {
        next = std::make_optional(m_points[index + 2]);
    }
    BezierCurve curve = generateBezier(m_points[index], m_points[index + 1], initialDirection, next);
    double_t length = curve.computeLength();

    return {std::move(curve), length};
}

void PathTrajectory::setupNextArc() {
    if (m_generatedArcs.empty()) {
        m_currentArc.emplace(*generateNextArc());
    } else {
        m_currentArc.emplace(std::move(m_generatedArcs.front()));
        m_generatedArcs.pop_front();
    }
    m_currentArcPosition = 0;
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
        double_t remainingOnArc = m_currentArc->length * (1 - m_currentArcPosition);
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
    return m_remainingLength + m_currentArc->length * (1 - m_currentArcPosition);
}

double_t PathTrajectory::getMaxCurvature(double_t distance) const {
    // We assume the curvature is maximal when the (absolute value of the) second derivative is maximal, which is NOT true
    // but should give an acceptable approximation for the kind of curves we are using.

    auto __maxCurvatureInner = [](const BezierArc &arc, double_t start, double_t *distance) {
        // This interpolation assumes dB/dt = cste, which is obviously NOT true but should give an acceptable approximation.
        double_t end = start + *distance / arc.length;
        if (end >= 1) {
            *distance -= arc.length * (1 - start);
        } else {
            *distance = -1;
        }

        // We are using cubic Bézier curves, so the second derivative is monotonic.
        return std::max(abs(arc.curve.curvature(start)), distance == 0 ? 0 : abs(arc.curve.curvature(std::min((double_t)1., end))));
    };

    double_t curvature = __maxCurvatureInner(*m_currentArc, m_currentArcPosition, &distance);

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
        curvature = std::max(curvature, __maxCurvatureInner(m_generatedArcs[anticipationCount], 0, &distance));
        anticipationCount++;
    }

    return curvature;
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