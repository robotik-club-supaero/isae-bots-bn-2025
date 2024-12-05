#include "geometry/Bezier.hpp"
#include "logging.hpp"

#define INTEGRAL_NUM_STEPS 1000
#define DIRECTION_EST_STEP 0.001

// Binomial coefficient function
double binomial(unsigned int n, unsigned int i) {
    if (i > n) {
        return 0;
    }
    if (i == 0 || i == n) {
        return 1;
    }
    // NB: n > 0, otherwise `i == 0 || i > n` would hold
    if (i == 1 || i == n - 1) {
        return n;
    }
    unsigned int fact_i = 1;
    unsigned int fact_n_i = 1;
    unsigned int fact_n = 1;

    for (unsigned int k = 2; k <= n; k++) {
        fact_n *= k;
        if (k == i) {
            fact_i = fact_n;
        }
        if (k == n - i) {
            fact_n_i = fact_n;
        }
    }
    return fact_n / (fact_i * fact_n_i);
}

// Bernstein polynomial function
double_t bernstein(unsigned int n, unsigned int i, double_t t) {
    if (t < 0.0 || t > 1.0) {
        abort("t must be between 0 and 1");
    }
    return binomial(n, i) * std::pow(t, i) * std::pow(1 - t, n - i);
}

BezierCurve::BezierCurve(std::vector<Point2D<Meter>> points) : m_points(std::move(points)) {}

BezierCurve::BezierCurve(std::initializer_list<Point2D<Meter>> points) : BezierCurve(std::vector(std::move(points))) {}

Point2D<Meter> BezierCurve::operator()(double_t t) const {
    Point2D<Meter> position = {0, 0};
    for (std::size_t i = 0; i < m_points.size(); i++) {
        double_t b = bernstein(m_points.size() - 1, i, t);    
        position += m_points[i] * b;
    }
    return position;
}
Position2D<Meter> BezierCurve::position(double_t t) const {
    Point2D<Meter> point1 = this->operator()(t);
    double_t t2 = t + DIRECTION_EST_STEP;
    if (t2 > 1) {
        t2 = t - DIRECTION_EST_STEP;
    }
    Point2D<Meter> point2 = this->operator()(t2);
    Angle direction = ((point2 - point1) / (t2 - t)).argument();

    return {point1, direction};
}

double_t BezierCurve::computeLength() const {
    return computeLength(INTEGRAL_NUM_STEPS);
}
double_t BezierCurve::computeLength(unsigned int num_steps) const {
    double_t length = 0;
    Point2D<Meter> lastPoint = m_points.at(0);
    for (int i = 1; i <= INTEGRAL_NUM_STEPS; i++) {
        Point2D<Meter> currentPoint = this->operator()((double_t)i / INTEGRAL_NUM_STEPS);
        length += Vector2D<Meter>::distance(lastPoint, currentPoint);
        lastPoint = currentPoint;
    }
    return length;
}

const std::vector<Point2D<Meter>> &BezierCurve::points() const {
    return m_points;
}
std::vector<Point2D<Meter>> &BezierCurve::points() {
    return m_points;
}