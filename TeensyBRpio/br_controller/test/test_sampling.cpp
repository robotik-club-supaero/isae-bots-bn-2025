#include "trajectories/PathTrajectory.hpp"

#include <cassert>

static constexpr number_t STEP = 0.005;

inline bool dbl_eq(number_t a, number_t b, number_t tol = 1e-9) {
    return std::abs(a - b) < tol;
}

void test_sample_line() {
    Point2D<Meter> start;
    Point2D<Meter> end(0.79, 1.43);
    number_t dist = Point2D<Meter>::distance(start, end);

    BezierCurve<2> curve({start, end});
    CurveSampling sampler(STEP);
    sampler.sample(curve, std::numeric_limits<number_t>::infinity());

    assert(sampler.samplingComplete() && dbl_eq(dist, sampler.length()));
    assert(dbl_eq(sampler.solveInverseArcLength(0), 0));
    assert(dbl_eq(sampler.solveInverseArcLength(dist), 1));

    assert(dbl_eq(sampler.solveInverseArcLength(dist / 2), 0.5));
}

void test_rolling_max() {
    auto fun = [](number_t value) { return value * value; };

    RollingMax rolling_max(STEP, /* initialStart = */ -1);
    assert(dbl_eq(rolling_max.getMaximum(-1, -0.5, fun), fun(-1)));

    rolling_max = RollingMax(STEP, /* initialStart = */ -1);
    assert(dbl_eq(rolling_max.getMaximum(0.2, 0.5, fun), fun(0.5)));

    rolling_max = RollingMax(STEP, /* initialStart = */ -1);
    assert(dbl_eq(rolling_max.getMaximum(-0.4, 0.5, fun), fun(0.5)));
    assert(rolling_max.numberOfSampledExtrema() == 1);

    rolling_max = RollingMax(STEP, /* initialStart = */ -1);
    assert(dbl_eq(rolling_max.getMaximum(-0.7, 0.5, fun), fun(-0.7)));
}

void test_rolling_max() {
    auto fun = [](number_t value) { return value * value; };

    RollingMax rolling_max(STEP, /* initialStart = */ -1);
    assert(dbl_eq(rolling_max.getMaximum(-1, -0.5, fun), fun(-1)));

    rolling_max = RollingMax(STEP, /* initialStart = */ -1);
    assert(dbl_eq(rolling_max.getMaximum(0.2, 0.5, fun), fun(0.5)));

    rolling_max = RollingMax(STEP, /* initialStart = */ -1);
    assert(dbl_eq(rolling_max.getMaximum(-0.4, 0.5, fun), fun(0.5)));
    assert(rolling_max.numberOfSampledExtrema() == 1);

    rolling_max = RollingMax(STEP, /* initialStart = */ -1);
    assert(dbl_eq(rolling_max.getMaximum(-0.7, 0.5, fun), fun(-0.7)));
}

void test_dyn_max() {
    SmallDeque<Point2D<Meter>> path;
    path.push_back(Point2D<Meter>());
    path.push_back(Point2D<Meter>(400., 50.));
    path.push_back(Point2D<Meter>(1500., 320.));

    PathTrajectory traj(0, 0, path);

}

int main() {
    test_sample_line();
    return 0;
}