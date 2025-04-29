#ifndef _TEST_OPTIM_CURVE_PARAMETERS_HPP_
#define _TEST_OPTIM_CURVE_PARAMETERS_HPP_

#include "Clock.hpp"
#include "test_logging.hpp"
#include "trajectories/PathTrajectory.hpp"

#include <limits>
#include <memory>
#include <random>

/// Intemediary control points of a cubic Bézier curve
struct ControlPoints {
    ControlPoints(Point2D<Meter> pt1, Point2D<Meter> pt2) : pt1(pt1), pt2(pt2) {}
    Point2D<Meter> pt1;
    Point2D<Meter> pt2;

    bool operator==(const ControlPoints &other) const { return pt1 == other.pt1 && pt2 == other.pt2; }
};

double measureTime(std::unique_ptr<Trajectory> trajectory, double limit = -1);

/// Control points of a multi-Bézier trajectory
class CurveParameters {
  public:
    CurveParameters(std::vector<Point2D<Meter>> path) : m_path(std::move(path)), m_ctrlPoints() {
        /// Initial guess = current implementation of `PathTrajectory`
        PathTrajectory trajectory(std::nullopt, std::nullopt, m_path);
        trajectory.forceGenerate();

        for (std::size_t i = 0; i < trajectory.numOfGeneratedCurves(); i++) {
            const BezierCurve &curve = trajectory.getGeneratedCurve(i);
            m_ctrlPoints.push_back(ControlPoints(curve.points()[1], curve.points()[2]));
        }
    }
    double cost(std::optional<double> limit = std::nullopt) const {
        DisableLoggingGuard _guard;
        return measureTime(std::make_unique<MultiCurveTrajectory<BezierCurve>>(generateTrajectory()), limit.value_or(-1));
    }

    MultiCurveTrajectory<BezierCurve> generateTrajectory() const {
        std::vector<BezierCurve> curves;
        for (std::size_t i = 0; i < m_path.size() - 1; i++) {
            ControlPoints ctrlPts = m_ctrlPoints[i];
            curves.push_back(BezierCurve({m_path[i], ctrlPts.pt1, ctrlPts.pt2, m_path[i + 1]}));
        }
        MultiCurveTrajectory<BezierCurve> c(std::move(curves));
        c.forceGenerate();
        return c;
    }

    /// Moves the intermediary control points randomly around their current position to generate a new trajectory that passes
    /// through the same crossing points.
    ///
    /// This is experimental and subject to change.
    ///
    /// @param dist_hint The greater, the more the control points will move
    /// @return the new parameters. The current object is untouched.
    CurveParameters randomNeighbour(double_t dist_hint) const {
        CurveParameters neighbour = *this;
        neighbour.randomNeighbourInPlace(dist_hint);
        return neighbour;
    }

    bool operator==(const CurveParameters &other) const { return m_path == other.m_path && m_ctrlPoints == other.m_ctrlPoints; }

  private:
    std::vector<Point2D<Meter>> m_path;
    std::vector<ControlPoints> m_ctrlPoints;

    void randomNeighbourInPlace(double_t dist_hint) {
        for (std::size_t i = 0; i < m_path.size() - 1; i++) {
            m_ctrlPoints[i].pt2 += {rand() * dist_hint, rand() * dist_hint};
        }

        m_ctrlPoints[0].pt1 += {rand() * dist_hint, rand() * dist_hint};
        for (std::size_t i = 1; i < m_path.size() - 1; i++) {
            double_t dist = (m_ctrlPoints[i].pt1 - m_path[i]).norm();
            double_t var;
            do {
                var = rand() * dist_hint;
            } while (dist + var <= 0);

            Vector2D<Meter> direction = m_path[i] - m_ctrlPoints[i - 1].pt2;
            m_ctrlPoints[i].pt1 = m_path[i] + direction.normalize() * (dist + var);
        }
    }

    static std::normal_distribution<double_t> rng;
    static std::default_random_engine generator;

    static double_t rand() { return rng(generator); }
};

std::normal_distribution<double_t> CurveParameters::rng(0, 0.5);
std::default_random_engine CurveParameters::generator(SystemClock().micros());

#endif