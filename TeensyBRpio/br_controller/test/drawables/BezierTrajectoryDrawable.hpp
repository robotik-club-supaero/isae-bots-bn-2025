#ifndef _TEST_BEZIER_TRAJECTORY_DRAWABLE_HPP_
#define _TEST_BEZIER_TRAJECTORY_DRAWABLE_HPP_

#include "drawables/BezierDrawable.hpp"
#include "drawables/TrajectoryDrawable.hpp"

#include "trajectories/MultiCurveTrajectory.hpp"

/// Draws multi-Bézier-curve trajectory with intermediary control points.
class BezierTrajectoryDrawable : public TrajectoryDrawable {
  public:
    template <CurveGenerator<BezierCurve<4>> TGenerator>
    BezierTrajectoryDrawable(const TableTransform &transform, MultiCurveTrajectory<BezierCurve<4>, TGenerator> &trajectory, double_t step,
                             sf::Color color = DEFAULT_COLOR, sf::Color secondaryColor = BezierDrawable::DEFAULT_COLOR)
        : TrajectoryDrawable(color), m_curves() {
        trajectory.forceGenerate();
        for (std::size_t i = 0; i < trajectory.numOfGeneratedCurves(); i++) {
            m_curves.push_back(BezierDrawable(transform, trajectory.getGeneratedCurve(i), secondaryColor));
        }

        appendTrajectory(transform, trajectory, step);
    }

  protected:
    void draw(sf::RenderTarget &target, sf::RenderStates states) const override {
        TrajectoryDrawable::draw(target, states);
        for (const BezierDrawable &curve : m_curves) {
            target.draw(curve, states);
        }
    }

  private:
    std::vector<BezierDrawable> m_curves;
};

#endif