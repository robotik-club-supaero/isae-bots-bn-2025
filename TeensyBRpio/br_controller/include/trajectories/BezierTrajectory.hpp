#include "geometry/Vector2D.hpp"
#include "trajectories/Trajectory.hpp"
#include <vector>

// Trajectory structure
class BezierTrajectory : public Trajectory {

  public:
    BezierTrajectory(std::vector<Point2D<Meter>> points);

    /// @copydoc Trajectory::advance()
    bool advance(double_t distance) override;

    /// @copydoc Trajectory::getCurrentPosition()
    Position2D<Meter> getCurrentPosition() const override;

    /// @copydoc Trajectory::getRemainingDistance()
    std::optional<double_t> getRemainingDistance() const override;

  private:
    std::vector<Point2D<Meter>> m_points;
    double_t m_totalLength;
    double_t m_position;
    Point2D<Meter> m_currentPosition;
    Angle m_direction;
};
