#ifndef _TRAJECTORY_HPP_
#define _TRAJECTORY_HPP_

#include "defines/math.hpp"
#include "geometry/Position2D.hpp"

#include <optional>

/**
 * A continuous 2D-trajectory the robot is expected to follow. The direction of the trajectory is allowed to be
 * discontinuous provided that the points of discontinuity are a discrete set.
 *
 * The trajectory is described by the successive positions of a point that moves along the trajectory.
 * This point can be seen as the target position for the robot.
 *
 * When the `Trajectory` instance is created, this point must be at the beginning of the trajectory.
 * The implementation is responsible for keeping track of this point internally as it advances.
 *
 * This class must be inherited. Smooth trajectories should inherit class `SmoothTrajectory` instead.
 *
 * @see SmoothTrajectory.
 */
class Trajectory {
  public:
    /**
     * Advances the current position on the trajectory by the given distance.
     * @param distance The distance (in meters) between the current trajectory position and the new position. If the new position would fall
     * outside the trajectory, then the position should be advanced to the end of the trajectory instead.
     * @return true if the position was advanced. false if the position is already at the end of the trajectory. If this method returns false,
     * subsequent calls to advance() must also return false.
     *
     * If `distance == 0`:
     * - if the current position is just before a point of discontinuity, this must move it just after the discontinuity and return true,
     * - otherwise this must be a no-op and return `false` only if the current position is already at the end of the trajectory.
     *
     * If `distance < 0`, the behavior is undefined.
     */
    virtual bool advance(number_t distance) = 0;

    /**
     * This is the same as `advance` but must stop at points of discontinuity.
     * - If the direction of the trajectory is continuous for the next `distance` meters, this must be equivalent to calling `advance`.
     * - Otherwise, this must stop just before the discontinuity. Then subsequent calls to advanceSmooth() must do nothing and return false until
     * `advance` is called.
     */
    virtual bool advanceSmooth(number_t distance) = 0;

    /**
     * Returns the current position on the trajectory. The position must be heading forward in the current local direction of the trajectory.
     * After advance() has returned false, this must return the last point of the trajectory.
     */
    virtual Position2D<Meter> getCurrentPosition() const = 0;

    /**
     * Returns the distance between the current position and the end of the trajectory. This allows to implement a deceleration ramp to stop the robot
     * as close to the end of the trajectory as possible (without overshoot and ringing).
     *
     * - 0 means the current position has reached the end of the trajectory. It is a logic error to return 0 unless advance() would return false.
     * - An empty optional means the trajectory is inifite or the remaining distance cannot be determined.
     * - Negative values are illegal and will lead to unspecified behaviour.
     *
     * After advance() has returned false, this must return 0.
     */
    virtual std::optional<number_t> getRemainingDistance() const = 0;

    /**
     * Returns the maximum absolute curvature of the trajectory for the next `distance` meters, starting at the current position.
     * The curvature is used to anticipate deceleration in turns. This function is not required to be `const` so that lazily-computed trajectories
     * may update the generated portion of the trajectory when this function is called.
     *
     * - Returning 0 means the trajectory is a straight line and effectively disables turn anticipation.
     *   0 is also a valid default value for trajectories that do not support curvature estimation.
     * - Returning +Inf means there is a discontinuity in the direction of the trajectory in the next `distance` meters.
     * - Returning a negative value is illegal and unspecified behavior.
     *
     * If `distance < 0`, the behavior is undefined.
     */
    virtual number_t getMaxCurvature(number_t distance) = 0;

    /**
     * Attempts to recompute the trajectory such that:
     * - The new start position is `newStartPosition`. The initial direction of the trajectory is not required to match the robot's orientation.
     * - The current position is reset to the beginning of the trajectory.
     * - The recomputed trajectory eventually catches up with the remaining part of the initial trajectory (what this exactly means depends on the
     * trajectory). If the initial trajectory is finite, this usually means the recomputed trajectory is finite as well and has the same final
     * position.
     *
     * @returns true if the trajectory was successfully recomputed, false otherwise. If this returns false, the state of this object must not have
     * changed.
     *
     * The default implementation always returns false.
     *
     */
    virtual bool recompute(Position2D<Meter> newStartPosition) { return false; }

    virtual ~Trajectory() = default;

  protected:
    Trajectory() = default;
};

/**
 * A reasonably smooth, i.e. at least of class C2, 2D-trajectory.
 *
 * # Contract:
 *
 * - The direction of the trajectory must be continuous.
 * - As a result, `getMaxCurvature` must never return +Inf.
 *
 * This class must be inherited.
 *
 * @see Trajectory
 */
class SmoothTrajectory : public Trajectory {
  public:
    /**
     * If you know the trajectory is smooth, you should call `advance` directly instead.
     *
     * @copydoc Trajectory::advanceSmooth */
    bool advanceSmooth(number_t distance) override final { return advance(distance); }

  protected:
    SmoothTrajectory() = default;
};

#endif