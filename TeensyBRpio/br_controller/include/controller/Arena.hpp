#ifndef _CONTROLLER_ARENA_HPP_
#define _CONTROLLER_ARENA_HPP_

#include "configuration.hpp"
#include "rotations/SetHeadingProfile.hpp"
#include "stl/PreallocUniquePtr.hpp"
#include "trajectories/PathTrajectory.hpp"
#include "trajectories/PolygonalTrajectory.hpp"

namespace controller {
struct Arena {
    using trajectory_ptr = unique_ptr_for<Trajectory, LinearTrajectory, PathTrajectory, PolygonalTrajectory>::type;
    using rotation_ptr = unique_ptr_for<OrientationProfile, SetHeadingProfile>::type;

    Arena() : currentTrajectory(), currentRotation() {}

    /// Pre-allocated buffer to store the current trajectory
    trajectory_ptr currentTrajectory;
    /// Pre-allocated buffer to store the current rotation profile
    rotation_ptr currentRotation;
};
} // namespace controller

#endif