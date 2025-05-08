#ifndef _DEFINE_STL_HPP_
#define _DEFINE_STL_HPP_

#include "fsm/PreallocUniquePtr.hpp"

#include "rotations/OrientationProfile.hpp"
#include "rotations/SetHeadingProfile.hpp"
#include "trajectories/LinearTrajectory.hpp"
#include "trajectories/PathTrajectory.hpp"
#include "trajectories/PolygonalTrajectory.hpp"

using trajectory_ptr = unique_ptr_for<Trajectory, LinearTrajectory, PathTrajectory, PolygonalTrajectory>::type;
using rotation_ptr = unique_ptr_for<OrientationProfile, SetHeadingProfile>::type;

#endif