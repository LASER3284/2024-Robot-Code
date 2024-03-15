#pragma once

/// This file likely shouldn't be used by regular users of this library. It is used internally by TrackTracker.

#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

#include "lib/tracktracker/Math.h"

namespace tracktracker {

#ifdef LASER_IMPL
extern Constraints _track_constraints;

extern PidConstants _track_xpid;
extern PidConstants _track_ypid;
extern PidConstants _track_theta_pid;
#endif

}