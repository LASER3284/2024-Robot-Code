#define LASER_IMPL
#include "lib/tracktracker/TrackPoint.h"

units::meter_t tracktracker::_track_xy_tolerance;
units::degree_t tracktracker::_track_theta_tolerance;

tracktracker::TrackPoint::TrackPoint(frc::Pose2d target, frc::ChassisSpeeds end_motion) {
    this->end_motion = end_motion;
    this->target = target;
}

void tracktracker::TrackPoint::set_tolerance(units::meter_t xy, units::degree_t theta) {
    _track_xy_tolerance = units::math::abs(xy);
    _track_theta_tolerance = units::math::abs(theta);
}

bool tracktracker::TrackPoint::at_point(const frc::Pose2d& other) const {
    return
        units::math::abs(other.Translation().X() - target.Translation().X()) < _track_xy_tolerance
        && units::math::abs(other.Translation().Y() - target.Translation().Y()) < _track_xy_tolerance
        && units::math::abs(other.Rotation().Degrees() - target.Rotation().Degrees()) < _track_theta_tolerance;
}

