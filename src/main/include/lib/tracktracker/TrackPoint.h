#pragma once

#include <units/length.h>
#include <units/angle.h>
#include <frc2/command/Commands.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/geometry/Pose2d.h>

#include <functional>
#include <memory>

namespace tracktracker {

#ifdef LASER_IMPL
extern units::meter_t _track_xy_tolerance;
extern units::degree_t _track_theta_tolerance;
#endif

class TrackPoint {
public:
    /// @brief Sets the *global* tolerance for _*all*_ `TrackPoint`s.
    /// @param xy X and Y direction tolerance in meters.
    /// @param theta Angular tolerance in degrees.
    static void set_tolerance(units::meter_t xy, units::degree_t theta);

    /// @brief Creates a new `TrackPoint` to be used in a `FollowTrackCommand`.
    /// @param target Target position.
    /// @param end_motion Target motion.
    /// @see FollowTrackCommand
    TrackPoint(frc::Pose2d target, frc::ChassisSpeeds end_motion);

    /// @see operator==
    bool at_point(const frc::Pose2d& other) const;

    /// @brief Compares a given pose to the goal pose.
    /// @param other Some pose to compare to.
    /// @return True if `other` is within tolerance, else false.
    bool operator==(const frc::Pose2d& other) { return at_point(other); }

    /// @brief Flips the target pose about the field length.
    /// @param field_length Length of the field in meters.
    /// @todo Implement.
    void hflip(units::meter_t field_length) {}

    /// @brief Flips the target pose about the field height.
    /// @param field_height Height of the field in meters.
    /// @todo Implement.
    void vflip(units::meter_t field_height) {}

    /// @brief Rotates the target pose about the center of the field 180 degrees.
    /// @param field_length Length of the field in meters.
    /// @param field_height Height of the field in meters.
    /// @todo Implement.
    void rotational_flip(units::meter_t field_length, units::meter_t field_height) {}

    frc::ChassisSpeeds end_motion;

    frc::Pose2d target;
};

}