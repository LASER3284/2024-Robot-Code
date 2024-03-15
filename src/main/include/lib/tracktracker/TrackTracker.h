#pragma once

#include "lib/tracktracker/Math.h"
#include "lib/tracktracker/TrackPoint.h"
#include "lib/tracktracker/Command.h"

#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <map>
#include <string>
#include <vector>

namespace tracktracker {

#ifdef LASER_IMPL
extern std::map<std::string, std::shared_ptr<frc2::Command>> _track_namedpoints;
#endif

class TrackTracker {
public:
    static void register_namedpoint(std::string name, frc2::CommandPtr&&);

    /// @brief Creates a sequential command based on the names of each of the points (commands).
    /// @tparam ...Names A list of types convertable to a `std::string`.
    /// @param ...point_names A list of params convertable to a `std::string`, representing the name of the registered command.
    /// @return frc2::cmd::Sequence() of each of the commands specified in the parameters list.
    template <std::convertible_to<std::string>... Names>
    frc2::CommandPtr get_auto(Names... point_names);

    /// @brief Creates a new TrackTracker. Initializes various constraints and constants.
    /// @param move_callback The callback to use when creating new `TrackPoint`s.
    /// @param pose_callback The callback to use when creating new `TrackPoint`s.
    /// @param xpid X direction PID.
    /// @param ypid Y direction PID.
    /// @param theta_pid Angular PID.
    /// @param constraints Linear and Angular constraints.
    /// @param flip_callback When to flip the points.
    /// @param requirements `frc2::Requirements` of the subsystems used for Trackpoints. Others may be specified when registering a CommandPtr.
    /// @todo Implement `TrackPoint` flipping.
    TrackTracker(
        std::function<void(frc::ChassisSpeeds)> move_callback,
        std::function<frc::Pose2d()> pose_callback,
        PidConstants xpid,
        PidConstants ypid,
        PidConstants theta_pid,
        Constraints constraints,
        std::function<bool()> flip_callback,
        frc2::Requirements requirements
    );

    /// @brief Creates a new `frc2::CommandPtr` to move to a specified end state.
    /// @param point `TrackPoint` that contains the end state
    /// @return `frc2::CommandPtr` based on the members of `this` and the given point.
    frc2::CommandPtr generate(const TrackPoint& point) {
        return frc2::CommandPtr(FollowTrackCommand{
            move_callback,
            pose_callback,
            point,
            requirements
        });
    }
private:
    std::function<void(frc::ChassisSpeeds)> move_callback;
    std::function<frc::Pose2d()> pose_callback;
    std::function<bool()> flip_callback;

    frc2::Requirements requirements;
};

}