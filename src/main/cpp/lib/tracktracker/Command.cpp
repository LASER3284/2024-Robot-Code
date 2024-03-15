#define LASER_IMPL
#include "lib/tracktracker/Global.h"
#include "lib/tracktracker/Command.h"

tracktracker::FollowTrackCommand::FollowTrackCommand(
    std::function<void(frc::ChassisSpeeds)> move_callback,
    std::function<frc::Pose2d()> pose_callback,
    const TrackPoint& point,
    frc2::Requirements requirements
) : point(point),
    xprofile(_track_constraints.linear), xpid(_track_xpid.kP, _track_xpid.kI, _track_xpid.kD),
    yprofile(_track_constraints.linear), ypid(_track_ypid.kP, _track_ypid.kI, _track_ypid.kD),
    theta_profile(_track_constraints.angular), theta_pid(_track_theta_pid.kP, _track_theta_pid.kI, _track_theta_pid.kD)
{
    AddRequirements(requirements);
    move_cb = move_callback;
    pose_cb = pose_callback;
}