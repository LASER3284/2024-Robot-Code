#define LASER_IMPL
#include "lib/tracktracker/TrackTracker.h"
#include "lib/tracktracker/Global.h"

std::map<std::string, std::shared_ptr<frc2::Command>> tracktracker::_track_namedpoints = {};

void tracktracker::TrackTracker::register_namedpoint(std::string name, frc2::CommandPtr&& command) {
    _track_namedpoints[name] = {std::move(command).Unwrap()};
}

tracktracker::TrackTracker::TrackTracker(
    std::function<void(frc::ChassisSpeeds)> move_callback,
    std::function<frc::Pose2d()> pose_callback,
    PidConstants xpid,
    PidConstants ypid,
    PidConstants theta_pid,
    Constraints constraints,
    std::function<bool()> flip_callback,
    frc2::Requirements requirements
) {
    this->move_callback = move_callback;
    this->pose_callback = pose_callback;
    this->flip_callback = flip_callback;
    this->requirements = requirements;

    _track_xpid = xpid;
    _track_ypid = ypid;
    _track_theta_pid = theta_pid;
    _track_constraints = constraints;
}

template <std::convertible_to<std::string>... Names>
frc2::CommandPtr tracktracker::TrackTracker::get_auto(Names... names) {
    std::vector<frc2::CommandPtr> commands = {frc2::cmd::None()};
    for (auto name : {names...}) {
        if (_track_namedpoints.contains(name))
            commands.push_back(_track_namedpoints[name]);
    }

    if (commands.size() == 0) {
        commands.push_back(frc2::cmd::None());
    }

    return frc2::cmd::Sequence(std::move(commands));
}