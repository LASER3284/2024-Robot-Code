#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/PIDController.h>

#include "lib/tracktracker/TrackPoint.h"

namespace tracktracker {

class FollowTrackCommand : public frc2::CommandHelper<frc2::Command, FollowTrackCommand> {
public:
    /// @brief Creates a new `FollowTrackCommand` based on the params.
    /// @param move_callback The callback to tell the drivetrain to move.
    /// @param pose_callback The callback to obtain the pose from the drivetrain.
    /// @param point The target end state.
    /// @param requirements Subsystem requirements.
    FollowTrackCommand(
        std::function<void(frc::ChassisSpeeds)> move_callback,
        std::function<frc::Pose2d()> pose_callback,
        const TrackPoint& point,
        frc2::Requirements requirements
    );

    /// @brief Initializes the goal states of the profiles.
    void Initialize() override {
        xgoal = {point.target.Translation().X(), point.end_motion.vx};
        ygoal = {point.target.Translation().Y(), point.end_motion.vy};
        theta_goal = {point.target.Rotation().Degrees(), point.end_motion.omega};
    }

    /// @brief Runs periodically while scheduled to move the drivetrain.
    void Execute() override {
        frc::ChassisSpeeds speeds = {};
        frc::Pose2d pose = pose_cb();

        xsetp = xprofile.Calculate(20_ms, xsetp, xgoal);
        speeds.vx = units::meters_per_second_t{xpid.Calculate(pose.Translation().X().value(), xsetp.position.value())};

        ysetp = yprofile.Calculate(20_ms, ysetp, ygoal);
        speeds.vy = units::meters_per_second_t{ypid.Calculate(pose.Translation().Y().value(), ysetp.position.value())};

        theta_setp = theta_profile.Calculate(20_ms, theta_setp, theta_goal);
        speeds.omega = units::degrees_per_second_t{theta_pid.Calculate(pose.Rotation().Degrees().value(), theta_setp.position.value())};

        move_cb(speeds);
    }

    /// @brief Used to determine when the drivetrain has reached within tolerance of the goal end state
    /// @return True if the current pose is within the tolerance of the point, else false.
    bool IsFinished() override {
        return point == pose_cb();
    }

    /// @brief Action to run on the end of the command.
    /// @param interrupted True if the command was interrupted, else false.
    void End(bool interrupted) override {
        if (interrupted) {
            move_cb({});
        }
    }

private:
    TrackPoint point;
    std::function<frc::Pose2d()> pose_cb;
    std::function<void(frc::ChassisSpeeds)> move_cb;

    frc::TrapezoidProfile<units::meters> xprofile;
    frc::TrapezoidProfile<units::meters>::State xsetp;
    frc::TrapezoidProfile<units::meters>::State xgoal;
    frc::PIDController xpid;

    frc::TrapezoidProfile<units::meters> yprofile;
    frc::TrapezoidProfile<units::meters>::State ysetp;
    frc::TrapezoidProfile<units::meters>::State ygoal;
    frc::PIDController ypid;

    frc::TrapezoidProfile<units::degrees> theta_profile;
    frc::TrapezoidProfile<units::degrees>::State theta_setp;
    frc::TrapezoidProfile<units::degrees>::State theta_goal;
    frc::PIDController theta_pid;
};

}