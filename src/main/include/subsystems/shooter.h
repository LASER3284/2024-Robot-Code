#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include "turret.h"
#include "flywheel.h"
#include "pivot.h"
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>

// sysid
#include <frc2/command/Commands.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

namespace subsystems {

namespace shooter {

namespace constants {
    enum ShooterStates {
        TrackingIdle = 0,
        TrackShot,
        StableIdle,
        StableShot,
        LowStableShot,
        PrepFeeding,
        Stopped
    };

    enum SubMech {
        Flywheel = 0,
        Pivot,
        Turret
    };

    constexpr units::feet_per_second_squared_t GRAVITY = 32.175_fps_sq;
    constexpr frc::Translation2d GOAL_BLUE_POSITION { -1.5_in, 218.42_in};
    constexpr frc::Translation2d GOAL_RED_POSITION { 652.75_in, 218.42_in};
    constexpr units::foot_t DELTA_Y = 6.67_ft;

    constexpr units::degree_t PIVOT_IDLE = 0_deg;
    constexpr units::degree_t TURRET_IDLE = 0_deg;

    constexpr units::feet_per_second_t SHOT_VELOCITY = 72_fps;
    constexpr units::feet_per_second_t IDLE_VELOCITY = 20_fps;
}

class Shooter : public frc2::SubsystemBase {
public:
    void init();

    /// @brief Supposed to make the thing move according to the state.
    /// @param robot_pose The pose of the robot according to the drive train.
    void tick(frc::Pose2d);

    void update_nt();

    void activate(constants::ShooterStates);

    void run_sysid(int, constants::SubMech);

    bool has_piece() { return flywheel.has_piece(); }

    void cancel_sysid() {
        turret.cancel_sysid();
        flywheel.cancel_sysid();
        pivot.cancel_sysid();
    }

    frc2::CommandPtr score() {
        return frc2::cmd::Sequence(
            frc2::cmd::RunOnce([this]() {
                activate(constants::ShooterStates::TrackShot);
            }),
            frc2::cmd::Wait(1.5_s),
            frc2::cmd::RunOnce([this]() {
                activate(constants::ShooterStates::StableIdle);
            })
        );
    }

    frc2::CommandPtr feed() {
        return frc2::cmd::Sequence(
            frc2::cmd::StartEnd(
                [this]() {
                    activate(constants::ShooterStates::PrepFeeding);
                },
                [this]() {
                    activate(constants::ShooterStates::StableIdle);
                }
            ).Until([this]() {
                return has_piece();
            })
        );
    }

    constants::ShooterStates get_state() const { return state; }

private:
    constants::ShooterStates state = constants::ShooterStates::Stopped;
    subsystems::turret::Turret turret;
    subsystems::flywheel::Flywheel flywheel;
    subsystems::pivot::Pivot pivot;
};

}

}