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
        Stopped,
        Down
    };

    enum SubMech {
        Flywheel = 0,
        Pivot,
        Turret
    };

    constexpr units::feet_per_second_squared_t GRAVITY = 32.175_fps_sq;
    constexpr frc::Translation2d GOAL_BLUE_POSITION { -1.5_in, 218.42_in};
    constexpr frc::Translation2d GOAL_RED_POSITION { 652.75_in, 218.42_in};
    constexpr units::foot_t DELTA_Y = 6_ft + 6_in;

    constexpr units::degree_t PIVOT_IDLE = 40_deg;
    constexpr units::degree_t TURRET_IDLE = 14_deg;

    constexpr units::degree_t PIVOT_DOWN = 10_deg;
    constexpr units::degree_t TURRET_DOWN = 0_deg;

    constexpr units::degree_t PIVOT_FEED = 40_deg;
    constexpr units::degree_t TURRET_FEED = 14_deg;

    constexpr units::feet_per_second_t SHOT_VELOCITY = 88_fps;
    constexpr units::feet_per_second_t IDLE_VELOCITY = 45_fps;
}

class Shooter : public frc2::SubsystemBase {
public:
    void init();

    /// @brief Supposed to make the thing move according to the state.
    void tick();

    /// @param robot_pose The pose of the robot according to the drive train.
    void update_nt(frc::Pose2d);

    void activate(constants::ShooterStates);

    void run_sysid(int, constants::SubMech);

    bool has_piece() { return flywheel.has_piece(); }

    bool pivot_ok() { return pivot.get_angle() < 48.5_deg && pivot.get_angle() > 12_deg; };

    bool in_place() {
        return turret.at_goal_point() && flywheel.at_speed() && pivot.at_angle();
    }

    void cancel_sysid() {
        turret.cancel_sysid();
        flywheel.cancel_sysid();
        pivot.cancel_sysid();
    }

    frc2::CommandPtr score() {
        return frc2::cmd::Sequence(
            this->Run([this]() {
                activate(constants::ShooterStates::TrackShot);
            }).WithTimeout(0.5_s),
            this->RunOnce([this]() {
                activate(constants::ShooterStates::Stopped);
            })
        );
    }

    frc2::CommandPtr feed() {
        return frc2::cmd::Sequence(
            this->RunEnd(
                [this]() {
                    activate(constants::ShooterStates::PrepFeeding);
                },
                [this]() {
                    activate(constants::ShooterStates::Stopped);
                }
            ).Until([this]() {
                return has_piece();
            })
        );
    }

    frc2::CommandPtr stable() {
        return this->RunOnce([this]() {
            activate(constants::ShooterStates::Stopped);
        });
    }

    frc2::CommandPtr track() {
        return this->RunOnce([this]() {
            activate(constants::ShooterStates::TrackingIdle);
        });
    }

    frc2::CommandPtr down() {
        return this->RunOnce([this]() {
            activate(constants::ShooterStates::Down);
        });
    }

    constants::ShooterStates get_state() const { return state; }

private:
    constants::ShooterStates state = constants::ShooterStates::Stopped;
    units::degree_t pivot_angle;
    units::degree_t turret_angle;
    subsystems::turret::Turret turret;
    subsystems::flywheel::Flywheel flywheel;
    subsystems::pivot::Pivot pivot;
};

}

}