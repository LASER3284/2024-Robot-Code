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
#include <frc/DriverStation.h>


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
        TrackForceShot,
        SubScore,
        StableIdle,
        StableShot,
        ReverseFeed,
        LowStableShot,
        CreamyShot,
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
    constexpr units::foot_t DELTA_Y = 7_ft;

    constexpr auto PIVOT_CORRECTION = 0.2_deg / 1_ft;
    constexpr auto TURRET_CORRECTION = -0.075_deg / 1_ft;
    constexpr auto FLYWHEEL_CORRECTION_CREAMY = 2.1_fps / 1_ft;

    constexpr units::degree_t PIVOT_IDLE = 40_deg;
    constexpr units::degree_t TURRET_IDLE = 14_deg;

    constexpr units::degree_t PIVOT_DOWN = 10_deg;
    constexpr units::degree_t TURRET_DOWN = 0_deg;

    constexpr units::degree_t PIVOT_FEED = 40_deg;
    constexpr units::degree_t TURRET_FEED = 14_deg;

    constexpr units::feet_per_second_t SHOT_VELOCITY = 77.5_fps;
    constexpr units::feet_per_second_t IDLE_VELOCITY = 45_fps;

    // change these number keaton and cameron 
    constexpr units::degree_t SUB_PIVOT_ANGLE = 70_deg;
    constexpr units::degree_t SUB_TURRET_ANGLE = 0_deg;

    constexpr units::degree_t CREAMY_PIVOT_ANGLE = 45_deg;
    constexpr units::degree_t CREAMY_TURRET_ANGLE = 0_deg;
    constexpr units::feet_per_second_t CREAMY_VELOCITY = 65_fps;
}

class Shooter : public frc2::SubsystemBase {
public:
    /// @brief Initializes the submechanisms of the shooter.
    void init();

    /// @brief Supposed to make the thing move according to the state.
    void tick();

    /// @brief Updates the goal positions of the shooter, and updates NT
    /// @param robot_pose The pose of the robot according to the drive train.
    void update_nt(frc::Pose2d);

    /// @brief Updates the internal state of the shooter to make it do a
    /// different thing.
    void activate(constants::ShooterStates);

    /// @brief Runs SysID for the specified submechanism.
    /// @param test_num The test to run
    /// @param submechanism The specific submechanism to run SysID on.
    /// @see Robot::SysIdChooser
    void run_sysid(int, constants::SubMech);

    /// @brief Returns if the flywheel submechanism detects a piece.
    /// @retun See brief :)
    bool has_piece() { return flywheel.has_piece(); }

    /// @brief Returns true if the Pivot is within the safe zone.
    /// @return See brief :)
    bool pivot_ok() { return pivot.get_angle() < 48.5_deg && pivot.get_angle() > 12_deg; };

    /// @brief Returns true if all three submechanisms are within their
    /// tolerances.
    bool in_place() {
        switch (state) {
        case constants::ShooterStates::Stopped: {
            return units::math::abs(turret.get_angle() - constants::TURRET_IDLE) < turret::constants::TOLERANCE
                && units::math::abs(pivot.get_angle() - constants::PIVOT_IDLE) < pivot::constants::TOLERANCE
                && units::math::abs(flywheel.get_exit_vel()) < flywheel::constants::TOLERANCE; 
        }
        break;
        case constants::ShooterStates::Down: {
            return units::math::abs(turret.get_angle() - constants::TURRET_DOWN) < turret::constants::TOLERANCE
                && units::math::abs(pivot.get_angle() - constants::PIVOT_DOWN) < pivot::constants::TOLERANCE
                && units::math::abs(flywheel.get_exit_vel()) < flywheel::constants::TOLERANCE; 
        }
        break;
        case constants::ShooterStates::TrackShot:
        case constants::ShooterStates::TrackForceShot:
        case constants::ShooterStates::TrackingIdle: {
            return units::math::abs(turret.get_angle() - (turret_angle - 4_deg)) < turret::constants::TOLERANCE
                && units::math::abs(pivot.get_angle() - pivot_angle) < pivot::constants::TOLERANCE
                && units::math::abs(flywheel.get_exit_vel() - constants::SHOT_VELOCITY) < flywheel::constants::TOLERANCE; 
        }
        break;
        case constants::ShooterStates::SubScore: {
            return units::math::abs(turret.get_angle() - constants::SUB_TURRET_ANGLE) < turret::constants::TOLERANCE
                && units::math::abs(pivot.get_angle() - constants::SUB_PIVOT_ANGLE) < pivot::constants::TOLERANCE
                && units::math::abs(flywheel.get_exit_vel() - constants::SHOT_VELOCITY) < flywheel::constants::TOLERANCE; 
        }
        break;
        case constants::ShooterStates::CreamyShot: {
            return units::math::abs(turret.get_angle() - (turret_angle - 15_deg * (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed ? -1 : 1))) < turret::constants::TOLERANCE
                && units::math::abs(pivot.get_angle() - constants::CREAMY_PIVOT_ANGLE) < pivot::constants::TOLERANCE
                && units::math::abs(flywheel.get_exit_vel() - (hypot * constants::FLYWHEEL_CORRECTION_CREAMY)) < flywheel::constants::TOLERANCE; 
        }
        break;
        default: {
            return false;
        }
        break;
    }
    }

    /// @brief Cancels the SysId command for each of the submechanisms.
    void cancel_sysid() {
        turret.cancel_sysid();
        flywheel.cancel_sysid();
        pivot.cancel_sysid();
    }

    /// @brief A command to score a piece and then stop the shooter.
    /// @return frc2::CommandPtr representing a new command.
    frc2::CommandPtr score() {
        return frc2::cmd::Sequence(
            this->Run([this]() {
                scratch = state;
                activate(constants::ShooterStates::TrackShot);
            }).WithTimeout(0.5_s),
            this->RunOnce([this]() {
                activate(scratch);
            })
        );
    }

    frc2::CommandPtr force_score() {
        return frc2::cmd::Sequence(
            this->Run([this]() {
                scratch = state;
                activate(constants::ShooterStates::TrackForceShot);
            }).WithTimeout(0.5_s),
            this->RunOnce([this]() {
                activate(scratch);
            })
        );
    }

    frc2::CommandPtr sub_score() {
        return frc2::cmd::Sequence(
            this->Run([this]() {
                scratch = state;
                activate(constants::ShooterStates::SubScore);
            }).WithTimeout(0.5_s),
            this->RunOnce([this]() {
                activate(scratch);
            })
        );
    }

    frc2::CommandPtr creamy_shot() {
        return frc2::cmd::Sequence(
            this->RunOnce([this]() {
                scratch = state;
                activate(constants::ShooterStates::CreamyShot);
            }),
            frc2::cmd::Wait(1.5_s),
            this->RunOnce([this]() {
                activate(scratch);
            })
        );
    }

    frc2::CommandPtr reverse_feed() {
        return frc2::cmd::Sequence(
            this->Run([this]() {
                scratch = state;
                activate(constants::ShooterStates::ReverseFeed);
            }).Until([this]() {
                return !has_piece();
            }),
            this->RunOnce([this]() {
                activate(scratch);
            })
        );
    }

    /// @brief Runs the feedwheel until a piece is detected.
    /// @return frc2::CommandPtr representing a new command.
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

    /// @brief Puts the shooter into a idle state, without the flywheel moving
    /// @return frc2::CommandPtr representing a new command.
    frc2::CommandPtr stable() {
        return this->RunOnce([this]() {
            activate(constants::ShooterStates::Stopped);
        });
    }

    /// @brief Activates the shooter to put it into `TrackingIdle` mode
    /// @return frc2::CommandPtr representing a new command.
    frc2::CommandPtr track() {
        return this->RunOnce([this]() {
            activate(constants::ShooterStates::TrackingIdle);
        });
    }

    /// @brief Puts the shooter in a down state to allow the Amp Arm to move up
    /// and down.
    /// @return frc2::CommandPtr representing a new command.
    frc2::CommandPtr down() {
        return this->RunOnce([this]() {
            activate(constants::ShooterStates::Down);
        });
    }

    /// @brief Returns the active state of the shooter.
    /// @return See brief :)
    constants::ShooterStates get_state() const { return state; }

private:
    constants::ShooterStates scratch = constants::ShooterStates::Stopped;
    constants::ShooterStates state = constants::ShooterStates::Stopped;
    units::degree_t pivot_angle;
    units::degree_t turret_angle;
    units::foot_t hypot;
    subsystems::turret::Turret turret;
    subsystems::flywheel::Flywheel flywheel;
    subsystems::pivot::Pivot pivot;
};

}

}
