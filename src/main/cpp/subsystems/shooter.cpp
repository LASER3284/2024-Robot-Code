#include "subsystems/shooter.h"
#include "subsystems/pivot.h"
#include "subsystems/turret.h"
#include <frc/DriverStation.h>
#include <units/angle.h>
#include <units/time.h>
#include <frc/smartdashboard/SmartDashboard.h>

void subsystems::shooter::Shooter::init() {
    turret.init();
    flywheel.init();
    pivot.init();
}

void subsystems::shooter::Shooter::update_nt(frc::Pose2d robot_pose) {
    units::foot_t x = robot_pose.Translation().X() - (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed ? constants::GOAL_RED_POSITION.X() : constants::GOAL_BLUE_POSITION.X());
    units::foot_t y = robot_pose.Translation().Y() - (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed ? constants::GOAL_RED_POSITION.Y() : constants::GOAL_BLUE_POSITION.Y());
    turret_angle = robot_pose.Rotation().Degrees() * (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed ? 1 : -1) - (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed ? 180_deg : 0_deg);
    turret_angle += units::math::atan2(y, x);

    units::foot_t hypotenuse = units::math::sqrt(y * y + x * x);

    pivot_angle = units::math::atan2(constants::DELTA_Y, hypotenuse);

    frc::SmartDashboard::PutNumber("shooter_pivotangle_goal", pivot_angle.value());
    frc::SmartDashboard::PutNumber("shooter_turretangle_goal", turret_angle.value());
    frc::SmartDashboard::PutBoolean("shooter_haspiece", has_piece());
    frc::SmartDashboard::PutBoolean("shooter_pivot_inplace", pivot.at_angle());
    frc::SmartDashboard::PutBoolean("shooter_turret_inplace", turret.at_goal_point());
    frc::SmartDashboard::PutBoolean("shooter_flywheel_inplace", flywheel.at_speed());

    turret.update_nt();
    flywheel.update_nt();
    pivot.update_nt();
}

void subsystems::shooter::Shooter::run_sysid(int test_num, subsystems::shooter::constants::SubMech mech) {
    switch (mech) {
        case subsystems::shooter::constants::SubMech::Flywheel:
            flywheel.run_sysid(test_num);
            break;
        case subsystems::shooter::constants::SubMech::Pivot:
            pivot.run_sysid(test_num);
            break;
        case subsystems::shooter::constants::SubMech::Turret:
            turret.run_sysid(test_num);
            break;
        default:
            break;
    }
}

void subsystems::shooter::Shooter::tick() {
    switch (state) {
        case constants::ShooterStates::Stopped: {
            flywheel.set_exit_vel(0_fps);
            pivot.set_angle(constants::PIVOT_IDLE);
            if (pivot.at_angle())
                turret.set_angle(constants::TURRET_IDLE);
        }
        break;
        case constants::ShooterStates::PrepFeeding: {
            pivot.set_angle(constants::PIVOT_FEED);
            if (pivot.at_angle())
                turret.set_angle(constants::TURRET_FEED);
            flywheel.feed(false);
        }
        break;
        case constants::ShooterStates::StableIdle: {
            flywheel.set_exit_vel(constants::IDLE_VELOCITY);
            flywheel.stop_feed();
            pivot.set_angle(constants::PIVOT_IDLE);
            if (pivot.at_angle())
                turret.set_angle(constants::TURRET_IDLE);
        }
        break;
        case constants::ShooterStates::TrackShot: {
            pivot.set_angle(pivot_angle);
            if (pivot.at_angle())
                turret.set_angle(turret_angle);

            flywheel.set_exit_vel(constants::SHOT_VELOCITY);

            if (flywheel.at_speed() && has_piece()) {
                flywheel.feed(true);
            }
        }
        break;
        case constants::ShooterStates::TrackingIdle: {
            pivot.set_angle(pivot_angle);
            if (pivot.at_angle())
                turret.set_angle(turret_angle);

            flywheel.set_exit_vel(constants::SHOT_VELOCITY);
        }
        break;
        default: {
            state = constants::StableIdle;
        }
        break;
    }

    flywheel.tick();
    pivot.tick();
    turret.tick();
}

void subsystems::shooter::Shooter::activate(constants::ShooterStates state) {
    this->state = state;
}