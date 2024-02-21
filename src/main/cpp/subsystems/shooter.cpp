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

void subsystems::shooter::Shooter::update_nt() {
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

void subsystems::shooter::Shooter::tick(frc::Pose2d robot_pose) {
    switch (state) {
        case constants::ShooterStates::Stopped: {
            flywheel.set_exit_vel(0_fps);
            pivot.set_angle(0_deg);
            turret.set_angle(0_deg);
        }
        break;
        case constants::ShooterStates::PrepFeeding: {
            pivot.set_angle(constants::PIVOT_IDLE);
            turret.set_angle(constants::TURRET_IDLE);
            flywheel.feed(false);
        }
        break;
        case constants::ShooterStates::StableIdle: {
            flywheel.set_exit_vel(constants::IDLE_VELOCITY);
            flywheel.stop_feed();
            pivot.set_angle(constants::PIVOT_IDLE);
            turret.set_angle(constants::TURRET_IDLE);
        }
        break;
        case constants::ShooterStates::TrackShot: {
            units::foot_t x = robot_pose.Translation().X();
            units::foot_t y = robot_pose.Translation().Y();
            units::degree_t turret_angle = robot_pose.Rotation().Degrees() * (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed ? 1 : -1);
            turret_angle += units::math::atan2(y, x);

            units::foot_t hypotenuse = units::math::sqrt(y * y + x * x);

            units::degree_t pivot_angle = units::math::atan2(constants::DELTA_Y, hypotenuse);

            pivot.set_angle(pivot_angle);
            turret.set_angle(turret_angle);

            if (pivot.at_angle() && turret.at_goal_point()) {
                flywheel.set_exit_vel(constants::SHOT_VELOCITY);
                if (flywheel.at_speed()) {
                    flywheel.feed(true);
                }
            }
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