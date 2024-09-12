#include "subsystems/shooter.h"
#include "subsystems/pivot.h"
#include "subsystems/turret.h"
#include <frc/DriverStation.h>
#include <frc/MathUtil.h>
#include <units/angle.h>
#include <units/time.h>
#include <frc/smartdashboard/SmartDashboard.h>

void subsystems::shooter::Shooter::init() {
    turret.init();
    flywheel.init();
    pivot.init();

    turret.set_angle(constants::TURRET_IDLE);
    pivot.set_angle(constants::PIVOT_IDLE);
}

void subsystems::shooter::Shooter::update_nt(frc::Pose2d robot_pose) {
    units::foot_t x = robot_pose.Translation().X() - 3_in - (frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue) == frc::DriverStation::Alliance::kRed ? constants::GOAL_RED_POSITION.X() : constants::GOAL_BLUE_POSITION.X());
    units::foot_t y = robot_pose.Translation().Y() - 3_in - (frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue) == frc::DriverStation::Alliance::kRed ? constants::GOAL_RED_POSITION.Y() : constants::GOAL_BLUE_POSITION.Y());
    turret_angle = -robot_pose.Rotation().Degrees();

    // This should be dead code, but it's being left bc it works anyway.
    // if (frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue) == frc::DriverStation::Alliance::kRed) {
    //     turret_angle = turret_angle;
    //     frc::SmartDashboard::PutNumber("REACHED THE THING", 12);
    // }

    hypot = units::math::sqrt(y * y + x * x);
    turret_angle += units::math::atan2(y, x) + constants::TURRET_CORRECTION * hypot;

    turret_angle = frc::AngleModulus(turret_angle);

    pivot_angle = units::math::atan2(constants::DELTA_Y, hypot);
    if (hypot > 5_ft) {
        pivot_angle += units::foot_t(pow(hypot.value(), 1.1)) * constants::PIVOT_CORRECTION;
    }

    pivot_angle = frc::AngleModulus(pivot_angle);

    frc::SmartDashboard::PutNumber("shooter_pivotangle_goal", pivot_angle.value() - 2);
    frc::SmartDashboard::PutNumber("shooter_turretangle_goal", turret_angle.value() - 4);
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
            //idle position
            flywheel.set_exit_vel(0_fps);
            pivot.set_angle(constants::PIVOT_IDLE);
            flywheel.stop_feed();
            if (pivot_ok())
                turret.set_angle(constants::TURRET_IDLE);
        }
        break;
        case constants::ShooterStates::Down: {
            //amp arm scoring shooter position
            flywheel.set_exit_vel(0_fps);
            pivot.set_angle(constants::PIVOT_DOWN);
            flywheel.stop_feed();
            turret.set_angle(constants::TURRET_DOWN);
        }
        break;
        case constants::ShooterStates::ReverseFeed: {
            //feed back into amp arm
            flywheel.reverse_feed();
        }
        break;
        
        case constants::ShooterStates::PrepFeeding: {
            flywheel.set_exit_vel(0_fps);
            pivot.set_angle(constants::PIVOT_FEED);
            if (pivot_ok())
                turret.set_angle(constants::TURRET_FEED);

            if (!has_piece()) {
                flywheel.feed(false);
            } else {
                flywheel.stop_feed();
            }
        }
        break;
        case constants::ShooterStates::StableIdle: {
            flywheel.set_exit_vel(constants::IDLE_VELOCITY);
            flywheel.stop_feed();
            pivot.set_angle(constants::PIVOT_IDLE);
            if (pivot_ok())
                turret.set_angle(constants::TURRET_IDLE);
        }
        break;

        //useless
        case constants::ShooterStates::TrackShot: {
            pivot.set_angle(pivot_angle);
            if (pivot_ok())
            //turret 4 degree offset because shooter doesn't shoot straight
                turret.set_angle(turret_angle - 4_deg);

            //flywheel.set_exit_vel(constants::SHOT_VELOCITY);

            if (in_place() && has_piece()) {
                flywheel.feed(true);
            }
        }
        break;
        case constants::ShooterStates::TrackForceShot: {
            pivot.set_angle(pivot_angle);
            if (pivot_ok())
                turret.set_angle(turret_angle - 4_deg);

            flywheel.set_exit_vel(constants::SHOT_VELOCITY);

            if (has_piece()) {
                flywheel.feed(true);
            }
        }
        break;
        case constants::ShooterStates::TrackingIdle: { 
            //hi its emma i put this in to see if we shoot faster \/
            // flywheel.prespin();
            pivot.set_angle(pivot_angle);
            if (pivot_ok())
            // pivot okay means withing soft stop zone not at aimed goal
                turret.set_angle(turret_angle - 4_deg);

            flywheel.set_exit_vel(constants::SHOT_VELOCITY);
        }
        break;
        case constants::ShooterStates::SubScore: {
            // sub shot, may not be necessary anymore with the changes to the soft stop and the shock
            pivot.set_angle(constants::SUB_PIVOT_ANGLE);
            turret.set_angle(constants::SUB_TURRET_ANGLE);
            flywheel.set_exit_vel(constants::SHOT_VELOCITY);
        }
        break; 
        case constants::ShooterStates::CreamyShot: {
            // shot from the source to ferry notes over near the speaker: UNTESTED
            pivot.set_angle(constants::CREAMY_PIVOT_ANGLE);
            if (pivot_ok())
                turret.set_angle(turret_angle - 15_deg * (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed ? -1 : 1));
            flywheel.set_exit_vel(hypot * constants::FLYWHEEL_CORRECTION_CREAMY);

            if (in_place() && has_piece()) {
                flywheel.feed(true);
            }
        }
        break;
        case constants::ShooterStates::Spit: {
            pivot.set_angle(constants::PIVOT_FEED);
            turret.set_angle(constants::TURRET_SPIT);
            flywheel.set_exit_vel(constants::IDLE_VELOCITY);

            if (in_place() && has_piece()) {
                flywheel.feed(true);
            }
        }
        break;
        case constants::ShooterStates::Override: { 
            pivot.set_angle(or_pivot_angle);
            turret.set_angle(or_turret_angle);
            flywheel.set_exit_vel(or_flywheel_speed);

            if (or_fire) {
                flywheel.feed(true);
            }
        }
        break;
        case constants::ShooterStates::TestAngle: {
            //this was for testing if the pivot and turret would go to set angles, specifically for PID
            //should not be assigned to a controller unless we are testing
            pivot.set_angle(constants::TEST_PIVOT_ANGLE);
            turret.set_angle(constants::TEST_TURRET_ANGLE);
            // flywheel.set_exit_vel(constants::IDLE_VELOCITY);
        }
        case constants::ShooterStates::Prespin: {
            flywheel.set_exit_vel(constants::SHOT_VELOCITY);
        }
        break;
        default: {
            state = constants::ShooterStates::Stopped;
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
