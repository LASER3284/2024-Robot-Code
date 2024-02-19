#include "subsystems/shooter.h"
#include "subsystems/pivot.h"
#include "subsystems/turret.h"

#include <units/angle.h>
#include <units/time.h>

#include <frc/smartdashboard/SmartDashboard.h>

using namespace subsystems::shooter;

/*int constants::sgn(double x) {
    return x >= 0 ? 1 : -1;
} */

void Shooter::tick(frc::Pose2d pose){
    frc::Translation2d robot_position_rel = pose.Translation() - constants::GOAL_POSITION;
    frc::Rotation2d robot_heading = pose.Rotation();

    units::degree_t theta = pivot.get_angle();
    units::degree_t alpha = turret.get_angle();

    units::feet_per_second_t exit_vel = constants::IDLE_VELOCITY;

    if (state == constants::ShooterStates::TrackingIdle || state == constants::ShooterStates::TrackShot) {
        // This is all some goofy math for finding the pivot angle
        units::foot_t delta_x = units::math::sqrt(units::math::pow<2>(robot_position_rel.X()) + units::math::pow<2>(robot_position_rel.Y()));

        auto cos_numerator = constants::GRAVITY * (units::math::pow<2>(delta_x) / units::math::pow<2>(constants::DELTA_Y)) + constants::DELTA_Y;

        auto cos_denominator = units::math::sqrt(units::math::pow<2>(delta_x) + units::math::pow<2>(constants::DELTA_Y)) * constants::sgn(constants::DELTA_Y.value());

        auto term_one = units::math::acos(cos_numerator / cos_denominator);

        auto term_two = units::math::atan2(-constants::DELTA_Y, delta_x);

        // Finally have the pivot angle!
        theta = (term_one - term_two) / 2;

        // Turret angle
        alpha = units::math::atan2(robot_position_rel.Y(), robot_position_rel.X()) - robot_heading.Degrees();
    }

    if (state == constants::ShooterStates::TrackShot || state == constants::ShooterStates::StableShot) {
        exit_vel = constants::SHOT_VELOCITY;
    }

    pivot.set_angle(theta);
    turret.set_angle(alpha);
    flywheel.set_exit_vel(exit_vel);
}

void Shooter::update_nt() {


};

inline void subsystems::shooter::Shooter::activate()
{
    switch ( state )
        case constants::ShooterStates::TrackingIdle:

        case constants::ShooterStates::TrackShot:

        case constants::ShooterStates::StableIdle:

        case constants::ShooterStates::StableShot:

        case constants::ShooterStates::LowStableShot:

        case constants::ShooterStates::PrepFeeding:

        case constants::ShooterStates::Stopped:


};

// turret
void subsystems::turret::Turret::idle_turret() {}

void subsystems::turret::Turret::set_angle(units::degree_t) {

}
units::degree_t subsystems::turret::Turret::get_angle() {
    //units::degree_t turret_angle = turret_encoder.GetVelocity() 
}



void subsystems::turret::Turret::update_nt() {
    // Calculate dtheta/dt
    //units::second_t dt = frc::Timer::GetFPGATimestamp() - last_time;
    //units::degree_t dtheta = get_pose() - last_angle;
    //velocity = dtheta / dt;

    // Do other things
    frc::SmartDashboard::PutNumber("turret_pos", get_pose().value());
    frc::SmartDashboard::PutBoolean("turret_inplace", at_goal_point());

    // Set last_position and last_time
    //last_time = frc::Timer::GetFPGATimestamp();
    //last_angle = get_pose();

}
bool subsystems::turret::Turret::at_goal_point() {}
bool subsystems::turret::Turret::turret_power() {}

// pivot
void subsystems::pivot::Pivot::set_angle(units::degree_t) {};
void subsystems::pivot::Pivot::idle_angle() {}
bool subsystems::pivot::Pivot::at_angle() {}
