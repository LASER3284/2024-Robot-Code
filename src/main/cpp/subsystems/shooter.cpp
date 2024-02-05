#include "subsystems/shooter.h"

using namespace subsystems::shooter;

int constants::sgn(double x) {
    return x >= 0 ? 1 : -1;
}

Shooter::Shooter(
    std::shared_ptr<turret::turret>turret_control,
    std::shared_ptr<flywheel::flywheel>flywheel_control,
    std::shared_ptr<pivot::pivot>pivot_control
) {
    this->flywheel_control = flywheel_control;
    this->turret_control = turret_control;
    this->pivot_control = pivot_control;
}

void Shooter::tick(frc::Pose2d pose){
    frc::Translation2d robot_position_rel = pose.Translation() - constants::GOAL_POSITION;
    frc::Rotation2d robot_heading = pose.Rotation();

    units::degree_t theta = pivot_control->get_angle();
    units::degree_t alpha = turret_control->get_angle();

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

    pivot_control->set_angle(theta);
    turret_control->set_angle(alpha);
    flywheel_control->set_exit_vel(exit_vel);
}