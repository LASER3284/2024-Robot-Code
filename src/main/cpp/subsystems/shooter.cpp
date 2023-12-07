#include "subsystems/shooter.h"

using namespace subsystems::shooter;

Shooter::Shooter(
    std::shared_ptr<turret::Turret> turret_control,
    std::shared_ptr<flywheel::Flywheel> flywheel_control,
    std::shared_ptr<hood::Hood> hood_control
) {
    this->flywheel_control = flywheel_control;
    this->turret_control = turret_control;
    this->hood_control = hood_control;
}

void Shooter::tick(frc::Pose2d pose, units::feet_per_second_t robot_velocity) {
    frc::Translation2d robot_position = pose.Translation();
    frc::Rotation2d velocity_heading = pose.Rotation();

    //// HOOD STUFF ////

    const units::feet_per_second_t exit_velocity = 0_fps;

    const units::foot_t x_final = robot_position.Distance(constants::GOAL_POSITION);

    const units::foot_t numerator = constants::GRAVITY * (units::math::pow<2>(x_final) / units::math::pow<2>(exit_velocity) + constants::GOAL_HEIGHT_DELTA);

    const units::foot_t denominator = units::math::sqrt(units::math::pow<2>(constants::GOAL_HEIGHT_DELTA) + units::math::pow<2>(x_final));

    const units::degree_t two_theta = units::math::acos(numerator / denominator) - units::math::atan2(-constants::GOAL_HEIGHT_DELTA, x_final);

    const units::degree_t theta = two_theta / 2;

    hood_control->set_goal(two_theta);

    //// TURRET STUFF ////

    const auto denominator2 = units::math::sqrt(units::math::pow<2>(exit_velocity) * units::math::pow<2>(units::math::cos(theta)) + robot_velocity * robot_velocity + 2 * exit_velocity * robot_velocity * units::math::cos(theta) * units::math::cos(velocity_heading));

    const units::degree_t alpha = units::math::asin(velocity_heading.Radians() * robot_velocity / denominator2);

    turret_control->set_goal(turret_control->get_position() + alpha);
}