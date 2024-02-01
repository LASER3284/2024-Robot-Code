#include "subsystems/shooter.h"

using namespace subsystems::shooter;

Shooter::Shooter(
    std::shared_ptr<turret::turret>turret_control,
    std::shared_ptr<flywheel::flywheel>flywheel_control,
    std::shared_ptr<pivot::pivot>pivot_control
){
    this->flywheel_control = flywheel_control;
    this->turret_control = turret_control;
    this->pivot_control = pivot_control;
}
void Shooter::tick(frc::Pose2d pose, units::feet_per_second_t robot_velocity){
    frc::Translation2d robot_position = pose.Translation();
    frc::Rotation2d robot_heading = pose.Rotation();

}

/// FlyWheel methods/////


