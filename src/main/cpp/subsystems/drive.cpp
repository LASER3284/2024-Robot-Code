#include "subsystems/drive.h"

subsystems::drive::Drivetrain::Drivetrain(std::shared_ptr<frc::XboxController> joystick) {
    this->joystick = joystick;
}

void subsystems::drive::Drivetrain::tick(bool is_field_oriented) {
    update_odometry();

    double x_axis = joystick->GetLeftX();
    double y_axis = joystick->GetLeftY();

    double r_axis = joystick->GetRightX();

    units::meters_per_second_t x_velocity = constants::MAX_SPEED * x_axis;
    units::meters_per_second_t y_velocity = constants::MAX_SPEED * y_axis;

    units::degrees_per_second_t r_velocity = constants::MAX_ROT_SPEED * r_axis;

    frc::ChassisSpeeds chassis_speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        x_velocity,
        y_velocity,
        r_velocity,
        gyro->GetRotation2d()
    );

    if (!is_field_oriented) {
        chassis_speeds.omega = r_velocity;
        chassis_speeds.vx = x_velocity;
        chassis_speeds.vy = y_velocity;
    }

    wpi::array<frc::SwerveModuleState, 4> states = kinematics.ToSwerveModuleStates(chassis_speeds);
    kinematics.DesaturateWheelSpeeds(
        &states,
        chassis_speeds,
        swerve::constants::kMAX_WHEEL_SPEED,
        constants::MAX_SPEED,
        constants::MAX_ROT_SPEED
    );

    auto [fl, fr, bl, br] = states;

    front_left.set_desired_goal(fl);
    front_right.set_desired_goal(fr);
    back_left.set_desired_goal(bl);
    back_right.set_desired_goal(br);
}

void subsystems::drive::Drivetrain::reset_odometry() {
    front_left.reset_drive_position();
    front_right.reset_drive_position();
    back_left.reset_drive_position();
    back_right.reset_drive_position();

    gyro->SetYaw(0_deg);
}

void subsystems::drive::Drivetrain::update_odometry() {
    pose_estimator.Update(
        gyro->GetRotation2d(),
        {
            front_left.get_position(), front_right.get_position(),
            back_left.get_position(), back_right.get_position()
        }
    );
}