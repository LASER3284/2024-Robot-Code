#include "subsystems/drive.h"
#include <frc/smartdashboard/SmartDashboard.h>

subsystems::drive::Drivetrain::Drivetrain(std::shared_ptr<frc::XboxController> joystick) {
    this->joystick = joystick;
}

void subsystems::drive::Drivetrain::tick(bool is_field_oriented) {
    if (joystick->GetStartButton()) {
        reset_odometry();
    } else {
        update_odometry();
    }

    const double fast_mode_mul = joystick->GetLeftTriggerAxis() > 0.60 ? joystick->GetLeftTriggerAxis() : 0.60;
    double slow_mode_mul = joystick->GetRightTriggerAxis() > 0.60 ? 1.0 - joystick->GetRightTriggerAxis() : 1.0;
    slow_mode_mul = slow_mode_mul < 0.1 ? 0.1 : slow_mode_mul;
    const double mode_mul = slow_mode_mul != 1.0 ? slow_mode_mul : fast_mode_mul;

    frc::SmartDashboard::PutNumber("Drivetrain_fast_mul", fast_mode_mul);

    double x_axis = -joystick->GetLeftX();
    double y_axis = joystick->GetLeftY();

    x_axis = fabs(x_axis) > 0.1 ? x_axis : 0.0;
    y_axis = fabs(y_axis) > 0.1 ? y_axis : 0.0;

    double r_axis = -joystick->GetRightX();
    r_axis = fabs(r_axis) > 0.1 ? r_axis : 0.0;

    units::feet_per_second_t x_velocity = constants::MAX_SPEED * x_axis * mode_mul;
    units::feet_per_second_t y_velocity = constants::MAX_SPEED * y_axis * mode_mul;

    units::degrees_per_second_t r_velocity = constants::MAX_ROT_SPEED * r_axis * mode_mul;

    frc::SmartDashboard::PutNumber("Drivetrain_y_vel", y_velocity.value());
    frc::SmartDashboard::PutNumber("Drivetrain_x_vel", x_velocity.value());

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

    frc::SmartDashboard::PutNumber("Drivetrain_speed", units::feet_per_second_t{front_left.get_velocity()}.value());
    frc::SmartDashboard::PutNumber("Drivetrain_fl_heading", units::degree_t{front_left.get_heading()}.value());

    if (units::math::fabs(front_left.get_velocity()) > max_detected_velocity) {
        max_detected_velocity = units::math::fabs(front_left.get_velocity());
    }

    frc::SmartDashboard::PutNumber("Drivetrain_max_speed", max_detected_velocity.value());
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

void subsystems::drive::Drivetrain::run_sysid(int test_num) {
    if (sysid_command && !sysid_command->IsScheduled()) {
        sysid_command->Schedule();
    } else if (sysid_command && sysid_command->IsScheduled()) {
        // empty code :)
        // wait for something else to cancel the scheduled command
    } else {
        switch (test_num) {
        case 0: {
            sysid_command = sysid.Quasistatic(frc2::sysid::Direction::kForward);
            break;
        }
        case 1: {
            sysid_command = sysid.Quasistatic(frc2::sysid::Direction::kReverse);
            break;
        }
        case 2: {
            sysid_command = sysid.Dynamic(frc2::sysid::Direction::kForward);
            break;
        }
        case 3: {
            not_scheduled = true;
            sysid_command = sysid.Dynamic(frc2::sysid::Direction::kReverse);
            break;
        }
        default: {
            cancel_sysid();
            break;
        }
        }
    }
}

void subsystems::drive::Drivetrain::cancel_sysid() {
    if (sysid_command)
        sysid_command->Cancel();

    sysid_command = std::nullopt;
}