#include "subsystems/drive.h"
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DataLogManager.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>

using namespace pathplanner;

subsystems::drive::Drivetrain::Drivetrain(std::shared_ptr<frc::XboxController> joystick) {
    this->joystick = joystick;

    frc::SmartDashboard::PutData("field", &field_drawing);

    PathPlannerLogging::setLogActivePathCallback([this](std::vector<frc::Pose2d> path) {
        field_drawing.GetObject("path")->SetPoses(path);
    });

    AutoBuilder::configureHolonomic(
        [this]() { return get_pose(); },
        [this](frc::Pose2d pose) { set_pose(pose); },
        [this]() { return get_robo_speeds(); },
        [this](frc::ChassisSpeeds speeds) { drive_robo(speeds); },
        HolonomicPathFollowerConfig(
            PIDConstants(5.5, 0.0, 0.0),
            PIDConstants(1.76, 0.0, 0.0),
            constants::MAX_AUTO_SPEED,
            16_in,
            ReplanningConfig(false, false)
        ),
        []() {
            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this
    );

    pose_estimator.SetVisionMeasurementStdDevs({2.7, 2.7, 2.7});
}

void subsystems::drive::Drivetrain::tick(bool is_field_oriented) {
    if (joystick->GetStartButton()) {
        reset_odometry();
    }

    const double fast_mode_mul = joystick->GetLeftTriggerAxis() > 0.60 ? joystick->GetLeftTriggerAxis() : 0.60;
    double slow_mode_mul = joystick->GetRightTriggerAxis() > 0.60 ? 1.0 - joystick->GetRightTriggerAxis() : 1.0;
    slow_mode_mul = slow_mode_mul < 0.1 ? 0.1 : slow_mode_mul;
    const double mode_mul = slow_mode_mul != 1.0 ? slow_mode_mul : fast_mode_mul;

    frc::SmartDashboard::PutNumber("Drivetrain_fast_mul", fast_mode_mul);

    double x_axis = -joystick->GetLeftX();
    double y_axis = -joystick->GetLeftY();

    x_axis = fabs(x_axis) > 0.1 ? x_axis : 0.0;
    y_axis = fabs(y_axis) > 0.1 ? y_axis : 0.0;

    x_axis *= frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed ? -1 : 1;
    y_axis *= frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed ? -1 : 1;

    double r_axis = -joystick->GetRightX();
    r_axis = fabs(r_axis) > 0.1 ? r_axis : 0.0;

    // Swapped bc forward is Vx but x_axis is side to side
    units::feet_per_second_t x_velocity = constants::MAX_SPEED * y_axis * mode_mul;
    units::feet_per_second_t y_velocity = constants::MAX_SPEED * x_axis * mode_mul;

    units::degrees_per_second_t r_velocity = constants::MAX_ROT_SPEED * r_axis * mode_mul;

    frc::SmartDashboard::PutNumber("Drivetrain_y_vel", y_velocity.value());
    frc::SmartDashboard::PutNumber("Drivetrain_x_vel", x_velocity.value());

    frc::ChassisSpeeds chassis_speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
        x_velocity,
        y_velocity,
        r_velocity,
        get_pose().Rotation()
    );

    if (!is_field_oriented) {
        chassis_speeds.omega = r_velocity;
        chassis_speeds.vx = x_velocity;
        chassis_speeds.vy = y_velocity;
    }

    if (joystick->GetYButton()) {
        chassis_speeds.omega = units::radians_per_second_t{heading_controller.Calculate(
            frc::AngleModulus(get_pose().Rotation().Radians()).value(),
            units::radian_t{
                -90_deg
            }.value()
        )};
    }

    if (joystick->GetBButton()) {
        chassis_speeds.omega = units::radians_per_second_t{heading_controller.Calculate(
            frc::AngleModulus(get_pose().Rotation().Radians()).value(),
            units::radian_t{
                frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed ? -135_deg : -45_deg
            }.value()
        )};
    }

    if (!joystick->GetXButton()) {
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
    } else {
        frc::SwerveModuleState left = {0_fps, frc::Rotation2d{45_deg}};
        frc::SwerveModuleState right = {0_fps, frc::Rotation2d{135_deg}};

        front_left.set_desired_goal(left, true);
        front_right.set_desired_goal(right, true);
        back_left.set_desired_goal(right, true);
        back_right.set_desired_goal(left, true);
    }

    if (units::math::fabs(front_left.get_velocity()) > max_detected_velocity) {
        max_detected_velocity = units::math::fabs(front_left.get_velocity());
    }

    frc::SmartDashboard::PutNumber("Drivetrain_max_speed", max_detected_velocity.value());

    swerve_tick();
}

void subsystems::drive::Drivetrain::swerve_tick() {
    front_left.tick();
    front_right.tick();
    back_left.tick();
    back_right.tick();
}

void subsystems::drive::Drivetrain::reset_odometry() {
    set_pose(frc::Pose2d {});
}

void subsystems::drive::Drivetrain::update_odometry() {
    frc::Pose2d last_pose = get_pose();
    frc::Pose2d newpose = pose_estimator.Update(
        gyro->GetRotation2d(),
        {
            front_left.get_position(), front_right.get_position(),
            back_left.get_position(), back_right.get_position()
        }
    );

    if (units::math::abs(newpose.X() - last_pose.X()) > 10_m || units::math::abs(newpose.Y() - last_pose.Y()) > 10_m) {
        set_pose(last_pose);
    }

    auto vision_est = photon_estimator.Update();

    if (vision_est) {
        double uncertainty = 0.7;
        int num_targets = 0;
        for (const auto &v : vision_est->targetsUsed) {
            uncertainty *= 1 / (v.GetArea() / 10);
            num_targets++;
        }
        uncertainty = uncertainty / num_targets;
        pose_estimator.AddVisionMeasurement(
            vision_est.value().estimatedPose.ToPose2d(),
            frc::Timer::GetFPGATimestamp(),
            {uncertainty, uncertainty, uncertainty}
        );
    }

    vision_est = photon_estimator_front.Update();

    if (vision_est) {
        double uncertainty = 0.7;
        for (const auto &v : vision_est->targetsUsed) {
            uncertainty *= 1 / (v.GetArea() / 10);
        }
        pose_estimator.AddVisionMeasurement(
            vision_est.value().estimatedPose.ToPose2d(),
            frc::Timer::GetFPGATimestamp(),
            {uncertainty, uncertainty, uncertainty}
        );
    }
}

void subsystems::drive::Drivetrain::reset_pose_to_vision() {
    auto vision_est = photon_estimator.Update();

    if (vision_est) {
        set_pose(vision_est.value().estimatedPose.ToPose2d());
    }
}

void subsystems::drive::Drivetrain::run_sysid(int test_num) {
    if (!sysid_command) {
        switch (test_num) {
        case 0: {
            sysid_command = sysid.Quasistatic(frc2::sysid::Direction::kForward);
            sysid_command->Schedule();
            break;
        }
        case 1: {
            sysid_command = sysid.Quasistatic(frc2::sysid::Direction::kReverse);
            sysid_command->Schedule();
            break;
        }
        case 2: {
            sysid_command = sysid.Dynamic(frc2::sysid::Direction::kForward);
            sysid_command->Schedule();
            break;
        }
        case 3: {
            sysid_command = sysid.Dynamic(frc2::sysid::Direction::kReverse);
            sysid_command->Schedule();
            break;
        }
        }
    }
}

frc2::CommandPtr subsystems::drive::Drivetrain::get_auto_path(std::string path_name) {
    current_traj = path_name;

    return AutoBuilder::buildAuto(path_name);
}

void subsystems::drive::Drivetrain::cancel_sysid() {
    if (sysid_command) {
        sysid_command->Cancel();

        front_left.set_desired_goal(frc::SwerveModuleState { 0_mps, frc::Rotation2d()}, true);
        front_right.set_desired_goal(frc::SwerveModuleState { 0_mps, frc::Rotation2d() }, true);
        back_left.set_desired_goal(frc::SwerveModuleState { 0_mps, frc::Rotation2d() }, true);
        back_right.set_desired_goal(frc::SwerveModuleState { 0_mps, frc::Rotation2d() }, true);
    }

    sysid_command = std::nullopt;
}

frc::Pose2d subsystems::drive::Drivetrain::get_pose() const {
    return pose_estimator.GetEstimatedPosition();
}

void subsystems::drive::Drivetrain::set_pose(const frc::Pose2d pose) {
    pose_estimator.ResetPosition(
        gyro->GetRotation2d(),
        {
            front_left.get_position(), front_right.get_position(),
            back_left.get_position(), back_right.get_position()
        },
        pose
    );
}

frc::ChassisSpeeds subsystems::drive::Drivetrain::get_robo_speeds() const {
    return kinematics.ToChassisSpeeds({
        front_left.get_state(), front_right.get_state(),
        back_left.get_state(), back_right.get_state()
    });
}

void subsystems::drive::Drivetrain::drive_robo(frc::ChassisSpeeds chassis_speeds) {
    wpi::array<frc::SwerveModuleState, 4> states = kinematics.ToSwerveModuleStates(chassis_speeds);
    kinematics.DesaturateWheelSpeeds(
        &states,
        chassis_speeds,
        swerve::constants::kMAX_WHEEL_SPEED,
        constants::MAX_AUTO_SPEED,
        constants::MAX_ROT_SPEED
    );

    auto [fl, fr, bl, br] = states;

    front_left.set_desired_goal(fl);
    front_right.set_desired_goal(fr);
    back_left.set_desired_goal(bl);
    back_right.set_desired_goal(br);
}

void subsystems::drive::Drivetrain::update_nt() {
    frc::SmartDashboard::PutNumber("Drivetrain_heading", get_pose().Rotation().Degrees().value());
    frc::SmartDashboard::PutNumber("Drivetrain_xpos_ft", units::foot_t{get_pose().Translation().X()}.value());
    frc::SmartDashboard::PutNumber("Drivetrain_ypos_ft", units::foot_t{get_pose().Translation().Y()}.value());

    frc::SmartDashboard::PutNumber("Drivetrain_xpos_m", get_pose().Translation().X().value());
    frc::SmartDashboard::PutNumber("Drivetrain_ypos_m", get_pose().Translation().Y().value());


    field_drawing.SetRobotPose(get_pose());

    frc::SmartDashboard::PutNumber("Drivetrain_fl_current", front_left.get_drive_current());

    frc::SmartDashboard::PutNumber("Drivetrain_fl_heading", units::degree_t{front_left.get_heading()}.value());
    frc::SmartDashboard::PutNumber("Drivetrain_speed_fps", units::feet_per_second_t{front_left.get_velocity()}.value());
    frc::SmartDashboard::PutNumber("Drivetrain_br_heading", units::degree_t{back_right.get_heading()}.value());
}
