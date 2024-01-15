#pragma once

#include <units/length.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <memory>
#include <frc/XboxController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include "swerve.h"

// Vision stuff
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonPoseEstimator.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

// SysID stuff
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc2/command/SubsystemBase.h>

namespace subsystems {

namespace drive {

namespace constants {
    const frc::Translation2d FRONT_LEFT_LOCATION {11.25_in, 11.25_in};
    const frc::Translation2d FRONT_RIGHT_LOCATION {11.25_in, -11.25_in};
    const frc::Translation2d BACK_LEFT_LOCATION {-11.25_in, 11.25_in};
    const frc::Translation2d BACK_RIGHT_LOCATION {-11.25_in, -11.25_in};

    constexpr units::feet_per_second_t MAX_SPEED = 18_fps;
    constexpr units::degrees_per_second_t MAX_ROT_SPEED = 720_deg_per_s;
} // namespace constants

class Drivetrain : public frc2::SubsystemBase {
public:
    Drivetrain(std::shared_ptr<frc::XboxController> joystick);

    /// @brief Runs the main loop for the drivetrain. This is what gets the motors in the modules to move and updates odometry.
    /// @param is_field_oriented Whether or not to use field-oriented drive.
    void tick(bool);

    void run_sysid(int);

    /// @brief Runs in the tick in order to estimate the pose of the robot.
    void update_odometry();

    /// @brief Resets the odometry (pose, etc).
    void reset_odometry();

    void cancel_sysid();
private:
    std::shared_ptr<frc::XboxController> joystick;

    units::feet_per_second_t max_detected_velocity = 0_fps;

    std::unique_ptr<ctre::phoenix6::hardware::Pigeon2> gyro = std::make_unique<ctre::phoenix6::hardware::Pigeon2>(62);

    swerve::Module front_left{7, 8, 14};
    swerve::Module front_right{5, 6, 13};
    swerve::Module back_left{1, 2, 11};
    swerve::Module back_right{3, 4, 12};

    frc::SwerveDriveKinematics<4> kinematics {
        constants::FRONT_LEFT_LOCATION,
        constants::FRONT_RIGHT_LOCATION,
        constants::BACK_LEFT_LOCATION,
        constants::BACK_RIGHT_LOCATION
    };

    frc::SwerveDrivePoseEstimator<4> pose_estimator {
        kinematics,
        frc::Rotation2d {},
        {
            front_left.get_position(),
            front_right.get_position(),
            back_left.get_position(),
            back_right.get_position()
        },
        frc::Pose2d {}
    };

    photonlib::PhotonPoseEstimator photon_estimator {
        frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp),
        photonlib::LOWEST_AMBIGUITY,
        std::move(photonlib::PhotonCamera{"mainCam"}), frc::Transform3d{
            frc::Translation3d{14.75_in, 14.75_in, 5.875_in},
            frc::Rotation3d{0_deg, 67.5_deg, 0_deg}
        }
    };

    frc2::sysid::SysIdRoutine sysid {
        frc2::sysid::Config {std::nullopt, std::nullopt, std::nullopt, std::nullopt},
        frc2::sysid::Mechanism {
            [this](units::volt_t volts) {
                front_left.apply_heading_goal(0_deg);
                front_left.set_drive_power(volts);
                front_right.apply_heading_goal(0_deg);
                front_right.set_drive_power(volts);
                back_left.apply_heading_goal(0_deg);
                back_left.set_drive_power(volts);
                back_right.apply_heading_goal(0_deg);
                back_right.set_drive_power(volts);
            },
            [this](auto log) {
                log->Motor("front-left-drive")
                    .voltage(front_left.get_drive_power())
                    .velocity(units::meters_per_second_t{front_left.get_velocity()})
                    .position(front_left.get_position().distance);
                log->Motor("front-right-drive")
                    .voltage(front_right.get_drive_power())
                    .velocity(units::meters_per_second_t{front_right.get_velocity()})
                    .position(front_right.get_position().distance);
                log->Motor("back-left-drive")
                    .voltage(back_left.get_drive_power())
                    .velocity(units::meters_per_second_t{back_left.get_velocity()})
                    .position(back_left.get_position().distance);
                log->Motor("back-right-drive")
                    .voltage(back_right.get_drive_power())
                    .velocity(units::meters_per_second_t{back_right.get_velocity()})
                    .position(back_right.get_position().distance);
            },
            this
        }
    };

    bool not_scheduled = false;

    std::optional<frc2::CommandPtr> sysid_command;

}; // class drivetrain

} // namespace drive

} // namespace subsystems