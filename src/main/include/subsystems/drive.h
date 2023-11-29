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

namespace subsystems {

namespace drive {

namespace constants {
    const frc::Translation2d FRONT_LEFT_LOCATION {11.25_in, 11.25_in};
    const frc::Translation2d FRONT_RIGHT_LOCATION {11.25_in, -11.25_in};
    const frc::Translation2d BACK_LEFT_LOCATION {-11.25_in, 11.25_in};
    const frc::Translation2d BACK_RIGHT_LOCATION {-11.25_in, -11.25_in};

    constexpr units::meters_per_second_t MAX_SPEED = 18_fps;
    constexpr units::degrees_per_second_t MAX_ROT_SPEED = 720_deg_per_s;
} // namespace constants

class Drivetrain {
public:
    Drivetrain(std::shared_ptr<frc::XboxController> joystick);

    /// @brief Runs the main loop for the drivetrain. This is what gets the motors in the modules to move and updates odometry.
    /// @param is_field_oriented Whether or not to use field-oriented drive.
    void tick(bool);

    /// @brief Runs in the tick in order to estimate the pose of the robot.
    void update_odometry();

    /// @brief Resets the odometry (pose, etc).
    void reset_odometry();
private:
    std::shared_ptr<frc::XboxController> joystick;

    std::unique_ptr<ctre::phoenix6::hardware::Pigeon2> gyro = std::make_unique<ctre::phoenix6::hardware::Pigeon2>(5);

    swerve::Module front_left{0, 0, 0};
    swerve::Module front_right{1, 1, 1};
    swerve::Module back_left{2, 2, 2};
    swerve::Module back_right{3, 3, 3};

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

}; // class drivetrain

} // namespace drive

} // namespace subsystems