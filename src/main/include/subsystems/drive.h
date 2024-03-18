#pragma once

#include <units/length.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <memory>
#include <string>
#include <frc/XboxController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include "swerve.h"

// Vision stuff
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

// SysID stuff
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/smartdashboard/SmartDashboard.h>

// Pathplanner stuff
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/Field2d.h>

namespace subsystems {

namespace drive {

namespace constants {

    // Locations of the swerve modules
    const frc::Translation2d FRONT_LEFT_LOCATION {11.25_in, 11.25_in};
    const frc::Translation2d FRONT_RIGHT_LOCATION {11.25_in, -11.25_in};
    const frc::Translation2d BACK_LEFT_LOCATION {-11.25_in, 11.25_in};
    const frc::Translation2d BACK_RIGHT_LOCATION {-11.25_in, -11.25_in};

    // Drivetrain parameters based on physical constraints (or practical ones,
    // optionally)

    /// @brief The maximum allowed speed of the drivetrain, used for calculating
    /// what speed to give to the swerve modules based on joystick input.
    constexpr units::feet_per_second_t MAX_SPEED = 18_fps;

    /// @brief The maximum allowed speed of the drivetrain in auto.
    constexpr units::feet_per_second_t MAX_AUTO_SPEED = 1_fps;

    /// @brief The maxiumum allowed rotational speed of the drivetrain, used for
    /// calculating what speed to give to the swerve modules based on the
    /// joystick input.
    constexpr units::degrees_per_second_t MAX_ROT_SPEED = 720_deg_per_s;
} // namespace constants

/// @brief The drive train wrapping class that implements the pathplanning and
/// teleop joystick tracking.
class Drivetrain : public frc2::SubsystemBase {
public:
    /// @brief Creates the Drivetrain object with the provided joystick
    /// reference.
    /// @param joystick The shared_ptr reference to frc::XboxController
    Drivetrain(std::shared_ptr<frc::XboxController>);

    /// @brief Runs the main loop for the drivetrain. This is what gets the
    /// motors in the modules to move according to joysticks.
    /// @param is_field_oriented Whether or not to use field-oriented drive.
    void tick(bool);

    /// @brief Updates the swerve modules to do the correct movements on every
    /// periodic. Called by tick, but should be called in AutonomousPeriodic
    /// where tick is not allowed.
    void swerve_tick();

    /// @brief Updates the Network Tables data.
    void update_nt();

    /// @brief Schedules a SysId command if one is not already scheduled. Does
    /// nothing otherwise.
    /// @param test_num The number of the test to run. 0 is
    /// Quasi-fwd, 1 is Quasi-rev, etc.
    /// @see Robot::SysIdChooser
    void run_sysid(int);

    /// @brief Runs with update_nt in RobotPeriodic to update values of the
    /// robot pose.
    /// @see update_nt
    void update_odometry();

    /// @brief Checkes for a estimation from the back camera and, if one is
    /// found, will reset the pose of the drive train to the esitmated pose.
    void reset_pose_to_vision();

    /// @brief Returns the translational and rotational components of the robot
    /// based on known odometry. Used by PathPlanner.
    /// @return See brief :)
    frc::Pose2d get_pose() const;

    /// @brief Sets the odometry to use the new translational and rotational
    /// components of robot position. Used by PathPlanner.
    /// @param pose A constant Pose2d that the odometry should use.
    void set_pose(const frc::Pose2d);

    /// @brief Returns the chassis-relative speeds of the robot. Used by
    /// PathPlanner.
    /// @return See brief :)
    frc::ChassisSpeeds get_robo_speeds() const;

    /// @brief Attempts to force the swerve into a speed for the robot to move
    /// at. Used by PathPlanner.
    /// @param chassis_speeds Chassis-relative speeds
    /// to force onto the swerve modules.
    void drive_robo(const frc::ChassisSpeeds);

    /// @brief Resets the odometry (pose, etc) to be 0 on all values.
    void reset_odometry();

    /// @brief Cancels the scheduled SysId command, if one is scheduled; does
    /// nothing otherwise.
    void cancel_sysid();

    /// @brief Fetches the command for the path based on the AutoBuilder and
    /// path name.
    /// @param path_name The 'human' name of the path (as seen in the
    /// PathPlanner GUI).
    /// @return The generated CommandPtr for the path, which will call the
    /// associated NamedCommands if they've been registered.
    frc2::CommandPtr get_auto_path(std::string);
private:
    std::shared_ptr<frc::XboxController> joystick;

    frc::Field2d field_drawing;

    std::string current_traj;

    frc::PIDController heading_controller {
        2.0,
        0,
        0
    };

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

    photon::PhotonPoseEstimator photon_estimator {
        frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo),
        photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
        std::move(photon::PhotonCamera{"mainCam"}),
        frc::Transform3d {
            frc::Translation3d{-14.75_in, -6_in, 8_in},
            frc::Rotation3d{0_deg, 30_deg, 180_deg}
        }
    };

    photon::PhotonPoseEstimator photon_estimator_front {
        frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo),
        photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
        std::move(photon::PhotonCamera{"frontCam"}),
        frc::Transform3d {
            frc::Translation3d{14.5_in, -2.75_in, 7.75_in},
            frc::Rotation3d{0_deg, 30_deg, 0_deg}
        }
    };\

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
            },
            this
        }
    };

    std::optional<frc2::CommandPtr> sysid_command;

}; // class drivetrain

} // namespace drive

} // namespace subsystems
