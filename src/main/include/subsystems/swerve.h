#pragma once

#include <units/voltage.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <numbers>

namespace subsystems {

namespace swerve {

namespace constants {
    constexpr double kDRIVE_RATIO = 6.12;

    constexpr units::inch_t kWHEEL_DIAM = 4_in;

    constexpr units::inch_t kWHEEL_CIRC = kWHEEL_DIAM * std::numbers::pi;

    constexpr units::feet_per_second_t kMAX_WHEEL_SPEED = 18.0_fps;
    constexpr units::feet_per_second_squared_t kMAX_ROBOT_ACCEL = 24_fps_sq;

    constexpr int kDRIVE_ENC_RES = 2048;
}

class Module {
public:
    /// @brief Swerve module constructor for the various encoder+motors
    /// @param drive The CAN ID of the drive motor (TalonFX)
    /// @param turn The CAN ID of the turn motor (TalonFX)
    /// @param enc The CAN ID of the CANCoder
    Module(const int, const int, const int);

    /// @brief Returns the heading of the swerve module, where straight ahead is
    /// forward
    /// @return The heading in degrees
    units::radian_t get_heading() const;

    units::feet_per_second_t get_velocity() const;

    /// @brief Sets the desired state of the module (velocity of the drive and
    /// position of the turn)
    /// @param ref_state The reference state to base on
    /// @param force_angle Whether or not to force the angle position
    void set_desired_goal(const frc::SwerveModuleState&, bool = false);

    /// @brief Resets the position of the drive motor to 0 encoder counts
    void reset_drive_position();

    /// @brief Returns the position of the swerve module.
    /// @return Swerve position based on drive encoder in the motor and the absolute encoder.
    /// @see get_state
    frc::SwerveModulePosition get_position() const;

    /// @brief Returns the state of the swerve module.
    /// @return Swerve state based on drive encoder in the motor and the
    /// absolute encoder.
    /// @see get_position
    frc::SwerveModuleState get_state() const;

    /// @brief Sets the raw voltage of the motor for drive.
    /// @warning Calling this method outside of SysId will fight
    /// `set_desired_goal` for control of the drive power.
    /// @param volts The voltage from -12V to 12V
    void set_drive_power(const units::volt_t);

    /// @brief Sets the heading angle to use for SysID of the drive motor.
    /// @warning This method will fight `set_desired_goal` for control of the
    /// heading.
    /// @param angle The heading angle as units::radian_t
    void apply_heading_goal(const units::radian_t);

    /// @brief Gets the drive motor voltage based on applied motor power and
    /// battery voltage.
    /// @return The voltage of the motor
    units::volt_t get_drive_power() const;
private:
    /// @brief Sets the raw voltage of the motor for azimuth. Private b/c
    /// azimuth angle is important for drive power.
    /// @param volts The voltage from -12V to 12V
    /// @see apply_heading_goal
    void _set_turn_power(const units::volt_t);

    frc::PIDController heading_controller {
        1.75,
        0.6,
        0.0
    };

    frc::PIDController drive_controller {
        0.03,
        0.0,
        0.0
    };

    frc::SimpleMotorFeedforward<units::feet> drive_ff {0.19355_V, 0.6182_V / 1_fps, 0.0187071_V / 1_fps_sq};

    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> drive_motor;
    std::unique_ptr<ctre::phoenix6::hardware::TalonFX> turn_motor;
    std::unique_ptr<ctre::phoenix6::hardware::CANcoder> encoder;

}; // class module

} // namespace swerve

} // namespace subsystems
