#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/time.h>
#include <rev/CANSparkMax.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/Timer.h>
#include <frc/DutyCycleEncoder.h>

// SysID
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc2/command/SubsystemBase.h>

namespace subsystems {

namespace ballast {

namespace constants {
    /// @brief The CAN ID for the ballast motor
    constexpr int BALLAST_ID = 0;

    /// @brief The gear ratio for the ballast motor to the output.
    constexpr double BALLAST_GEAR_RATIO = 90.0 * (0.0 / 0.0);

    /// @brief The setpoint for the ballast motor, measured in volts.
    constexpr units::volt_t BALLAST_SETPOINT = 0_V;

    /// @brief The proportional gain for the PID controller.

    /// @brief The statically applied voltage for the feedforward.
    constexpr auto BALLAST_KS = 0.0_V;
    /// @brief The gravitationally applied voltage for the feedforward.
    constexpr auto BALLAST_KG = 0.0_V;
    /// @brief The applied voltage per unit velocity.
    constexpr auto BALLAST_KV = 0.0_V / 1_deg_per_s;

    /// @brief The starting angle for the ballast.
    constexpr units::degree_t STARTING_ANGLE = 0_deg;

    /// @brief The angle offset for the absolute encoder.
    constexpr units::degree_t ANGLE_OFFSET = 0_deg;

} // namespace constants

class Ballast : public frc2::SubsystemBase {
public: 
    Ballast();

    /// @brief Calculates the percent output to set the motor to based on the angle of the ballast.
    void tick(units::degree_t shoulder_rotation);

    /// @brief Gives the current rotation of the output shaft.
    /// @return Rotation of output shaft, 0 degrees is parallel to the bottom of the robot frame.
    units::degree_t get_rotation();
            
    /// @brief Gives the most recent commanded setpoint of the output shaft.
    /// @return The most recently commanded angle of the ballast.
    units::degree_t get_last_setpoint() { return last_setpoint; }

    /// @brief Sets the rotation goal for the feedforward.
    /// Rotation should be 0 degrees parallel to the bottom of the frame in CCW+ orientation.
    /// @param rot The rotation goal to rotate the wrist to.
    void set_rotation_goal(units::degree_t rot);

    /// @brief Manually control the rotation of the ballast.
    void manual_control(double percentage) { manual_percentage = percentage; }

    void reset_rotation() {
    set_rotation_goal(get_rotation());
    ballast_timer.Restart();
    }

    /// @brief Cancels the SysId command if it has not been done already.
    void cancel_sysid();

    /// @brief Schedules a SysId command if it has not been done already, does nothing otherwise.
    /// @param test_num The test number to run (0 for Quasi-fwd, 1 for Quasi-rev, 2 for Dynam-fwd, 3 for Dynam-rev).
    /// @see Robot::SysIdChooser
    void run_sysid(int);
private:
    /// @brief The motor controller for the NEO object.
    rev::CANSparkMax ballast_motor { constants::BALLAST_ID, rev::CANSparkMaxLowLevel::MotorType::kBrushless };
    rev::SparkRelativeEncoder ballast_encoder = ballast_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

    frc::DutyCycleEncoder thrubore_enc { 0 };

    frc::PIDController controller {
        0.0,    // KP
        0.0,    // KI
        0.0    // KD
    };

    double manual_percentage = 0.0;

    /// @brief The trapezoidal profile constraints for the shoulder rotation
    /// This specifies the max rotational velocity *and* the max rotational acceleration.
    frc::TrapezoidProfile<units::radians>::Constraints rotational_constraints { 0_deg_per_s, 0_deg_per_s_sq };

    /// @brief The current goal to rotate the ballast to.
    frc::TrapezoidProfile<units::radians>::State ballast_goal;

    /// @brief The current setpoint for the ballast.
    frc::TrapezoidProfile<units::radians>::State ballast_setpoint;

    /// @brief A timer used for overriding the manual percentage vs the feedforward calculations.
    frc::Timer ballast_timer;
            
    /// @brief The last commanded setpoint for the ballast.
    units::degree_t last_setpoint;

    frc2::sysid::SysIdRoutine sysid {
        frc2::sysid::Config { 0.35_V / 1_s, 4_V, std::nullopt, std::nullopt },
        frc2::sysid::Mechanism {
        [this](units::volt_t volts) {
            ballast_motor.SetVoltage(volts);
        },
        [this](auto log) {
            log->Motor("ballast")
            .voltage(ballast_motor.Get() * frc::RobotController::GetBatteryVoltage())
            .velocity(units::radians_per_second_t {get_velocity()})
            .position(units::radian_t {get_position()});
        },
        this
        }
    };

    std::optional<frc2::CommandPtr> sysid_command;

}; // class Ballast

} // namespace ballast

} // namespace subsystems