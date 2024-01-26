#pragma once

#include <units/length.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/acceleration.h>
#include <units/time.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/CANSparkMax.h> 

namespace subsystems {

namespace arm {

namespace constants {
    // comments are here to mark what i need to do in the class, ignore for
    // now

    /// @brief The CAN IDs for the Neos for extending the arms
    constexpr int ARM_CAN_ID_1 = 0;
    constexpr int ARM_CAN_ID_2 = 0;


    /// @brief The gear ratio for the arms (sprockets are 1 to 1)
    constexpr double ARM_RATIO = 17.07/1;

    /// @brief The outer diameter of the pulley * pi in order to convert to linear units
    constexpr double PULLEY_DIAMETER = (1.25 * 3.14); // to be changed

    /// @brief The constant kG value for the arm extension This value is
    /// defined here rather than in the feedforward because we need to do
    /// custom math with it due to the extension being on the pivot Since
    /// the effect of gravity would change based on the shoulder angle
    /// rather than be a constant value

} // namespace constants

class Arm {
public:
    Arm();

    units::foot_t get_position();

    void set_position_goal(units::foot_t distance);

    units::foot_t get_positional_goal() {}

    void toggle_control(bool enable) { bEnabled = enable; }

    void manual_control(double percentage) { manual_percentage = percentage; }

    void refresh_controller()
    {
        
    }

private:

    bool bEnabled = false;
    double manual_percentage = 0.0;

    rev::CANSparkMax extension_motor1 {constants::ARM_CAN_ID_1, rev::CANSparkMaxLowLevel::MotorType::kBrushless}; // wrong
    rev::CANSparkMax extension_motor2 {constants::ARM_CAN_ID_2, rev::CANSparkMaxLowLevel::MotorType::kBrushless}; // wrong

    // same thing with these, but to help myself

    /// @brief the feedforward object for the extension of the arms (it acts
    /// an ""elevator"")
    frc::ElevatorFeedforward feedforward { 0_V, 0_V, 0_V / 1_fps, 0_V / 1_fps_sq }; 

    frc::PIDController position_controller { 0, 0, 0 };

    frc::TrapezoidProfile<units::feet>::Constraints constraints { 0_fps, 0_fps_sq };

    /// @brief the goal for the arms to extend to
    frc::TrapezoidProfile<units::feet>::State extension_goal;

    /// @brief the current set point for the arms
    frc::TrapezoidProfile<units::feet>::State extension_setpoint;

    /// @brief A timer used for overriding the manual percentage vs the
    /// feedforward calculations
    frc::Timer control_timer;

    units::foot_t last_goal;
}; // class Arm

} // namespace arm

} // namespace subsystems