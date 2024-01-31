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
#include <frc/RobotController.h>

// sysid
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc2/command/SubsystemBase.h>

namespace subsystems {

namespace arm {

namespace constants {
    // comments are here to mark what i need to do in the class, ignore for
    // now

    /// @brief The CAN IDs for the Neos for extending the arms
    constexpr int ARM_CAN_ID_1 = 0;
    constexpr int ARM_CAN_ID_2 = 0;


    /// @brief The gear ratio for the arms (sprockets are 1 to 1)
    constexpr double ARM_RATIO = 16.2/1;

    /// @brief The outer diameter of the pulley * pi in order to convert to linear units
    constexpr units::inch_t PULLEY_DIAMETER = (1_in * 3.14); // to be changed

    /// @brief The constant kG value for the arm extension This value is
    /// defined here rather than in the feedforward because we need to do
    /// custom math with it due to the extension being on the pivot Since
    /// the effect of gravity would change based on the shoulder angle
    /// rather than be a constant value

} // namespace constants

class Arm : public frc2::SubsystemBase {
public:
    Arm();

    void tick();

    units::foot_t get_position1();
    units::foot_t get_position2();

    units::feet_per_second_t get_velocity1();
    units::feet_per_second_t get_velocity2();

    void set_position_goal1(units::foot_t distance);
    void set_position_goal2(units::foot_t distance);

    units::foot_t get_position_goal1() {}
    units::foot_t get_position_goal2() {}

    void manual_control(double percentage) { manual_percentage = percentage; }

    void cancel_sysid();

    void refresh_controller()
    {
        position_controller1.Reset();
        position_controller2.Reset();
        extension_setpoint1 = {get_position1(), 0_fps};
        extension_setpoint2 = {get_position2(), 0_fps};
        extension_goal1 = {get_position1(), 0_fps};
        extension_goal2 = {get_position2(), 0_fps};
    }

private:

    double manual_percentage = 0.0;

    rev::CANSparkMax extension_motor1 {constants::ARM_CAN_ID_1, rev::CANSparkMaxLowLevel::MotorType::kBrushless}; // canid wrong
    rev::SparkMaxRelativeEncoder relative_encoder1 = extension_motor1.GetEncoder();

    rev::CANSparkMax extension_motor2 {constants::ARM_CAN_ID_2, rev::CANSparkMaxLowLevel::MotorType::kBrushless}; // canid wrong
    rev::SparkMaxRelativeEncoder relative_encoder2 = extension_motor2.GetEncoder();

    // same thing with these, but to help myself

    /// @brief the feedforward for the extension of the arms (it acts
    /// as an "elevator")
    frc::ElevatorFeedforward feedforward { 0_V, 0_V, 0_V / 1_fps, 0_V / 1_fps_sq }; 

    frc::PIDController position_controller1 { 0, 0, 0 };
    frc::PIDController position_controller2 { 0, 0, 0 };

    frc::TrapezoidProfile<units::feet>::Constraints constraints { 0_fps, 0_fps_sq };

    /// @brief the goal for the arm 1 and arm 2 to extend to
    frc::TrapezoidProfile<units::feet>::State extension_goal1;
    frc::TrapezoidProfile<units::feet>::State extension_goal2;

    /// @brief the current set point for arm 1 and arm 2
    frc::TrapezoidProfile<units::feet>::State extension_setpoint1;
    frc::TrapezoidProfile<units::feet>::State extension_setpoint2;

    /// @brief A timer used for overriding the manual percentage vs the
    /// feedforward calculations
    frc::Timer control_timer;

    units::foot_t last_goal;

    frc2::sysid::SysIdRoutine sysid {
        frc2::sysid::Config {0.35_V/1_s, 4_V , std::nullopt, std::nullopt},
        frc2::sysid::Mechanism {
        [this](units::volt_t volts) {
            extension_motor1.SetVoltage(volts);
            extension_motor2.SetVoltage(volts);
        },
        [this](auto log) {
            log->Motor("arm-1")
                .voltage(extension_motor1.Get() * frc::RobotController::GetBatteryVoltage())
                .velocity(units::meters_per_second_t{get_velocity1()})
                .position(units::meter_t {get_position1()});
            log->Motor("arm-2")
                .voltage(extension_motor2.Get() * frc::RobotController::GetBatteryVoltage())
                .velocity(units::meters_per_second_t{get_velocity2()})                    
                .position(units::meter_t {get_position2()});
        },
        this
        }

    };

    std::optional<frc2::CommandPtr> sysid_command; 

}; // class Arm

} // namespace arm

} // namespace subsystems