#pragma once

#include <frc/trajectory/TrapezoidProfile.h>
#include <units/velocity.h>
#include <memory>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <map>
#include <units/angle.h>
#include <units/time.h>
#include <units/voltage.h>
#include <rev/CANSparkMax.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/Timer.h>
#include <optional>

namespace subsystems {

namespace intake {

namespace constants {
    enum DeployStates {
        NOSPIN = 0,
        SPIN = 0
    };

    /// @brief The CAN ID for the roller motor.
    constexpr int ROLLER_ID = 0;

    /// @brief The statically applied voltage for the roller motor.
    constexpr auto ROLLER_KS = 0.0_V;
    /// @brief The applied voltage per velocity unit for the roller motor.
    constexpr auto ROLLER_KV = 0.0_V / 1_deg_per_s;
    /// @brief The applied voltage per acceleration unit for the roller motor.
    constexpr auto ROLLER_KA = 0.0_V / 1_deg_per_s_sq;

    /// @brief The roller intake speed in RPM.
    constexpr units::volt_t ROLLER_INTAKE_SETPOINT = 5_V;

    /// @brief The upper setpoint limit.
    /// @todo Determine real value on encoder
    constexpr units::degree_t UPPER_LIMIT = 90_deg;
    /// @brief The lower setpoint limit.
    /// @todo Determine real value on encoder
    constexpr units::degree_t LOWER_LIMIT = 0_deg;

    constexpr units::second_t ROLLER_DELAY = 0_ms;
} // namespace constants

class Intake : public frc2::SubsystemBase {
public:
    /// @brief Initializes the motors and ensures break mode on the NEO.
    void init();

    /// @brief Updates the state of the motors and gives new voltages to feed to
    /// the motors. Should be called every enabled periodic, except
    /// TestPeriodic().
    void tick();

    /// @brief Converts from the enum value to a setpoint for the roller voltage
    /// and the deploy goal.
    /// @param state The desired state of the intake. This can only be applied
    /// through periodic calls to tick().
    void activate(constants::DeployStates);
private:
    /// @brief The roller motor. 
    ctre::phoenix6::hardware::TalonFX roller_motor {
        constants::ROLLER_ID,
    };

    units::volt_t roller_voltage = 0_V;
}; // class intake

} // namespace intake

} // namespace subsystems