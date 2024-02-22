#pragma once

#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/angle.h>
#include <units/time.h>
#include <units/voltage.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

namespace subsystems {

namespace intake {

namespace constants {
    enum DeployStates {
        NOSPIN = 0,
        SPIN
    };

    /// @brief The CAN ID for the roller motor.
    constexpr int ROLLER_ID = 15;

    /// @brief The roller intake speed in RPM.
    constexpr units::volt_t ROLLER_INTAKE_SETPOINT = 7_V;
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

    frc2::CommandPtr intake() {
        return this->StartEnd(
            [this]() {
                activate(constants::DeployStates::SPIN);
            },
            [this]() {
                activate(constants::DeployStates::NOSPIN);
            }
        );
    }
private:
    /// @brief The roller motor. 
    ctre::phoenix6::hardware::TalonFX roller_motor {
        constants::ROLLER_ID,
    };

    units::volt_t roller_voltage = 0_V;
}; // class intake

} // namespace intake

} // namespace subsystems