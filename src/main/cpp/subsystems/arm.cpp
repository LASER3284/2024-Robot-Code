#include "subsystems/arm.h"
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace subsystems;

void climi::Climi::init() {
    motor.SetInverted(true); //maybe not true actually
    motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
}

void climi::Climi::tick() {
    if((get_position() <= 0_in && motor_voltage < 0_V) || (get_position() >= 27_in && motor_voltage > 0_V)) {
        motor_voltage = 0_V;
    }   
    motor.SetVoltage(motor_voltage);

    frc::SmartDashboard::PutNumber("arm pose", get_position().value());
}

units::inch_t climi::Climi::get_position() {
    return motor.GetPosition().GetValue() / units::turn_t{1} / constants::GEAR_RATIO * (constants::PULLEY_DIAMETER * std::numbers::pi);;

}


void climi::Climi::uppy() {
    motor_voltage = 12_V;
}

void climi::Climi::downy() {
    motor_voltage = -9_V;
}

void climi::Climi::stop() {
    motor_voltage = 0_V;
}
