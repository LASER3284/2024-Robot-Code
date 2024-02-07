#include "subsystems/intake.h"
#include <frc/RobotController.h>

using namespace subsystems;

void intake::Intake::init() {
    roller_motor.SetInverted(true);
}

void intake::Intake::tick() {
    roller_motor.SetVoltage(roller_voltage);
}

void intake::Intake::activate(intake::constants::DeployStates state) {
    if (state % 2 == 0)
        roller_voltage = 0_V;
    else {
        roller_voltage = intake::constants::ROLLER_INTAKE_SETPOINT;
    }
}