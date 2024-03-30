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
    switch (state) {
        default:
        case constants::DeployStates::NOSPIN: {
            roller_voltage = 0_V;
        }
        break;
        case constants::DeployStates::SPIN: {
            roller_voltage = constants::ROLLER_INTAKE_SETPOINT;
        }
        break;
        case constants::DeployStates::REVSPIN: {
            roller_voltage = -12_V;
        }
        break;
    }
}