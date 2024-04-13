#include "subsystems/arm.h"
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace subsystems;

void climi::Climi::init() {
    motor.SetInverted(true); //maybe not true actually
}

void climi::Climi::tick() {
    motor.SetVoltage(motor_voltage);
}

void climi::Climi::update_nt()
{
    frc::SmartDashboard::PutNumber("climi_velocity", get_velocity().value());
}

void climi::Climi::uppy() {
    motor.SetVoltage(1_V);
}

void climi::Climi::downy() {
    motor.SetVoltage(-1_V);
}

void subsystems::climi::Climi::run_sysid(int test_num) {
    if (!sysid_command) {
        switch (test_num) {
        case 0: {
            sysid_command = sysid.Quasistatic(frc2::sysid::Direction::kForward);
            sysid_command->Schedule();
            break;
        }
        case 1: {
            sysid_command = sysid.Quasistatic(frc2::sysid::Direction::kReverse);
            sysid_command->Schedule();
            break;
        }
        case 2: {
            sysid_command = sysid.Dynamic(frc2::sysid::Direction::kForward);
            sysid_command->Schedule();
            break;
        }
        case 3: {
            sysid_command = sysid.Dynamic(frc2::sysid::Direction::kReverse);
            sysid_command->Schedule();
            break;
        }
        }
    }
}

void subsystems::climi::Climi::cancel_sysid() {
    if (sysid_command) {
        sysid_command->Cancel();
    }

    sysid_command = std::nullopt;
}