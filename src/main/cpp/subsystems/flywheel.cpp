#include "subsystems/flywheel.h"
#include <frc/smartdashboard/SmartDashboard.h>

void subsystems::flywheel::Flywheel::init() {
    flywheel.SetInverted(constants::FLYWHEEL_DIRECTION);
    feedwheel_motor.SetInverted(constants::FEED_DIRECTION);
}

void subsystems::flywheel::Flywheel::update_nt() {
    // Nothing to log yet
}


void subsystems::flywheel::Flywheel::tick() {
    flywheel.SetVoltage(units::volt_t{pid.Calculate(get_exit_vel().value(), setpoint.value())}
        + flywheel_ff.Calculate(setpoint));
};

units::feet_per_second_t subsystems::flywheel::Flywheel::get_exit_vel(){
    return flywheel_encoder.GetVelocity() * flywheel::constants::fly_ratio * 3_in * std::numbers::pi / 60_s;
};

units::foot_t subsystems::flywheel::Flywheel::get_fly_position(){
    return flywheel_encoder.GetPosition() * flywheel::constants::fly_ratio * 3_in * std::numbers::pi;
};

void subsystems::flywheel::Flywheel::set_exit_vel(units::feet_per_second_t goal) {
    setpoint = goal;
}

void subsystems::flywheel::Flywheel::stop_feed() {
    feedwheel_motor.SetVoltage(0_V);
}

void subsystems::flywheel::Flywheel::feed() {
    if (has_piece()) {
        feedwheel_motor.SetVoltage(0_V);
    } else {
        feedwheel_motor.SetVoltage(7.5_V);
    }
}

void subsystems::flywheel::Flywheel::reverse_feed() {
    if (!has_piece()) {
        feedwheel_motor.SetVoltage(0_V);
    } else {
        feedwheel_motor.SetVoltage(-7.5_V);
    }
}

bool subsystems::flywheel::Flywheel::has_piece(){
    return !piece_sensor.Get();
};

void subsystems::flywheel::Flywheel::run_sysid(int test_num) {
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

void subsystems::flywheel::Flywheel::cancel_sysid() {
    if (sysid_command) {
        sysid_command->Cancel();
    }

    sysid_command = std::nullopt;
}