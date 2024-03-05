#include "subsystems/flywheel.h"
#include <frc/smartdashboard/SmartDashboard.h>

void subsystems::flywheel::Flywheel::init() {
    motor.SetInverted(constants::FLYWHEEL_DIRECTION);
    feedwheel_motor.SetInverted(constants::FEED_DIRECTION);
}

void subsystems::flywheel::Flywheel::update_nt() {
    frc::SmartDashboard::PutNumber("shooter_flywheel_velocity", get_exit_vel().value());
    frc::SmartDashboard::PutNumber("shooter_flywheel_current", motor.GetOutputCurrent());
}


void subsystems::flywheel::Flywheel::tick() {
    motor.SetVoltage(units::volt_t{pid.Calculate(get_exit_vel().value(), setpoint.value())}
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

void subsystems::flywheel::Flywheel::feed(bool fire) {
    if (fire) {
        feedwheel_motor.SetVoltage(12_V);
    } else {
        if (!has_piece())
            feedwheel_motor.SetVoltage(1_V);
        else
            feedwheel_motor.SetVoltage(0_V);
    }
}

void subsystems::flywheel::Flywheel::reverse_feed() {
    if (piece_sensor.Get()) {
        feedwheel_motor.SetVoltage(0_V);
    } else {
        feedwheel_motor.SetVoltage(-7.5_V);
    }
}

bool subsystems::flywheel::Flywheel::has_piece() {
    /*
    if (piece_sensor.Get()) {
        return false;
    } else {
        if (units::math::abs(goal_feed - units::turn_t{feed_enc.GetPosition()}) < units::turn_t{0.1}) {
            return true;
        }
        goal_feed = units::turn_t{feed_enc.GetPosition()} + units::turn_t{0.25};
        return false;
    }
    */
   return !piece_sensor.Get();
};

bool subsystems::flywheel::Flywheel::at_speed() {
    return units::math::abs(setpoint - get_exit_vel()) < constants::TOLERANCE;
}

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
