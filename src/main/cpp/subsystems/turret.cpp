#include "subsystems/turret.h"
#include <numbers>
#include <frc/smartdashboard/SmartDashboard.h>

void subsystems::turret::Turret::init() {
    motor.SetInverted(constants::DIRECTION);
    pid.SetIZone(360);
}

void subsystems::turret::Turret::tick() {
    setpoint = profile.Calculate(20_ms, setpoint, goal);

    if (units::math::abs(get_angle()) < 170_deg || units::math::abs(setpoint.position) < 170_deg) {
        motor.SetVoltage(ff.Calculate(setpoint.position, setpoint.velocity) + units::volt_t{pid.Calculate(setpoint.position.value(), get_angle().value())});
    } else {
        motor.SetVoltage(0_V);
    }
}

void subsystems::turret::Turret::set_angle(units::degree_t goal) {
    goal = units::math::abs(goal) < 160_deg ? goal : (160_deg * (goal < 0_deg ? -1 : 1));
    this->goal = {goal, 0_deg_per_s};
}

units::degree_t subsystems::turret::Turret::get_angle() {
    // negative bc thrubore is wrong direction on physical bot
    units::degree_t initial_offset = 0_deg;
    const units::degree_t constant_offset = 141_deg;
    while (units::math::abs(turret_encoder.Get() + constant_offset + initial_offset) > 180_deg) {
        initial_offset -= 360_deg * (turret_encoder.Get() + constant_offset < 0_deg ? -1 : 1);
    }
    return -(turret_encoder.Get() + constant_offset + initial_offset);
}

bool subsystems::turret::Turret::at_goal_point() {
    return units::math::abs(goal.position - get_angle()) < constants::TOLERANCE;
}

void subsystems::turret::Turret::update_nt() {
    units::degree_t dtheta = last_angle - get_angle();
    units::second_t dt = last_time - frc::Timer::GetFPGATimestamp();
    velocity = dtheta / dt;

    // update nt
    frc::SmartDashboard::PutNumber("shooter_turret_current_angle", get_angle().value());

    last_angle = get_angle();
    last_time = frc::Timer::GetFPGATimestamp();
}

// sysid
void subsystems::turret::Turret::run_sysid(int test_num) {
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

void subsystems::turret::Turret::cancel_sysid() {
    if (sysid_command) {
        sysid_command->Cancel();
    }

    sysid_command = std::nullopt;
}
