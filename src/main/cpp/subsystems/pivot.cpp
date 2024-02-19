#include "subsystems/pivot.h"
#include <frc/smartdashboard/SmartDashboard.h>


using namespace subsystems::pivot;

void subsystems::pivot::Pivot::init() {
    pivot.SetInverted(constants::DIRECTION);
}

void subsystems::pivot::Pivot::update_nt() {
    units::degree_t dtheta = last_angle - get_angle();
    units::second_t dt = last_time - frc::Timer::GetFPGATimestamp();
    velocity = dtheta / dt;

    // update nt
    frc::SmartDashboard::PutNumber("shooter_pivot_angle", get_angle().value());

    last_angle = get_angle();
    last_time = frc::Timer::GetFPGATimestamp();
}

void subsystems::pivot::Pivot::tick() {
    setpoint = profile.Calculate(20_ms, setpoint, goal);

    pivot.SetVoltage(ff.Calculate(setpoint.position, setpoint.velocity) + units::volt_t{pid.Calculate(setpoint.position.value(), get_angle().value())});
}

void subsystems::pivot::Pivot::set_angle(units::degree_t goal) {
    this->goal = {goal, 0_deg_per_s};
}

units::degree_t subsystems::pivot::Pivot::get_angle() {
    return pivot_encoder.Get() - 0_deg;
    //                           ^---- CHANGE THIS
}

bool subsystems::pivot::Pivot::at_angle() {
    return units::math::abs(goal.position - get_angle()) < constants::TOLERANCE;
}

// sysid
void subsystems::pivot::Pivot::run_sysid(int test_num) {
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

void subsystems::pivot::Pivot::cancel_sysid() {
    if (sysid_command) {
        sysid_command->Cancel();
    }

    sysid_command = std::nullopt;
}