#include "subsystems/pivot.h"
#include <frc/smartdashboard/SmartDashboard.h>

void subsystems::pivot::Pivot::init() {
    motor.SetInverted(constants::DIRECTION);
    motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    pid.SetIZone(90);
}

void subsystems::pivot::Pivot::update_nt() {
    units::degree_t dtheta = last_angle - get_angle();
    units::second_t dt = last_time - frc::Timer::GetFPGATimestamp();
    velocity = dtheta / dt;

    // update nt

    frc::SmartDashboard::PutNumber("shooter_pivot_current_angle", get_angle().value());
    frc::SmartDashboard::PutNumber("shooter_pivot_current", motor.GetOutputCurrent());
    //frc::SmartDashboard::PutString("pivot_encoder", std::to_string(pivot_encoder.Get()));

    last_angle = get_angle();
    last_time = frc::Timer::GetFPGATimestamp();
}

void subsystems::pivot::Pivot::tick() {
    setpoint = profile.Calculate(20_ms, setpoint, goal);

    if (get_angle() < 55_deg || setpoint.position < 55_deg) {
        motor.SetVoltage(ff.Calculate(setpoint.position, setpoint.velocity) + units::volt_t{pid.Calculate(setpoint.position.value(), get_angle().value())});
    } else {
        motor.SetVoltage(0_V);
    }
}

// this acts as a softstop by the way
void subsystems::pivot::Pivot::set_angle(units::degree_t goal) {
    goal = goal > 75_deg ? 75_deg : goal;
    this->goal = {goal, 0_deg_per_s};
}

units::degree_t subsystems::pivot::Pivot::get_angle() {
    units::degree_t initial_offset = 0_deg;
    //offset for uppy downy, is added to total
    const units::degree_t constant_offset = 134_deg;
    while (units::math::abs(pivot_encoder.Get() + constant_offset + initial_offset) > 180_deg) {
        initial_offset -= 360_deg * (pivot_encoder.Get() + constant_offset < 0_deg ? -1 : 1);
    }
    return pivot_encoder.Get() + constant_offset + initial_offset;
}

units::degree_t subsystems::pivot::Pivot::get_auto_angle() {
    units::degree_t initial_offset = 0_deg;
    //offset for uppy downy, is added to total
    const units::degree_t constant_offset = 140_deg;
    while (units::math::abs(pivot_encoder.Get() + constant_offset + initial_offset) > 180_deg) {
        initial_offset -= 360_deg * (pivot_encoder.Get() + constant_offset < 0_deg ? -1 : 1);
    }
    return pivot_encoder.Get() + constant_offset + initial_offset;
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