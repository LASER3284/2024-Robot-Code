#include "subsystems/swerve.h"

using namespace ctre::phoenix6;

subsystems::swerve::Module::Module(const int drive, const int turn, const int enc) {
    drive_motor = std::make_unique<hardware::TalonFX>(drive);
    turn_motor = std::make_unique<hardware::TalonFX>(turn);
    encoder = std::make_unique<hardware::CANcoder>(enc);

    heading_controller.EnableContinuousInput(
        -std::numbers::pi,
        std::numbers::pi
    );
}

frc::SwerveModuleState subsystems::swerve::Module::get_state() const {
    return {
        get_velocity(),
        get_heading()
    };
}

frc::SwerveModulePosition subsystems::swerve::Module::get_position() const {
    return {
        units::meter_t{
            drive_motor->GetPosition().GetValue().value() / constants::kDRIVE_RATIO * constants::kWHEEL_CIRC
        },
        get_heading()
    };
}

void subsystems::swerve::Module::set_desired_goal(const frc::SwerveModuleState& ref_state, bool force) {
    frc::Rotation2d encoder_rotation{get_heading()};

    auto state = frc::SwerveModuleState::Optimize(ref_state, encoder_rotation);

    state.speed *= (state.angle - encoder_rotation).Cos();

    const units::volt_t drive_power = units::volt_t{drive_controller.Calculate(
        get_velocity().value(),
        state.speed.value()
    )};

    const units::volt_t turn_power = units::volt_t{heading_controller.Calculate(
        get_heading().value(),
        state.angle.Radians().value()
    )};

    _set_drive_power(drive_power);
    _set_turn_power(turn_power);
}

units::radian_t subsystems::swerve::Module::get_heading() const {
    return encoder->GetPosition().GetValue();
}

units::meters_per_second_t subsystems::swerve::Module::get_velocity() const {
    return units::meters_per_second_t{
        drive_motor->GetVelocity().GetValue().value() / constants::kDRIVE_RATIO * constants::kWHEEL_CIRC / 1_s
    };
}

void subsystems::swerve::Module::reset_drive_position() {
    drive_motor->SetPosition(0_deg);
}

void subsystems::swerve::Module::_set_drive_power(const units::volt_t volts) {
    drive_motor->SetVoltage(volts);
}

void subsystems::swerve::Module::_set_turn_power(const units::volt_t volts) {
    turn_motor->SetVoltage(volts);
}
