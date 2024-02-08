#include "subsystems/swerve.h"
#include <frc/RobotController.h>

using namespace ctre::phoenix6;

subsystems::swerve::Module::Module(const int drive, const int turn, const int enc) {
    drive_motor = std::make_unique<rev::CANSparkFlex>(drive, rev::CANSparkLowLevel::MotorType::kBrushless);
    turn_motor = std::make_unique<hardware::TalonFX>(turn);
    encoder = std::make_unique<hardware::CANcoder>(enc);

    drive_motor->SetIdleMode(rev::CANSparkFlex::IdleMode::kBrake);
    turn_motor->SetNeutralMode(signals::NeutralModeValue::Brake);

    drive_enc = std::make_unique<rev::SparkRelativeEncoder>(drive_motor->GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor));

    heading_controller.EnableContinuousInput(
        -std::numbers::pi,
        std::numbers::pi
    );

    auto cancoder_heading = get_cancoder_heading();
    if (cancoder_heading) {
        turn_motor->SetPosition(cancoder_heading.value() * constants::TURN_RATIO);
    }
    encoder->OptimizeBusUtilization();
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
            drive_enc->GetPosition() / constants::kDRIVE_RATIO * constants::kWHEEL_CIRC
        },
        get_heading()
    };
}

void subsystems::swerve::Module::set_desired_goal(const frc::SwerveModuleState& ref_state, bool force) {
    const frc::Rotation2d encoder_rotation {get_heading()};

    auto state = frc::SwerveModuleState::Optimize(ref_state, encoder_rotation);

    if (!force && units::math::abs(state.speed) < constants::kMAX_WHEEL_SPEED * 0.01) {
        state.angle = last_angle;
    }
    else {
        last_angle = state.angle;
    }

    state.speed *= (state.angle - encoder_rotation).Cos();

    const units::volt_t drive_power = units::volt_t{drive_controller.Calculate(
        get_velocity().value(),
        state.speed.value()
    )} + drive_ff.Calculate(state.speed);

    const units::volt_t turn_power = units::volt_t{heading_controller.Calculate(
        encoder_rotation.Radians().value(),
        state.angle.Radians().value()
    )};

    set_drive_power(drive_power);
    _set_turn_power(turn_power);
}

units::radian_t subsystems::swerve::Module::get_heading() const {
    return turn_motor->GetPosition().GetValue() / constants::TURN_RATIO;
}

units::feet_per_second_t subsystems::swerve::Module::get_velocity() const {
    return units::feet_per_second_t{
        drive_enc->GetVelocity() / constants::kDRIVE_RATIO * constants::kWHEEL_CIRC / 60_s
    };
}

void subsystems::swerve::Module::reset_drive_position() {
    drive_enc->SetPosition(0);
}

void subsystems::swerve::Module::set_drive_power(const units::volt_t volts) {
    drive_motor->SetVoltage(volts);
}

void subsystems::swerve::Module::_set_turn_power(const units::volt_t volts) {
    turn_motor->SetVoltage(volts);
}

void subsystems::swerve::Module::apply_heading_goal(const units::radian_t angle) {
    turn_motor->SetVoltage(units::volt_t{
        heading_controller.Calculate(get_heading().value(), angle.value())
    });
}

units::volt_t subsystems::swerve::Module::get_drive_power() const {
    return drive_motor->Get() * frc::RobotController::GetBatteryVoltage();
}

void subsystems::swerve::Module::force_update_azimuth() {
    auto cancoder_heading = get_cancoder_heading();
    if (cancoder_heading)
        turn_motor->SetPosition(cancoder_heading.value() * constants::TURN_RATIO);
}

std::optional<units::degree_t> subsystems::swerve::Module::get_cancoder_heading() const {
    auto status = encoder->GetPosition();
    if (ctre::phoenix6::BaseStatusSignal::IsAllGood(status)) {
        return status.GetValue();
    }
    return std::nullopt;
}