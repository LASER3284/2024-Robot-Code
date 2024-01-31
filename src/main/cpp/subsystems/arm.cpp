#include "subsystems/arm.h"


subsystems::arm::Arm::Arm() {
    extension_motor1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    extension_motor2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    set_position_goal1(get_position1());
    set_position_goal2(get_position2());

}

units::foot_t subsystems::arm::Arm::get_position1() {
    auto value = (relative_encoder1.GetPosition() / constants::ARM_RATIO) * constants::PULLEY_DIAMETER;
    return value;
}

units::foot_t subsystems::arm::Arm::get_position2() {
    auto value = (relative_encoder2.GetPosition() / constants::ARM_RATIO) * constants::PULLEY_DIAMETER;
    return value;
}

units::feet_per_second_t subsystems::arm::Arm::get_velocity1() {
    auto value = (relative_encoder1.GetVelocity() / constants::ARM_RATIO) * constants::PULLEY_DIAMETER / 1_s;
    return value;
}

units::feet_per_second_t subsystems::arm::Arm::get_velocity2() {
    auto value = (relative_encoder2.GetVelocity() / constants::ARM_RATIO) * constants::PULLEY_DIAMETER / 1_s;
    return value;
}

void subsystems::arm::Arm::set_position_goal1(units::foot_t distance) {
    if (distance > 0_in) { distance = 0_in; } // distances to be determined
    position_controller1.SetSetpoint(distance.value());
        extension_goal1 = { distance, 0_fps };

}

void subsystems::arm::Arm::set_position_goal2(units::foot_t distance) {
    if (distance > 0_in) { distance = 0_in; } // tbd
    position_controller2.SetSetpoint(distance.value());
        extension_goal2 = { distance, 0_fps};

}

void subsystems::arm::Arm::tick() { 
    frc::TrapezoidProfile<units::feet> extension_profile1 {
        constraints,
        extension_goal1,
        extension_setpoint1,
    };
    frc::TrapezoidProfile<units::feet> extension_profile2 {
        constraints,
        extension_goal2,
        extension_setpoint2,

    };
    extension_setpoint1 = extension_profile1.Calculate(20_ms);
    extension_setpoint2 = extension_profile2.Calculate(20_ms);
    const auto output_voltage1 = feedforward.Calculate(extension_setpoint1.velocity);
    const auto output_voltage2 = feedforward.Calculate(extension_setpoint2.velocity);

    extension_motor1.SetVoltage(
        units::volt_t(position_controller1.Calculate(get_position1().value(), extension_setpoint1.position.value()))
        + output_voltage1
    );

    extension_motor2.SetVoltage(
        units::volt_t(position_controller2.Calculate(get_position2().value(), extension_setpoint2.position.value()))
        + output_voltage2
    );
}