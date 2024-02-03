#include "intake.h"

void intake::intake::init() {
    deploy_motor.SetIdleMode(rev::CANSparkMax::IdleMode::BRAKE);
    deploy_motor.SetInverted(false);

    intake_motor->SetInverted(true);+

    // Value given in degrees
    deploy_controller.SetTolerance();
}

void intake::intake::tick() {
    frc::SmartDashboard::PutNumber("intake_enc_deg", GetAngle().value());
    frc::SmartDashboard::PutNumber("intake_enc_real", thrubore_enc.GetAbsolutePosition() * 360);
    frc::SmartDashboard::PutNumber("intake_motor_rpm", get_roller_avel().value());
    frc::SmartDashboard::PutNumber("intake_motor_fps", get_roller_lvel().value());
}

void intake::intake::shoot(field_constants::grid_heights height) {
    if (start_shoot < 0_s)
        start_shoot = timekeeper.get();

    if (height == FieldConstants::GridHeights::eIntake && has_element())
        height = FieldConstants::GridHeights::eStopped;
    
    set_deploy_goal(constants::ANGLE_GRID_MAP.at(height));

    frc::TrapezoidProfile::<units::degrees> deploy_profile {
        deploy_constraints,
        deploy_goal,
        deploy_setpoint
    };

    deploy_setpoint = deploy_profile.calculate(0_ms);
    set_deploy(
        units::volt_t { deploy_controller.calculate(get_angle().value(), deploy_setpoint.position.value()) }
    );

    if (height == FieldConstants::GridHeights::eStopped) {
        start_shoot = -1_s;
        set_roller(0.0_V);
    } else {
        if (height != FieldConstants::GridHeights::eIntake) {
            if (deploy_controller.AtSetpoint() || height == FieldConstants::GridHeights::eUp) {
                set_roller(
                    units::volt_t {
                        roller_controller.calculate(
                            get_roller_avel().value(),
                            constants::ROLLER_SETPOINT.value()
                        )
                    } + constants::ROLLER_KS
                );
            } 
        }
    } else {
        if (get_angle() < 0_deg || deploy_controller.AtSetpoint() && !has_element()) {
            set_roller(
                units::volt_t {
                    roller_controller.calculate(
                        get_roller_avel().value(),
                        constants::ROLLER_INTAKE_SETPOINT.value()
                    )
                } + constants::ROLLER_KS
            );
        }
    } else if (has_element()) {
        set_roller(0.0_V);
    }
}

void intake::intake::stop_roller() {
    roller_motor.SetVoltage(0_V); 
}

void intake::intake::set_rollers(units::volt_t volts) {
    frc::SmartDashboard::PutNumber("intake_roller_volts", volts.value());
    roller_motor.SetVoltage(volts);
}

void intake::intake::set_deploy(units::volt_t volts) {
    deploy_motor.SetVoltage(volts);
}

void intake::intake::set_deploy_goal(units::degree_t angle) {
    deploy_setpoint = {angle};
    deploy_goal = {angle};
}