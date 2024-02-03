#include "subsystems/intake.h"
#include <frc/RobotController.h>

using namespace subsystems;

void intake::Intake::init() {
    deploy_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    deploy_motor.SetInverted(false);

    roller_motor.SetInverted(true);

    // Value given in degrees
    deploy_controller.SetTolerance(1.0);
    velocity_timer.Start();
}

void intake::Intake::update_nt() {
    // Must be the first thing
    units::degree_t dtheta = previous_angle - get_angle();
    units::second_t dt = previous_time - velocity_timer.Get();
    deploy_velocity = dtheta / dt;

    // Stuff in middle

    // Must be the last thing
    previous_angle = get_angle();
    previous_time = velocity_timer.Get();
}

void intake::Intake::tick() {
    frc::TrapezoidProfile<units::degrees> rot_profile {
        deploy_constraints,
        deploy_goal,
        deploy_setpoint
    };

    deploy_setpoint = rot_profile.Calculate(20_ms);

    units::volt_t deploy_voltage = deploy_ff.Calculate(
        deploy_setpoint.position,
        deploy_setpoint.velocity
    ) + units::volt_t{deploy_controller.Calculate(
        get_angle().value(),
        deploy_setpoint.position.value()
    )};

    frc::SmartDashboard::PutNumber("intake_deploy_v", deploy_voltage.value());

    deploy_motor.SetVoltage(deploy_voltage);

    roller_motor.SetVoltage(roller_voltage);
}

void intake::Intake::activate(intake::constants::DeployStates state) {
    if (state % 2 == 0)
        roller_voltage = 0_V;
    else {
        roller_voltage = intake::constants::ROLLER_INTAKE_SETPOINT;
    }

    if (state > intake::constants::DeployStates::DOWN_SPIN) {
        deploy_goal = {intake::constants::UPPER_LIMIT, 0_deg_per_s};
    } else {
        deploy_goal = {intake::constants::LOWER_LIMIT, 0_deg_per_s};
    }
}

void intake::Intake::set_deploy(units::volt_t volts) {
    deploy_motor.SetVoltage(volts);
}

units::volt_t intake::Intake::get_deploy_volts() {
    return deploy_motor.Get() * frc::RobotController::GetBatteryVoltage();
}

void intake::Intake::set_deploy_goal(units::degree_t angle) {
    deploy_setpoint = {angle};
    deploy_goal = {angle};
}

void intake::Intake::run_sysid(int test_num) {
    if (sysid_command && !sysid_command->IsScheduled()) {
        sysid_command->Schedule();
    } else if (sysid_command && sysid_command->IsScheduled()) {
        // empty code :)
        // wait for something else to cancel the scheduled command
    } else {
        switch (test_num) {
        case 0: {
            sysid_command = sysid.Quasistatic(frc2::sysid::Direction::kForward);
            break;
        }
        case 1: {
            sysid_command = sysid.Quasistatic(frc2::sysid::Direction::kReverse);
            break;
        }
        case 2: {
            sysid_command = sysid.Dynamic(frc2::sysid::Direction::kForward);
            break;
        }
        case 3: {
            sysid_command = sysid.Dynamic(frc2::sysid::Direction::kReverse);
            break;
        }
        default: {
            cancel_sysid();
            break;
        }
        }
    }
}

void intake::Intake::cancel_sysid() {
    if (sysid_command)
        sysid_command->Cancel();

    sysid_command = std::nullopt;
}