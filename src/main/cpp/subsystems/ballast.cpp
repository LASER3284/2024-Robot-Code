#include <subsystems/ballast.h>
#include <frc/RobotController.h>

using namespace subsystems; 

ballast::Ballast::Ballast() {
    ballast_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    ballast_motor.SetSmartCurrentLimit(25);
    ballast_motor.SetInverted(true);
    set_rotation_goal(get_rotation());
    ballast_timer.Restart();
};

void ballast::Ballast::tick(units::degree_t shoulder_rotation) {
    frc::SmartDashboard::PutNumber("balance_angle", get_rotation().value());
    frc::SmartDashboard::PutNumber("balance_goal", units::degree_t(ballast_goal.position).value());
    frc::SmartDashboard::PutNumber("balance_timer_s", ballast_timer.Get().value());

    if(manual_percentage != 0) {
        if ((get_rotation().value() >= -90 && get_rotation().value() <= 90) || manual_percentage * (get_rotation().value() >= 0 ? 1 : -1) < 0) {
            ballast_motor.SetVoltage((manual_percentage * 12_V));
            ballast_timer.Restart();
            ballast_goal = { get_rotation(), 0_deg_per_s };
            ballast_setpoint = { get_rotation(), 0_deg_per_s };
        } else {
            ballast_motor.SetVoltage(0_V);
        }
    }
    else {
        if(units::math::abs(ballast_goal.position - get_rotation()) > 0.0_deg) {
            frc::TrapezoidProfile<units::radians> rotational_profile { 
                rotational_constraints, 
                ballast_goal,
                ballast_setpoint,
            };

            ballast_setpoint = rotational_profile.Calculate(0_ms);
        }

        frc::SmartDashboard::PutNumber("ballast_setpoint_velocity", units::degrees_per_second_t(ballast_setpoint.velocity).value());
        frc::SmartDashboard::PutNumber("ballast_setpoint_position", units::degree_t(ballast_setpoint.position).value());
        frc::SmartDashboard::PutNumber("ballast_setpoint_position", units::degree_t(ballast_goal.position).value());

        units::volt_t ff = (constants::BALLAST_KS * wpi::sgn(ballast_setpoint.velocity));
        frc::SmartDashboard::PutNumber("ballast_ff_v", ff.value());
        frc::SmartDashboard::PutNumber("ballast_pid_v", controller.Calculate(units::radian_t(get_rotation()).value(), ballast_goal.position.value()));
        const auto voltage = ff + units::volt_t(controller.Calculate(units::radian_t(get_rotation()).value(), ballast_goal.position.value()));
        //if ((get_rotation().value() >= -90 && get_rotation().value() <= 90) || voltage.value() * (get_rotation().value() >= 0 ? -1 : 1) < 0 )
        ballast_motor.SetVoltage(voltage);
        // else
        // balance_motor.SetVoltage(0_V);
    }
}

void ballast::Ballast::set_rotation_goal(units::degree_t rot) {
    ballast_goal =  { rot, 0_deg_per_s };
}

void subsystems::ballast::Ballast::run_sysid(int test_num) {
    if (!sysid_command) {
        switch (test_num) {
        case 0: {
            sysid_command = sysid.Quasistatic(frc2::sysid::Direction::kForward);
            sysid_command->Schedule();
            break;
        }
        case 1: (test_num); {
            sysid_command = sysid.Quasistatic(frc2::sysid::Direction::kReverse);
            sysid_command->Schedule();
            break;
        }
        case 2: (test_num); {
            sysid_command = sysid.Dynamic(frc2::sysid::Direction::kForward);
            sysid_command->Schedule();
            break;
        }
        case 3: (test_num); {
            sysid_command = sysid.Dynamic(frc2::sysid::Direction::kReverse);
            sysid_command->Schedule();
            break;
        }
        }
    }
}

void subsystems::ballast::Ballast::cancel_sysid() {
    if (sysid_command) {
        sysid_command->Cancel();
    }

    sysid_command = std::nullopt;
}