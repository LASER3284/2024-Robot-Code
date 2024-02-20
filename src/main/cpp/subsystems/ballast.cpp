#include <subsystems/ballast.h>
#include <frc/RobotController.h>

using namespace subsystems; 

ballast::Ballast::Ballast() {
    ballast_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    ballast_motor.SetSmartCurrentLimit(25);
    ballast_motor.SetInverted(true);
    set_rotation_goal(get_rotation());
};

void ballast::Ballast::tick(units::degree_t shoulder_rotation) {
    frc::SmartDashboard::PutNumber("ballast_angle", get_rotation().value());
    frc::SmartDashboard::PutNumber("ballast_goal", units::degree_t(ballast_goal.position).value());
}

units::degree_t ballast::Ballast::get_rotation() {
    return thrubore_enc.Get() - 0_deg;
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