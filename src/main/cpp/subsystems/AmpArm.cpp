#include "subsystems/AmpArm.h"
#include <units/angle.h>
#include <numbers>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>

// AMPARM SECTION //
void subsystems::amparm::AmpArm::init() {
    roller.init();
    shoulder.init();
    extension.init();
}

void subsystems::amparm::AmpArm::tick() {
    shoulder.tick();
    extension.tick();
}

void subsystems::amparm::AmpArm::update_nt() {
    shoulder.update_nt();
    extension.update_nt();

    frc::SmartDashboard::PutBoolean("amparm_inplace", in_place());
    frc::SmartDashboard::PutBoolean("amparm_haspiece", has_piece());
}

void subsystems::amparm::AmpArm::activate(constants::States state) {
    switch (state) {
        default:
        case constants::States::Stopped:
            roller.stop();
            extension.set_goal(constants::DOWN_EXTENSION);
            if (extension.in_place())
                shoulder.set_goal(constants::DOWN_ANGLE);
            break;
        case constants::States::Intake:
            shoulder.set_goal(constants::DOWN_ANGLE);
            if (!has_piece() && in_place())
                roller.spin();
            extension.set_goal(constants::DOWN_EXTENSION);
            break;
        case constants::States::Feed:
            shoulder.set_goal(constants::DOWN_ANGLE);
            if (in_place())
                roller.spin();
            extension.set_goal(constants::DOWN_EXTENSION);
            break;
        case constants::States::ReverseFeed:
            shoulder.set_goal(constants::DOWN_ANGLE);
            if (!has_piece())
                roller.reverse();
            extension.set_goal(constants::DOWN_EXTENSION);
            break;
        case constants::States::AmpScore:
            shoulder.set_goal(constants::AMPSCORE_ANGLE);
            if (shoulder.in_place())
                extension.set_goal(constants::AMPSCORE_EXTENSION);
            break;
        case constants::States::Spit:
            roller.spin();
            break;
    }
}

void subsystems::amparm::AmpArm::run_sysid(int test_num, subsystems::amparm::constants::AmpArmSubmechs mech) {
    switch (mech) {
        case constants::AmpArmSubmechs::ShoulderMech:
            shoulder.run_sysid(test_num);
            break;
        case constants::AmpArmSubmechs::ExtensionMech:
            extension.run_sysid(test_num);
            break;
        default:
            break;
    }
}

void subsystems::amparm::AmpArm::cancel_sysid() {
    shoulder.cancel_sysid();
    extension.cancel_sysid();
}

void subsystems::amparm::AmpArm::reset() {
    shoulder.reset();
}

// ROLLER SECTION //
void subsystems::amparm::Roller::init() {
    motor.SetInverted(true);
}

// SHOULDER SECTION //
void subsystems::amparm::Shoulder::init() {
    using namespace subsystems::amparm::constants::shoulder;
    motor.SetInverted(DIRECTION);
}

units::degree_t subsystems::amparm::Shoulder::get_position() const {
    return (encoder.Get() - 155_deg);
}

void subsystems::amparm::Shoulder::reset() {
    setpoint = {get_position(), 0_deg_per_s};
    goal = {get_position(), 0_deg_per_s};
}

void subsystems::amparm::Shoulder::update_nt() {
    // Calculate dtheta/dt
    units::second_t dt = frc::Timer::GetFPGATimestamp() - last_time;
    units::degree_t dtheta = get_position() - last_angle;
    velocity = dtheta / dt;

    // Do other things
    frc::SmartDashboard::PutNumber("amparm_shoulder_pos", get_position().value());
    frc::SmartDashboard::PutBoolean("amp_shoulder_inplace", in_place());

    // Set last_position and last_time
    last_time = frc::Timer::GetFPGATimestamp();
    last_angle = get_position();
}

void subsystems::amparm::Shoulder::tick() {
    setpoint = profile.Calculate(20_ms, setpoint, goal);

    frc::SmartDashboard::PutNumber("amp_shoulder_pos_setp", setpoint.position.value());
    frc::SmartDashboard::PutNumber("amp_shoulder_vel_setp", setpoint.velocity.value());

    frc::SmartDashboard::PutNumber("amp_shoulder_volts", ff.Calculate(setpoint.position, setpoint.velocity).value());



    motor.SetVoltage(units::volt_t{pid.Calculate(get_position().value(), setpoint.position.value())}
        + ff.Calculate(setpoint.position, setpoint.velocity)
    );
}

void subsystems::amparm::Shoulder::set_goal(units::degree_t goal) {
    this->goal = {goal, 0_deg_per_s};
}

bool subsystems::amparm::Shoulder::in_place() {
    using namespace subsystems::amparm::constants::shoulder;
    return units::math::abs(goal.position - get_position()) < TOLERANCE;
}

void subsystems::amparm::Shoulder::run_sysid(int test_num) {
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

void subsystems::amparm::Shoulder::cancel_sysid() {
    if (sysid_command) {
        sysid_command->Cancel();
    }

    sysid_command = std::nullopt;
}

// EXTENSION SECTION //
void subsystems::amparm::Extension::init() {
    using namespace subsystems::amparm::constants::extension;
    motor.SetInverted(DIRECTION);
    motor.SetPosition(0_deg);
    motor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
}

void subsystems::amparm::Extension::reset() {
    setpoint = {get_position(), 0_fps};
    goal = {get_position(), 0_fps};
}

units::inch_t subsystems::amparm::Extension::get_position() {
    using namespace subsystems::amparm::constants::extension;
    // Multiply by 2 at the end bc of the cascading effect where one pulley rotation is extension of 2 pulley circumference
    return (motor.GetPosition().GetValue() / units::turn_t{1}) / GEAR_RATIO * PULLEY_DIAMETER * std::numbers::pi * 2;
}

units::feet_per_second_t subsystems::amparm::Extension::get_velocity() {
    using namespace subsystems::amparm::constants::extension;
    // Multiply by 2 at the end bc of the cascading effect. See get_position.
    return (motor.GetVelocity().GetValue() / units::turn_t{1}) / GEAR_RATIO * PULLEY_DIAMETER * std::numbers::pi * 2;
}

void subsystems::amparm::Extension::set_goal(units::inch_t goal) {
    this->goal = {goal, 0_in / 1_s};
}

void subsystems::amparm::Extension::tick() {
    setpoint = profile.Calculate(20_ms, setpoint, goal);

    motor.SetVoltage(0_V);
    //    units::volt_t{pid.Calculate(get_position().value(), setpoint.position.value())}
    //    + ff.Calculate(setpoint.velocity)
    //);
}

bool subsystems::amparm::Extension::in_place() {
    using namespace subsystems::amparm::constants::extension;
    return units::math::abs(goal.position - get_position()) < TOLERANCE;
}

void subsystems::amparm::Extension::update_nt() {
    frc::SmartDashboard::PutNumber("amparm_extension", get_position().value());
    frc::SmartDashboard::PutBoolean("amp_extension_inplace", in_place());
}

void subsystems::amparm::Extension::run_sysid(int test_num) {
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

void subsystems::amparm::Extension::cancel_sysid() {
    if (sysid_command) {
        sysid_command->Cancel();
    }

    sysid_command = std::nullopt;
}
