#include "subsystems/flywheel.h"
#include <frc/smartdashboard/SmartDashboard.h>

void subsystems::flywheel::flywheel::tick() {
    flywheel.SetVoltage(units::volt_t{velocity_controller.Calculate(get_exit_vel().value(), setpoint.value())}
        + flywheel_ff.Calculate(setpoint));

    
};

units::feet_per_second_t subsystems::flywheel::flywheel::get_exit_vel() {
    units::feet_per_second_t exit_velocity = flywheel_encoder.GetVelocity() * constants::GEAR_RATIO * constants::WHEEL_DIAMETER;
    return exit_velocity;
};

void subsystems::flywheel::flywheel::shoot() {

}

void subsystems::flywheel::flywheel::low_spit() {

}

bool subsystems::flywheel::flywheel::feed_pow() {
    
}

void subsystems::flywheel::flywheel::update_nt() {
    frc::SmartDashboard::PutNumber("flywheel_exit_vel", get_exit_vel().value());
}
