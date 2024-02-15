#include "subsystems/flywheel.h"
#include <numbers>
using namespace subsystems::flywheel;

void subsystems::flywheel::Flywheel::tick(){
    units::volt_t fly_vol = flywheel_ff.Calculate(flywheel::Flywheel::fly_speed);
    flywheel_motor.SetVoltage(fly_vol);
};

void subsystems::flywheel::Flywheel::set_exit_vel(units::feet_per_second_t vel){
    flywheel::Flywheel::fly_speed = vel;
};

units::feet_per_second_t subsystems::flywheel::Flywheel::get_exit_vel(){
    return (flywheel_encoder.GetVelocity()*flywheel::constants::fly_ratio*3_in*std::numbers::pi)/60_s;
};

units::foot_t subsystems::flywheel::Flywheel::get_fly_position(){
     return (flywheel_encoder.GetVelocity()*flywheel::constants::fly_ratio*3_in*std::numbers::pi);
};
void subsystems::flywheel::Flywheel::feed(){
    if(flywheel::constants::fly_has_ring == true){
        feedwheel_motor.SetVoltage(0_V);
    }
}
bool subsystems::flywheel::Flywheel::check_if_ring(){
    if(Fly_sense.Get() == true){
        flywheel::constants::fly_has_ring = true;
    };
    if(Fly_sense.Get() == false){
        flywheel::constants::fly_has_ring = false;
    }

};
void subsystems::flywheel::Flywheel::run_sysid(int test_num) {
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

void subsystems::flywheel::Flywheel::cancel_sysid() {
    if (sysid_command) {
        sysid_command->Cancel();
    }

    sysid_command = std::nullopt;
}