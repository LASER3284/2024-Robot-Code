#include "subsystems/turret.h"
#include <numbers>
#include "shooter.cpp"

using namespace subsystems::turret;

void subsystems::turret::Turret::tick(){

};
void subsystems::turret::Turret::set_angle(units::degree_t goal){
    ///goal_angle = units::math::atan2(-shooter::constants::DELTA_Y, delta_x);
    if(turret::Turret::goal_angle <=180_deg || turret::Turret::goal_angle <=-180_deg ){
    
}
};
units::degree_t subsystems::turret::Turret::get_angle(){
    
   ///turret_encoder.get();
};
bool subsystems::turret::Turret::at_goal_point(){

};
void subsystems::turret::Turret::idle_turret(){
    turret.SetVoltage(0_V);
};
bool subsystems::turret::Turret::turret_power(){

};

// sysid
void subsystems::turret::Turret::run_sysid(int test_num) {
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

void subsystems::turret::Turret::cancel_sysid() {
    if (sysid_command) {
        sysid_command->Cancel();
    }

    sysid_command = std::nullopt;
}
