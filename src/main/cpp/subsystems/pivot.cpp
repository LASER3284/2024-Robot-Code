#include "subsystems/pivot.h"

using namespace subsystems::pivot;

void subsystems::pivot::Pivot::tick(){
///units::volt_t piv_vol = pivot_ff.Calculate();
};
void subsystems::pivot::Pivot::set_angle(units::degree_t){
    if(pivot::Pivot::set_pivot <= 90_deg && pivot::Pivot::set_pivot >= 0_deg){

    }
};
units::degree_t subsystems::pivot::Pivot::get_angle(){

};
bool subsystems::pivot::Pivot::at_angle(){

};
void subsystems::pivot::Pivot::idle_angle(){

};
void subsystems::pivot::Pivot::run_sysid(int test_num) {
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

void subsystems::pivot::Pivot::cancel_sysid() {
    if (sysid_command) {
        sysid_command->Cancel();
    }

    sysid_command = std::nullopt;
}