#include "subsystems/shooter.h"
#include "subsystems/pivot.h"
#include "subsystems/turret.h"

#include <units/angle.h>
#include <units/time.h>

#include <frc/smartdashboard/SmartDashboard.h>

void subsystems::shooter::Shooter::init() {
    turret.init();
    flywheel.init();
    pivot.init();
}

void subsystems::shooter::Shooter::update_nt() {
    turret.update_nt();
    flywheel.update_nt();
    pivot.update_nt();
}

void subsystems::shooter::Shooter::run_sysid(int test_num, subsystems::shooter::constants::SubMech mech) {
    switch (mech) {
        constants::SubMech::Flywheel:
            flywheel.run_sysid(test_num);
            break;
        constants::SubMech::Pivot:
            pivot.run_sysid(test_num);
            break;
        constants::SubMech::Turret:
            turret.run_sysid(test_num);
            break;
        default:
            break;
    }
}