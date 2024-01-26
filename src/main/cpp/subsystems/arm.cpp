#include "subsystems/arm.h"


subsystems::arm::Arm::Arm(){
    extension_motor1.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    extension_motor2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    // extensionMotor.SetInverted(true);
    set_position_goal(get_position());

}

units::foot_t subsystems::arm::Arm::get_position() {
    // auto value = (extensionEncoder.GetPosition() / Constants::ARM_RATIO) * Constants::PULLEY_DIAMETER;
    // return value;

    return 0_ft;
}

void subsystems::arm::Arm::set_position_goal(units::foot_t distance){
    if (distance > 0_in) { distance = 0_in; } // distances to be determined
    // something
        extension_goal = { distance, 0_fps };

}





