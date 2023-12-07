#pragma once

#include <frc/controller/PIDController.h>
#include <units/velocity.h>
#include <ctre/phoenix6/TalonFX.hpp>

namespace subsystems {

namespace flywheel {

namespace constants {
    constexpr int MOTOR_ID = 99;
}

class Flywheel {
public:
    void set_exit_vel_goal(units::feet_per_second_t);
    units::feet_per_second_t get_exit_vel();
private:
    ctre::phoenix6::hardware::TalonFX motor{constants::MOTOR_ID};
};

}    

}
