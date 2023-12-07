#pragma once

#include <frc/controller/PIDController.h>
#include <units/angle.h>
#include <ctre/phoenix6/TalonFX.hpp>

namespace subsystems {

namespace turret {

namespace constants {
    constexpr int MOTOR_ID = 99;

    constexpr double GEAR_RATIO = 1.0;

} // constants

class Turret {
public:
    void set_goal(units::degree_t);
    units::degree_t get_position();
    bool at_goal();

private:
    ctre::phoenix6::hardware::TalonFX motor {constants::MOTOR_ID};

    frc::PIDController pid {
        0,
        0,
        0
    };
};

} // turret

} // subsystems
