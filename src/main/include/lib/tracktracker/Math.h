#pragma once

#include <frc/controller/PIDController.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <frc//trajectory/TrapezoidProfile.h>

namespace tracktracker {

struct PidConstants {
    double kP;
    double kI;
    double kD;

    /// @brief Returns a new frc::PIDController based on the contained constants.
    /// @return `frc::PIDController`
    frc::PIDController new_controller() { return frc::PIDController{kP, kI, kD}; }
};

struct Constraints {
    frc::TrapezoidProfile<units::meters>::Constraints linear;
    frc::TrapezoidProfile<units::degrees>::Constraints angular;
};

}