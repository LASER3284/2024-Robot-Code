#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/controller/PIDController.h>
#include <frc/DutyCycleEncoder.h>

// sysid
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc2/command/SubsystemBase.h>


namespace subsystems {

namespace pivot {

namespace constants {
    /// @brief this is the id of the pivot motor
    constexpr int PIVOT_ID = 94;
}

class pivot : public frc2::SubsystemBase {
public:
    /// @brief set the point for the angle of the turn
    void set_angle(units::degree_t);
    units::degree_t get_angle();
    /// @brief keeps the idle position
    void idle_angle();
    /// @brief checks if at angle
    bool at_angle();

    units::turns_per_second_t get_vel();

    units::turn_t get_pose();
private:
    ///@brief this will be the angle to pivot is currently at 
    int at_pivot = 0;
    ///@brief this will be the angle the pivot wants to go to
    int set_pivot = 0;
    ///@brief this is the value of the voltage/power needed to keep the angle
    int hold_pivot = 0;
    /// @brief this is the absolute encoder
    frc::DutyCycleEncoder pivot_encoder { 0 };

    ctre::phoenix6::hardware::TalonFX pivot{54};

    frc2::sysid::SysIdRoutine sysid {
        frc2::sysid::Config { 0.35_V / 1_s, 4_V, std::nullopt, std::nullopt },
        frc2::sysid::Mechanism {
        [this](units::volt_t volts) {
            pivot.SetVoltage(volts);
        },
        [this](auto log) {
            log->Motor("pivot")
                .voltage(pivot.Get() * frc::RobotController::GetBatteryVoltage())
                .velocity(units::turns_per_second_t{get_vel()})
                .position(units::turn_t{get_pose()});
        },
        this
        }
    };

};

}

}
