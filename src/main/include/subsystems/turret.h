#pragma once

#include <frc/controller/PIDController.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DutyCycleEncoder.h>


namespace subsystems {

namespace turret {

namespace constants {
    /// @brief this is the canid of the turret motor
    static constexpr int k_turret_id = 95;  
}

class turret {
    public:
        /// @brief idle the turret
        void idle_turret();
        /// @brief set setpoint
        void set_angle(units::degree_t);
        units::degree_t get_angle();
        ///@brief checks if a set point
        bool at_goal_point();
        ///@brief enables and disables the turret
        bool turret_power();

        units::degrees_per_second_t get_vel();

        units::degree_t get_pose();
    private:
        /// @brief variable for the point that the turret is going to
        int set_point = 0;
        /// @brief this will be the value of where the turret is 
        int at_point = 0;
        /// @brief this is the motor that spins the turret
        rev::CANSparkMax pivot{99,rev::CANSparkLowLevel::MotorType::kBrushless};
        /// @brief absolute encoder for the turret
        frc::DutyCycleEncoder turret_encoder { 0 };
        
        frc::PIDController pid{
            0,
            0,
            0
        };

        frc2::sysid::SysIdRoutine sysid {
        frc2::sysid::Config { 0.35_V / 1_s, 4_V, std::nullopt, std::nullopt },
        frc2::sysid::Mechanism {
        [this](units::volt_t volts) {
            pivot.SetVoltage(volts);
        },
        [this](auto log) {
            log->Motor("turret")
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