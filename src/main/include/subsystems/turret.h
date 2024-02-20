#pragma once

#include <frc/controller/PIDController.h>
#include <rev/CANSparkMax.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/RobotController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/ArmFeedforward.h>

// sysid
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc2/command/SubsystemBase.h>

namespace subsystems {

namespace turret {

namespace constants {
    /// @brief The value to give to SetInverted
    constexpr bool DIRECTION = false;
    // CHANGE IF INVERTED      ^----

    constexpr units::degree_t TOLERANCE = 2.5_deg;
}

class Turret : public frc2::SubsystemBase{
public:
    void init();

    void update_nt();
    void tick();

    void run_sysid(int);
    void cancel_sysid();

    /// @brief set setpoint
    void set_angle(units::degree_t);
    units::degree_t get_angle();
    ///@brief checks if a set point
    bool at_goal_point();
private:
    std::optional<frc2::CommandPtr> sysid_command;

    units::degree_t last_angle;
    units::second_t last_time;
    units::degrees_per_second_t velocity;
    
    units::degree_t goal_angle;

    frc::TrapezoidProfile<units::degrees>::Constraints constraints {0_deg_per_s, 0_deg_per_s_sq};

    frc::TrapezoidProfile<units::degrees>::State goal;
    frc::TrapezoidProfile<units::degrees>::State setpoint;

    frc::TrapezoidProfile<units::degrees> profile {constraints};

    frc::ArmFeedforward ff {0.13947_V, 0.00947_V, 0.01325_V / 1_deg_per_s, 0.00103_V / 1_deg_per_s_sq};

    /// @brief variable for the point that the turret is going to
    int set_point = 0;
    /// @brief this will be the value of where the turret is 
    int at_point = 0;
    /// @brief this is the motor that spins the turret
    rev::CANSparkMax motor {21, rev::CANSparkLowLevel::MotorType::kBrushless};
    /// @brief absolute encoder for the turret
    frc::DutyCycleEncoder turret_encoder {8};
    
    frc::PIDController pid {
        0,
        0,
        0
    };

    frc2::sysid::SysIdRoutine sysid {
        frc2::sysid::Config { 0.4_V / 1_s, 3_V, std::nullopt, std::nullopt },
        frc2::sysid::Mechanism {
            [this](units::volt_t volts) {
                motor.SetVoltage(volts);
            },
            [this](auto log) {
                log->Motor("shooter-turret")
                    .voltage(motor.Get() * frc::RobotController::GetBatteryVoltage())
                    .velocity(units::turns_per_second_t{velocity})
                    .position(units::turn_t{get_angle()});
            },
            this
        }
    };
};

}

}