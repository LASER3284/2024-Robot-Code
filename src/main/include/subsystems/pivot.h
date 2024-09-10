#pragma once

#include <rev/CANSparkMax.h>
#include <frc/controller/PIDController.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/RobotController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/ArmFeedforward.h>

// sysid
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc2/command/SubsystemBase.h>


namespace subsystems {

namespace pivot {

namespace constants {
    /// @brief The value to give to SetInverted
    constexpr bool DIRECTION = false;
    // CHANGE THIS IF INVERTED ^----

    constexpr units::degree_t TOLERANCE = 1_deg;
}

class Pivot : public frc2::SubsystemBase {
public:
    void init();

    void update_nt();

    void tick();

    void run_sysid(int);

    void cancel_sysid();
    
    /// @brief set the point for the angle of the turn
    void set_angle(units::degree_t);
    units::degree_t get_angle();
    /// @brief checks if at angle
    bool at_angle();
private:
    std::optional<frc2::CommandPtr> sysid_command;
    
    /// @brief this is the absolute encoder
    frc::DutyCycleEncoder pivot_encoder {7};

    units::degree_t last_angle;
    units::second_t last_time;

    units::degrees_per_second_t velocity;

    frc::TrapezoidProfile<units::degrees>::Constraints constraints {180_deg_per_s, 180_deg_per_s_sq};

    frc::TrapezoidProfile<units::degrees>::State goal {44_deg, 0_deg_per_s};
    frc::TrapezoidProfile<units::degrees>::State setpoint;

    frc::TrapezoidProfile<units::degrees> profile {constraints};

    frc::ArmFeedforward ff {0.43058_V, 0.13126_V, 0.0088_V / 1_deg_per_s, 0.00086_V / 1_deg_per_s_sq};

    rev::CANSparkMax motor {23, rev::CANSparkLowLevel::MotorType::kBrushless};

    frc::PIDController pid {
        //0.12, OG
        0.35,
        //0.090816, OG
        0.04504,
        //0.005767 OG
        0.00675
    };

    frc2::sysid::SysIdRoutine sysid {
        frc2::sysid::Config { 0.35_V / 1_s, 4_V, std::nullopt, std::nullopt },
        frc2::sysid::Mechanism {
        [this](units::volt_t volts) {
            motor.SetVoltage(volts);
        },
        [this](auto log) {
            log->Motor("shooter-pivot")
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
