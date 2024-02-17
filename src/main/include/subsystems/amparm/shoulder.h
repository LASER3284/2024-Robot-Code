#pragma once

// General stuff
#include <units/voltage.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/PIDController.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/RobotController.h>
#include <frc/DutyCycleEncoder.h>

// SysID stuff
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>

namespace subsystems {

namespace amparm {

namespace constants {

namespace shoulder {

    /// @brief Whether the motor should be inverted or not (given to SetInverted).
    constexpr bool DIRECTION = true;

    constexpr units::degree_t TOLERANCE = 2_deg;
}

}

class Shoulder : public frc2::SubsystemBase {
public:
    void init();

    void reset();

    void update_nt();

    void tick();

    units::degree_t get_position() const;

    units::degrees_per_second_t get_velocity() const {
        return velocity;
    }

    void set_goal(units::degree_t);

    bool in_place();

    void run_sysid(int);

    void cancel_sysid();

private:
    ctre::phoenix6::hardware::TalonFX motor {17};

    frc::DutyCycleEncoder encoder {1};

    units::degrees_per_second_t velocity;

    units::second_t last_time = frc::Timer::GetFPGATimestamp();
    units::degree_t last_angle = get_position();

    frc::TrapezoidProfile<units::degrees>::Constraints constraints {180_deg / 1_s, 180_deg / 1_s / 1_s};

    frc::TrapezoidProfile<units::degrees>::State goal;
    frc::TrapezoidProfile<units::degrees>::State setpoint;

    frc::TrapezoidProfile<units::degrees> profile {constraints};

    /// @brief Ks, Kg, Kv, Ka
    frc::ArmFeedforward ff {0.43219_V, 0.31952_V, 0.065_V / 1_deg_per_s, 0.017_V / 1_deg_per_s_sq};

    frc::PIDController pid {0.7, 0, 0};

    frc2::sysid::SysIdRoutine sysid {
        frc2::sysid::Config {0.5_V / 1_s, 3_V, std::nullopt, std::nullopt},
        frc2::sysid::Mechanism {
            [this](units::volt_t volts) {
                motor.SetVoltage(volts);
            },
            [this](auto log) {
                log->Motor("amp-shoulder-motor")
                    .position(units::turn_t{get_position()})
                    .velocity(units::turns_per_second_t{get_velocity()})
                    .voltage(motor.Get() * frc::RobotController::GetBatteryVoltage());
            },
            this
        }
    };

    std::optional<frc2::CommandPtr> sysid_command;
};

}

}