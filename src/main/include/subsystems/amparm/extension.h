#pragma once

// General stuff
#include <units/voltage.h>
#include <units/length.h>
#include <units/velocity.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/controller/PIDController.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/RobotController.h>

// SysID stuff
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>

namespace subsystems {

namespace amparm {

namespace constants {

namespace extension {
    constexpr double GEAR_RATIO = 8.519;
    constexpr units::inch_t PULLEY_DIAMETER = 2.256_in;

    /// @brief The direction to use on the Falcon (argument provided to
    /// SetInverted)
    constexpr bool DIRECTION = false;

    constexpr units::inch_t TOLERANCE = 1_in;
}

}

class Extension : public frc2::SubsystemBase {
public:
    void init();

    void update_nt();

    void tick();

    void set_goal(units::inch_t);

    units::inch_t get_position();

    units::feet_per_second_t get_velocity();

    bool in_place();

    void run_sysid(int);

    void cancel_sysid();

private:
    ctre::phoenix6::hardware::TalonFX motor {18};

    frc::TrapezoidProfile<units::inches>::Constraints constraints {24_in / 1_s, 24_in / 1_s / 1_s};

    frc::TrapezoidProfile<units::inches>::State goal;
    frc::TrapezoidProfile<units::inches>::State setpoint;

    frc::TrapezoidProfile<units::inches> profile {constraints};

    /// @brief Ks, Kg, Kv, Ka
    frc::ElevatorFeedforward ff {0.47055_V, 0.37605_V, 1.5945_V / 1_fps, 0.10519_V / 1_fps_sq};

    frc::PIDController pid {0.5, 0, 0};

    frc2::sysid::SysIdRoutine sysid {
        frc2::sysid::Config {0.5_V / 1_s, 3_V, std::nullopt, std::nullopt},
        frc2::sysid::Mechanism {
            [this](units::volt_t volts) {
                motor.SetVoltage(volts);
            },
            [this](auto log) {
                log->Motor("amp-extension-motor")
                    .position(units::meter_t{get_position()})
                    .velocity(units::meters_per_second_t{get_velocity()})
                    .voltage(motor.Get() * frc::RobotController::GetBatteryVoltage());
            },
            this
        }
    };

    std::optional<frc2::CommandPtr> sysid_command;
};

}

}