#pragma once

#include <units/voltage.h>
#include <units/length.h>
#include <units/velocity.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/controller/PIDController.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/RobotController.h>
#include <frc/DigitalInput.h>

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>

namespace subsystems{

namespace climi{

namespace constants {
    constexpr double GEAR_RATIO = 44.5;
    constexpr units::inch_t PULLEY_DIAMETER = 1.2_in;

    constexpr bool DIRECTION = false;



namespace extension {


} //extension
} //constants

class Climi : public frc2::SubsystemBase {
public:
    void init();

    void update_nt();

    void tick();

    void reset();

    void set_goal(units::inch_t);

    units::inch_t get_position();

    units::feet_per_second_t get_velocity();

    bool in_place();

    void run_sysid(int);

    void cancel_sysid();

    void uppy();

    void downy();

private:
    void set_position(units::inch_t);

    ctre::phoenix6::hardware::TalonFX motor {40};

    frc::TrapezoidProfile<units::inches>::Constraints constraints {48_in / 1_s, 60_in / 1_s / 1_s};

    frc::TrapezoidProfile<units::inches>::State goal;
    frc::TrapezoidProfile<units::inches>::State setpoint;

    frc::TrapezoidProfile<units::inches> profile {constraints};

    frc::ElevatorFeedforward ff{0.54078_V, 0.50262_V, 1.21088_V / 1_fps, 0.08067_V / 1_fps_sq};

    frc::PIDController pid {0.6, 0, 0};

    frc2::sysid::SysIdRoutine sysid {
        frc2::sysid::Config {0.5_V / 1_s, 3_V, std::nullopt, std::nullopt},
        frc2::sysid::Mechanism {
            [this](units::volt_t volts) {
                motor.SetVoltage(volts);
            },
            [this](auto log) {
                log->Motor("motor")
                    .position(units::meter_t{get_position()})
                    .velocity(units::meters_per_second_t{get_velocity()})
                    .voltage(motor.Get() * frc::RobotController::GetBatteryVoltage());
            },
            this
        }
    };

    std::optional<frc2::CommandPtr> sysid_command;

    units::volt_t motor_voltage = 0_V;
};
} //arm
} //subsystems