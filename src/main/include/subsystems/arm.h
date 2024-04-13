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
    constexpr double GEAR_RATIO = 44.44;
    constexpr units::inch_t PULLEY_DIAMETER = 1.25_in;

    constexpr bool DIRECTION = true;



namespace extension {


} //extension
} //constants

class Climi : public frc2::SubsystemBase {
public:
    void init();

    void tick();

    void reset();

    void set_goal(units::inch_t);

    units::inch_t get_position();

    bool in_place();

    void run_sysid(int);

    void cancel_sysid();

    void uppy();

    void downy();

    void stop();

private:
    void set_position(units::inch_t);

    ctre::phoenix6::hardware::TalonFX motor {40};

    frc::TrapezoidProfile<units::inches>::Constraints constraints {48_in / 1_s, 60_in / 1_s / 1_s};

    frc::TrapezoidProfile<units::inches>::State goal;
    frc::TrapezoidProfile<units::inches>::State setpoint;

    frc::TrapezoidProfile<units::inches> profile {constraints};

    frc::ElevatorFeedforward ff{0.54078_V, 0.50262_V, 1.21088_V / 1_fps, 0.08067_V / 1_fps_sq};

    frc::PIDController pid {0.6, 0, 0};

    units::volt_t motor_voltage = 0_V;
};
} //arm
} //subsystems