#pragma once
#include <frc/controller/PIDController.h>
#include <units/velocity.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <rev/CANSparkFlex.h>
#include <rev/CANSparkMax.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/RobotController.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
namespace subsystems{

namespace flywheel{

namespace constants {
    /// @brief this is the id of the shooter flywheel motor
    constexpr int FLYWHEEL_ID = 97;
    /// @brief this is the id of the shooter feedwheel motor
    constexpr int FEED_ID = 96;
    /// @brief this is the pid values for proportional gain(kp)
     constexpr double FLY_KP = 0.0;
    /// @brief intergral gain(ki)
    constexpr double FLY_KI = 0.0; 
    /// @brief derivative gain (kd)
    constexpr double FLY_KD= 0.0;
    /// @brief this is the gear ratio of the flywheel
    constexpr double fly_ratio = 1.66;
    
}  

class Flywheel : public frc2::SubsystemBase{
public:
    
    /// @brief ticks 
    void tick();
    ///@brief this will set the flywheel to a constant low speed mode
    void idle();
    /// @brief this will turn on the feed motor 
    void feed();
    /// @brief sets the exit fly volocity goal
    void set_exit_vel(units::feet_per_second_t);
    units::feet_per_second_t get_exit_vel();

    /// @brief gets the positions of the feed and fly wheel motor
    units::foot_t get_fly_position();
    /// @brief this like the name say resets the sysid.
    void cancel_sysid();
    
    void run_sysid(int);

    bool check_if_ring();
    
private:
    bool fly_has_ring = false;
    /// @brief set the exit vel for the flywheel
    units::feet_per_second_t fly_speed = 0_ft; 
    
    frc::SimpleMotorFeedforward<units::feet> flywheel_ff {0_V, 0_V / 1_fps, 0_V / 1_fps_sq};

    /// @brief this is the flywheel motor
    rev::CANSparkFlex flywheel{97,rev::CANSparkLowLevel::MotorType::kBrushless};

    units::feet_per_second_t setpoint;

    /// @brief this is the feedwheel motor
    rev::CANSparkMax feedwheel_motor{99,rev::CANSparkLowLevel::MotorType::kBrushless};
    /// @brief this is the can spark flex encoder
    rev::SparkRelativeEncoder flywheel_encoder = flywheel.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
    /// @brief Creates a PIDController with gains kP, kI, and kD
     
    frc::SimpleMotorFeedforward<units::feet> flywheel_ff {0_V, 0_V / 1_fps, 0_V / 1_fps_sq};
    frc::PIDController Flywheel_PID{constants::FLY_KP, constants::FLY_KI, constants::FLY_KD};
    /// @brief this is the sensor for the shooter
    frc::DigitalInput Fly_sense;

     frc::PIDController velocity_controller {
        0,
        0,
        0
    };
    /// this is my attempt at sysid with my very limited knowledge
    frc2::sysid::SysIdRoutine sysid{
        frc2::sysid::Config{0.25_V / 1_s, 4_V, std::nullopt, std::nullopt},
        frc2::sysid::Mechanism{
        [this](units::volt_t volts) {
            flywheel.SetVoltage(volts);
        },
        [this](auto log){
            log->Motor("flywheel-1")
                .voltage(flywheel.Get()* frc::RobotController::GetBatteryVoltage())
                .velocity(units::meters_per_second_t{get_exit_vel()})
                .position(units::meter_t {get_fly_position()});

        },
        this
        }
    };
    std::optional<frc2::CommandPtr> sysid_command;
};

}

}