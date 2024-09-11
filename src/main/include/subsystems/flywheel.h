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
#include <ctre/phoenix6/CANcoder.hpp>
namespace subsystems{

namespace flywheel{

namespace constants {
    /// @brief The value to give to SetInverted
    constexpr bool FLYWHEEL_DIRECTION = false;

    /// @brief The value to give to SetInverted
    constexpr bool FEED_DIRECTION = true;

    constexpr units::feet_per_second_t TOLERANCE = 8_fps;

    /// @brief this is the pid values for proportional gain(kp)
     constexpr double FLY_KP = 0.0;
    /// @brief intergral gain(ki)
    constexpr double FLY_KI = 0.0; 
    /// @brief derivative gain (kd)
    constexpr double FLY_KD = 0.0;
    /// @brief this is the gear ratio of the flywheel
    constexpr double fly_ratio = 1.66;
    
}  

class Flywheel : public frc2::SubsystemBase{
public:
    void init();
    void update_nt();
    /// @brief ticks 
    void tick();
    /// @brief this will set the flywheel to a constant low speed mode
    void stop_feed();

    bool at_speed();

    void reverse_feed();
    /// @brief this will turn on the feed motor
    void feed(bool);
    /// @brief sets the exit fly volocity goal
    void set_exit_vel(units::feet_per_second_t);
    units::feet_per_second_t get_exit_vel();

    /// @brief gets the positions of the feed and fly wheel motor
    units::foot_t get_fly_position();
    /// @brief this like the name say resets the sysid.
    void cancel_sysid();
    
    void run_sysid(int);

    bool has_piece();

    // void prespin();
    
private:
    units::turn_t goal_feed = units::turn_t{0};
    frc::SimpleMotorFeedforward<units::feet> flywheel_ff {0_V, 0.08283_V / 1_fps, 0.02161_V / 1_fps_sq};

    /// @brief this is the flywheel motor
    rev::CANSparkFlex motor {20, rev::CANSparkLowLevel::MotorType::kBrushless};

    units::feet_per_second_t setpoint;

    /// @brief this is the feedwheel motor
    ctre::phoenix6::hardware::TalonFX feedwheel_motor {22};

    /// @brief this is the feedwheel encoder
    std::unique_ptr<ctre::phoenix6::hardware::CANcoder> feed_enc;

    /// @brief this is the can spark flex encoder
    rev::SparkRelativeEncoder flywheel_encoder = motor.GetEncoder();
    /// @brief Creates a PIDController with gains kP, kI, and kD
     
    /// @brief this is the sensor for the shooter
    frc::DigitalInput piece_sensor {9};

     frc::PIDController pid {
        0.025,
        0.035,
        0.001
    };
    /// this is my attempt at sysid with my very limited knowledge
    frc2::sysid::SysIdRoutine sysid{
        frc2::sysid::Config{0.25_V / 1_s, 4_V, std::nullopt, std::nullopt},
        frc2::sysid::Mechanism{
        [this](units::volt_t volts) {
            motor.SetVoltage(volts);
        },
        [this](auto log){
            log->Motor("shooter-flywheel")
                .voltage(motor.Get()* frc::RobotController::GetBatteryVoltage())
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