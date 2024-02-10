#include <frc/controller/PIDController.h>
#include <units/velocity.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <rev/CANSparkFlex.h>
#include <rev/CANSparkMax.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/RobotController.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc2/command/SubsystemBase.h>


namespace subsystems{

namespace flywheel{

namespace constants {
    /// @brief this is the id of the shooter flywheel motor
    constexpr int FLYWHEEL_ID = 97;
    /// @brief this is the id of the shooter feedwheel motor
    constexpr int FEED_ID = 96;
    /// @brief this is the pid values for proportional gain(kp)
     constexpr double FLY_KP = 0.0;
    /// @brief interfral gain(ki)
    constexpr double FLY_KI = 0.0; 
    /// @brief derivative gain (kd)
    constexpr double FLY_KD= 0.0;

}

class Flywheel : public frc2::SubsystemBase{
public:
    
    /// @brief ticks 
    void tick();

    /// @brief sets the exit fly volocity goal
    void set_exit_vel(units::feet_per_second_t);
    units::feet_per_second_t get_exit_vel();

    /// @brief gets the positions of the feed and fly wheel motor
    units::foot_t get_fly_position();
    /// @brief this like the name say resets the sysid.
    void cancel_sysid();


private:
    /// @brief the value of the shooter motor power(this is to be changed later)
    int shoot_power = 1;
    /// @brief the value of the feed wheel motor power(this is to be changed later)
    int feed_power = 0;

    frc::SimpleMotorFeedforward<units::feet> flywheel_ff {0_V, 0_V / 1_fps, 0_V / 1_fps_sq};

    /// @brief this is the flywheel motor
    rev::CANSparkFlex flywheel_motor{97,rev::CANSparkLowLevel::MotorType::kBrushless};
    /// @brief this is the feedwheel motor
    rev::CANSparkMax feedwheel_motor{99,rev::CANSparkLowLevel::MotorType::kBrushless};
    
    // Creates a PIDController with gains kP, kI, and kD
    frc::PIDController Flywheel_PID{constants::FLY_KP, constants::FLY_KI, constants::FLY_KD};

    /// this is my atempt at sysid with my very limited knowledge
    frc2::sysid::SysIdRoutine sysid{
        frc2::sysid::Config{0.25_V / 1-s, 4_V, std::nullopt, std::nullopt},
        frc2::sysid::Mechanism{
        [this](units::volt_t volts) {
            flywheel_motor.SetVoltage(volts);
        },
        [this](auto log){
            log->Motor("flywheel-1")
                .voltage(flywheel_motor.Get()* frc::RobotController::GetBatteryVoltage())
                .velocity(units::meters_per_second_t{get_exit_vel()})
                .position(units::meter_t {get_fly_position()});

        },
        this
        }
    };
};

}

}
