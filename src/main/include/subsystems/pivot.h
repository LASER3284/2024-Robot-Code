#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/controller/PIDController.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/RobotController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
/// Sysid
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc2/command/SubsystemBase.h>

namespace subsystems {

namespace pivot {

namespace constants {
    /// @brief this is the id of the pivot motor
    constexpr int PIVOT_ID = 94;
    /// @brief this is the pid values for proportional gain(kp)
    constexpr double PIV_KP = 0.0;
    /// @brief interfral gain(ki)
    constexpr double PIV_KI = 0.0; 
    /// @brief derivative gain (kd)
    constexpr double PIV_KD= 0.0;
    /// @brief 
}

class Pivot : frc2::SubsystemBase {
public:
    /// @brief 
    void tick();
    /// @brief set the point for the angle of the turn
    void set_angle(units::degree_t);
    units::degree_t get_angle();
    /// @brief keeps the idle position
    void idle_angle();
    /// @brief checks if at angle
    bool at_angle();
    /// @brief this reset the sysid
    void cancel_sysid();
    void run_sysid(int);
    /// gets the velocity for the speed of the change of the pivot
    units::degrees_per_second_t get_vel();



private:
    ///@brief this will be the angle to pivot is currently at 
    int at_pivot = 0;
    ///@brief this will be the angle the pivot wants to go to
    units::degree_t set_pivot = 0_deg;
    ///@brief this is the value of the voltage/power needed to keep the angle
    int hold_pivot =0;
    /// @brief this is the pivot motor
    ctre::phoenix6::hardware::TalonFX PIV_motor{91,"idk"};
    /// @brief this is the aboslute incoder
    frc::DutyCycleEncoder Pivot_encoder { 0 };
    /// @brief pivot feed forward
     frc::SimpleMotorFeedforward<units::degree_t> pivot_ff  {0_V, 0_V / 1_fps, 0_V / 1_fps_sq};


    frc::PIDController PIV_PID{ constants::PIV_KP,constants::PIV_KI,constants::PIV_KD};
    /// @brief this is my attempt at pivot sysid
    frc2::sysid::SysIdRoutine sysid{
        frc2::sysid::Config{0.25_V / 1_s, 4_V, std::nullopt, std::nullopt},
        frc2::sysid::Mechanism{
        [this](units::volt_t volts) {
            PIV_motor.SetVoltage(volts);
        },
        [this](auto log){
            log->Motor("turret-1")
                .voltage(PIV_motor.Get()* frc::RobotController::GetBatteryVoltage())
                .velocity(units::turns_per_second_t{get_vel()})
                .position(units::turn_t{get_angle()});
        },
        this
        }
    };

    std::optional<frc2::CommandPtr> sysid_command;
};

}

}
