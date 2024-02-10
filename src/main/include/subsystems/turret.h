#include <frc/controller/PIDController.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DutyCycleEncoder.h>
#include <frc/RobotController.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc2/command/SubsystemBase.h>
#include <units/velocity.h>
#include <frc/Timer.h>

namespace subsystems {
 
namespace turret {

namespace constants {
    /// @brief this is the id of the turret motor
    static constexpr int k_turret_id = 95;  
     /// @brief this is the pid values for proportional gain(kp)
     constexpr double TRT_KP = 0.0;
    /// @brief interfral gain(ki)
    constexpr double TRT_KI = 0.0; 
    /// @brief derivative gain (kd)
    constexpr double TRT_KD= 0.0;
}

class Turret {
    public:
        /// @brief idle the turret
        void idle_turret();
        /// @brief set setpoint
        void set_angle(units::degree_t);
        units::degree_t get_angle();
        ///@brief checks if a set point
        bool at_goal_point();
        ///@brief enables and disables the turret
        bool turret_power();
        /// @brief get the turn val 
        units::degrees_per_second_t get_vel(){};
        void cancel_sysid();

        void tick();

        /// @brief used to find the delta theta so can get d-theta/dt
        units::degree_t previous_angle;

        units::second_t previous_time;

        units::degrees_per_second_t deploy_velocity;

        frc::Timer velocity_timer{};

    private:
        /// @brief varuale for the point that the turret is going to
        int set_point = 0;
        /// @brief this will be the value of where the turrent is 
        int at_point = 0;
        /// @brief this is the motor that spins the turret
        ctre::phoenix6::hardware::TalonFX TRT_motor{93,"idk"};
        /// @brief absolute incoder for th turret
        frc::DutyCycleEncoder turret_encoder { 0 };
        
        frc::PIDController TRT_PID{ constants::TRT_KP,constants::TRT_KI,constants::TRT_KD};

        /// this is my atempt at sysid with my very limited knowledge
    frc2::sysid::SysIdRoutine sysid{
        frc2::sysid::Config{0.25_V / 1-s, 4_V, std::nullopt, std::nullopt},
        frc2::sysid::Mechanism{
        [this](units::volt_t volts) {
            TRT_motor.SetVoltage(volts);
        },
        [this](auto log){
            log->Motor("turret-1")
                .voltage(TRT_motor.Get()* frc::RobotController::GetBatteryVoltage())
                .velocity(units::degrees_per_second_t{get_vel()})
                .position(units::degree_t{get_angle()});

        },
        this
        }
    };
};

}

}