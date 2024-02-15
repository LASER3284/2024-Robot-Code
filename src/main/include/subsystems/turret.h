#include <frc/controller/PIDController.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/RobotController.h>
#include <rev/CANSparkMax.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc2/command/SubsystemBase.h>
#include <units/velocity.h>
#include <frc/Timer.h>
#include <rev/CANSparkFlex.h>

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

class Turret : frc2::SubsystemBase {
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

        void run_sysid(int);

        void tick();

        bool TRT_Saftey();
        
        /// @brief used to find the delta theta so can get d-theta/dt
        units::degree_t previous_angle;

        units::second_t previous_time;

        units::degrees_per_second_t deploy_velocity;

        frc::Timer velocity_timer{};

        units::degree_t cur_angle;


    private:

        units::degree_t goal_angle;

        bool Is_Safe = true;
        
        /// @brief this is the motor that spins the turret
        rev::CANSparkFlex Turret_motor{97,rev::CANSparkLowLevel::MotorType::kBrushless};
        /// @brief this is the pid contrller
        frc::PIDController Turret_PID{ constants::TRT_KP,constants::TRT_KI,constants::TRT_KD};
        /// @brief this is the encoder for the turret 
        rev::SparkRelativeEncoder turret_encoder = Turret_motor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
        /// @brief this is the feed forward for the turret
        frc::SimpleMotorFeedforward<units::degree_t> turret_ff  {0_V, 0_V / 1_fps, 0_V / 1_fps_sq};


        /// this is my atempt at sysid with my very limited knowledge
    frc2::sysid::SysIdRoutine sysid{
        frc2::sysid::Config{0.25_V / 1_s, 4_V, std::nullopt, std::nullopt},
        frc2::sysid::Mechanism{
        [this](units::volt_t volts) {
            Turret_motor.SetVoltage(volts);
        },
        [this](auto log){
            log->Motor("turret-1")
                .voltage(Turret_motor.Get()* frc::RobotController::GetBatteryVoltage())
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