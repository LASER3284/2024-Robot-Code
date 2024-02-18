#include <frc/controller/PIDController.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DutyCycleEncoder.h>


namespace subsystems {

namespace turret {

namespace constants {
    /// @brief this is the canid of the turret motor
    static constexpr int k_turret_id = 95;  
}

class turret {
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
    private:
        /// @brief variable for the point that the turret is going to
        int set_point = 0;
        /// @brief this will be the value of where the turret is 
        int at_point = 0;
        /// @brief this is the motor that spins the turret
        ctre::phoenix6::hardware::TalonFX fly_motor{93};
        /// @brief absolute encoder for the turret
        frc::DutyCycleEncoder turret_encoder { 0 };
        
        frc::PIDController pid{
            0,
            0,
            0
        };
};

}

}