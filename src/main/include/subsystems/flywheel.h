#include <frc/controller/PIDController.h>
#include <units/velocity.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/system/plant/DCMotor.h>
#include <rev/CANSparkFlex.h>
namespace subsystems{

namespace flywheel{

namespace constants {
    /// @brief this is the id of the shooter flywheel motor
    const int k_shot_fly_id = 97;
    /// @brief this is the id of the shooter feedwheel motor
    const int k_shot_feed_id = 96;
}
class flywheel{
    public:
        /// @brief shoot/ go burrrrr at the disired speed
        void Shoot();
        /// @brief this will spit at a low speed
        void low_spit();
        /// @brief this will turn on and off the feed wheel
        bool feed_pow();
        /// @brief sets the exit volocity goal
        void set_exit_vel_exit(units::feet_per_second_t);
        units::feet_per_second_t get_exit_vel();
    private:
        ///@brief the value of the shooter motor power(this is to be changed later)
        int shoot_power = 1;
        ///@brief the value of the feed wheel motor power(this is to be changed later)
        int feed_power = 0;
        /// @brief this is the flywheel motor
        rev::CANSparkFlex flywheel{97,rev::CANSparkLowLevel::MotorType::kBrushless};
        /// @ breif this is the feedwheel motor
        ctre::phoenix6::hardware::TalonFX feed_motor{99,"idk"};
};

}

}