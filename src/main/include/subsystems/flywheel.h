#include <frc/controller/PIDController.h>
#include <units/velocity.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <rev/CANSparkFlex.h>
#include <rev/CANSparkMax.h>
#include <frc/controller/SimpleMotorFeedforward.h>

namespace subsystems{

namespace flywheel{

namespace constants {
    /// @brief this is the id of the shooter flywheel motor
    constexpr int FLYWHEEL_ID = 97;
    /// @brief this is the id of the shooter feedwheel motor
    constexpr int FEED_ID = 96;
}

class flywheel{
public:
    /// @brief shoot/ go burrrrr at the disired speed
    void shoot();
    /// @brief this will spit at a low speed
    void low_spit();
    /// @brief this will turn on and off the feed wheel
    bool feed_pow();
    /// @brief sets the exit volocity goal
    void set_exit_vel(units::feet_per_second_t);
    units::feet_per_second_t get_exit_vel();
private:
    /// @brief the value of the shooter motor power(this is to be changed later)
    int shoot_power = 1;
    /// @brief the value of the feed wheel motor power(this is to be changed later)
    int feed_power = 0;

    frc::SimpleMotorFeedforward<units::feet> flywheel_ff {0_V, 0_V / 1_fps, 0_V / 1_fps_sq};

    /// @brief this is the flywheel motor
    rev::CANSparkFlex flywheel{97,rev::CANSparkLowLevel::MotorType::kBrushless};
    /// @brief this is the feedwheel motor
    rev::CANSparkMax feed_motor{99,rev::CANSparkLowLevel::MotorType::kBrushless};
};

}

}