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

    constexpr float GEAR_RATIO = 1.33;

    constexpr units::feet_per_second_t SHOOT_SPEED = 0_fps;

    constexpr units::foot_t WHEEL_DIAMETER = 3_in;
}

class flywheel{
public:
    void tick();
    /// @brief shoot/ go burrrrr at the desired speed
    void shoot();
    /// @brief this will spit at a low speed
    void low_spit();
    /// @brief this will turn on and off the feed wheel
    bool feed_pow();
    /// @brief sets the exit volocity goal
    void set_exit_vel(units::feet_per_second_t exit_vel) {
        setpoint = exit_vel;
    };
    units::feet_per_second_t get_exit_vel();

    // this is literally only for sysid so just ignore it i suppose
    units::foot_t get_pose();

    void update_nt();
private:
    /// @brief the value of the shooter motor power(this is to be changed later)
    int shoot_power = 1;
    /// @brief the value of the feed wheel motor power(this is to be changed later)
    int feed_power = 0;

    units::feet_per_second_t setpoint;

    frc::SimpleMotorFeedforward<units::feet> flywheel_ff {0_V, 0_V / 1_fps, 0_V / 1_fps_sq};

     frc::PIDController velocity_controller {
        0,
        0,
        0
    };

    /// @brief this is the flywheel motor
    rev::CANSparkFlex flywheel{97,rev::CANSparkLowLevel::MotorType::kBrushless};
    /// @brief this is the flywheel encoder!
    rev::SparkRelativeEncoder flywheel_encoder = flywheel.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
    /// @brief this is the feedwheel motor
    rev::CANSparkMax feed_motor{99,rev::CANSparkLowLevel::MotorType::kBrushless};

    frc2::sysid::SysIdRoutine sysid {
        frc2::sysid::Config { 0.35_V / 1_s, 4_V, std::nullopt, std::nullopt },
        frc2::sysid::Mechanism {
        [this](units::volt_t volts) {
            flywheel.SetVoltage(volts);
        },
        [this](auto log) {
            log->Motor("flywheel")
                .voltage(flywheel.Get() * frc::RobotController::GetBatteryVoltage())
                .velocity(units::meters_per_second_t{get_exit_vel()})
                .position(units::meter_t {get_pose()});
        },
        this
        }
    };
};

}

}