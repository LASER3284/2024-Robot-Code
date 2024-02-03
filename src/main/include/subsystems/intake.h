#pragma once

#include <frc/trajectory/TrapezoidProfile.h>
#include <units/velocity.h>
#include <memory>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <map>
#include <units/angle.h>
#include <units/time.h>
#include <units/voltage.h>
#include <rev/CANSparkMax.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/Timer.h>
#include <optional>

namespace subsystems {

namespace intake {

namespace constants {
    enum DeployStates {
        DOWN_NOSPIN = 0,
        DOWN_SPIN,
        UP_NOSPIN,
        UP_SPIN
    };

    /// @brief The CAN ID for the roller motor.
    constexpr int ROLLER_ID = 0;
    /// @brief The CAN ID for the deploy motor.
    constexpr int DEPLOY_ID = 0;

    /// @brief Angle offset for the thru-bore (this is subtracted from the reported angle).
    constexpr units::degree_t THRUBORE_ANGLE_OFFSET = -0_deg;
    
    // NOTE: The PID values for the deploy motor are set with max control effort from SysID at 0V.

    /// @brief The proportional gain for the deploy controller.
    constexpr double DEPLOY_P = 0.0;
    /// @brief The integral gain for the deploy controller.
    constexpr double DEPLOY_I = 0.0;
    /// @brief The derivative gain for the deploy controller.
    constexpr double DEPLOY_D = 0.0;

    /// @brief The statically applied voltage for the roller motor.
    constexpr auto ROLLER_KS = 0.0_V;
    /// @brief The applied voltage per velocity unit for the roller motor.
    constexpr auto ROLLER_KV = 0.0_V / 1_deg_per_s;
    /// @brief The applied voltage per acceleration unit for the roller motor.
    constexpr auto ROLLER_KA = 0.0_V / 1_deg_per_s_sq;

    /// @brief The roller intake speed in RPM.
    constexpr units::volt_t ROLLER_INTAKE_SETPOINT = 5_V;

    /// @brief The upper setpoint limit.
    /// @todo Determine real value on encoder
    constexpr units::degree_t UPPER_LIMIT = 90_deg;
    /// @brief The lower setpoint limit.
    /// @todo Determine real value on encoder
    constexpr units::degree_t LOWER_LIMIT = 0_deg;

    constexpr units::second_t ROLLER_DELAY = 0_ms;
} // namespace constants

class Intake : public frc2::SubsystemBase {
public:
    /// @brief Initializes the motors and ensures break mode on the NEO.
    void init();

    /// @brief Updates SmartDashboard values, should be called in RobotPeriodic().
    void update_nt();

    /// @brief Updates the state of the motors and gives new voltages to feed to
    /// the motors. Should be called every enabled periodic, except
    /// TestPeriodic().
    void tick();

    /// @brief Converts from the enum value to a setpoint for the roller voltage
    /// and the deploy goal.
    /// @param state The desired state of the intake. This can only be applied
    /// through periodic calls to tick().
    void activate(constants::DeployStates);

    /// @brief Calculates the angle of the intake (i.e, whether it's up or down).
    /// @return The angle in degrees.
    units::degree_t get_angle() const {
        return thrubore_enc.Get() + constants::THRUBORE_ANGLE_OFFSET;
    }

    units::degrees_per_second_t get_angle_vel() {
        return deploy_velocity;
    }

    void run_sysid(int);

    void cancel_sysid();
private:
    /// @brief Used to find the delta-theta so we can get d-theta/dt.
    units::degree_t previous_angle;

    units::second_t previous_time;

    units::degrees_per_second_t deploy_velocity;

    frc::Timer velocity_timer {};

    /// @brief Sets the voltage of the deploy motor.
    /// @param volts The voltage that should be applied to the deploy motor.
    void set_deploy(units::volt_t volts);

    /// @brief Fetches the amount of voltage applied to the motor as a
    /// percentage of battery voltage.
    /// @return See brief :)
    units::volt_t get_deploy_volts();

    /// @brief Has the side effect of setting the deploy_setpoint and deploy_goal to the specified position.
    /// @param angle The desired position.
    /// @warning Bounds checking is not done for this function! Be cautious of its side effects.
    void set_deploy_goal(units::degree_t angle);
    
    /// @brief The deploy motor.
    rev::CANSparkMax deploy_motor {
        constants::DEPLOY_ID,
        rev::CANSparkMaxLowLevel::MotorType::kBrushless
    };

    /// @brief The encoder that measures the absolute angle of the intake.
    frc::DutyCycleEncoder thrubore_enc { 0 };

    /// @brief The PID controller for the deploy motor.
    frc::PIDController deploy_controller {
        constants::DEPLOY_P,
        constants::DEPLOY_I,
        constants::DEPLOY_D
    };

    frc::ArmFeedforward deploy_ff {0_V, 0_V, 0_V / 1_deg_per_s, 0_V / 1_deg_per_s_sq};

    frc::TrapezoidProfile<units::degrees>::Constraints deploy_constraints {
        0_deg_per_s,
        0_deg_per_s_sq
    };

    frc::TrapezoidProfile<units::degrees>::State deploy_goal;

    frc::TrapezoidProfile<units::degrees>::State deploy_setpoint;

    /// @brief The roller motor. 
    ctre::phoenix6::hardware::TalonFX roller_motor {
        constants::ROLLER_ID,
    };

    units::volt_t roller_voltage = 0_V;

    frc2::sysid::SysIdRoutine sysid {
        frc2::sysid::Config {0.25_V / 1_s, 4_V, std::nullopt, std::nullopt},
        frc2::sysid::Mechanism {
            [this](units::volt_t volts) {
                deploy_motor.SetVoltage(volts);
            },
            [this](auto log) {
                log->Motor("intake-deploy-motor")
                    .voltage(get_deploy_volts())
                    .velocity(units::turns_per_second_t{get_angle_vel()})
                    .position(units::turn_t{get_angle()});
            },
            this
        }
    };

    std::optional<frc2::CommandPtr> sysid_command;
}; // class intake

} // namespace intake

} // namespace subsystems