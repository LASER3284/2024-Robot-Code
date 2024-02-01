#include <frc/trajectory/TrapezoidProfile.h>
#include <units/velocity.h>
#pragma once

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
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>

namespace subsystems {

namespace intake {

namespace constants {
    /// @brief The CAN ID for the roller motor.
    constexpr int ROLLER_ID = ;
    /// @brief The CAN ID for the deploy motor.
    constexpr int DEPLOY_ID = ;

    /// @brief Angle offset for the thru-bore (this is subtracted from the reported angle).
    constexpr units::degree_t THRUBORE_ANGLE_OFFSET = -0_deg;
    
    // NOTE: The PID values for the deploy motor are set with max control effort from SysID at 0V.

    /// @brief The proportional gain for the deploy controller.
    constexpr double DEPLOY_P = 0.0;
    /// @brief The integral gain for the deploy controller.
    constexpr double DEPLOY_I = 0.0;
    /// @brief The derivative gain for the deploy controller.
    constexpr double DEPLOY_D = 0.0;

    /// @brief The statically applied voltage for the deploy motor.
    constexpr auto DEPLOY_KS = 0.0_V;
    /// @brief The applied voltage per velocity unit for the deploy motor.
    constexpr auto DEPLOY_KV = 0.0_V / 1_deg_per_s;
    /// @brief The applied voltage per acceleration unit for the deploy motor. 
    constexpr auto DEPLOY_KA = 0.0_V / 1_deg_per_s_sq;

    /// @brief The gravitationally applied voltage for the deploy motor.
    constexpr auto DEPLOY_KG = 0.0_V;
    /// @brief The point at which the intake is horizontal.
    constexpr units::degree_t DEPLOY_KG_MAX_ANGLE = -0.0_deg;

    /// @brief The statically applied voltage for the roller motor.
    constexpr auto ROLLER_KS = 0.0_V;
    /// @brief The applied voltage per velocity unit for the roller motor.
    constexpr auto ROLLER_KV = 0.0_V / 1_deg_per_s;
    /// @brief The applied voltage per acceleration unit for the roller motor.
    constexpr auto ROLLER_KA = 0.0_V / 1_deg_per_s_sq;

    /// @brief The roller speed in RPM.
    constexpr units::revolutions_per_minute_t ROLLER_SETPOINT = -0.0_rpm;
    /// @brief The roller intake speed in RPM.
    constexpr units::revolutions_per_minute_t ROLLER_INTAKE_SETPOINT = 0.0_rpm;

    constexpr units::degree_t UPPER_LIMIT = 0_deg;
    constexpr units::degree_t SHOOTING_POS = 0_deg;
    constexpr units::degree_t LOWER_LIMIT = 0_deg;

    /// @brief This is a map of GridHeights to degree values where the intake should position itself.
    const std::map<
        ::constants::FieldConstants::GridHeights,
        units::degree_t
    >   ANGLE_GRID_MAP = {
        { ::constants::FieldConstants::GridHeights::eUp, UPPER_LIMIT },
        { ::constants::FieldConstants::GridHeights::eMid, SHOOTING_POS },
        { ::constants::FieldConstants::GridHeights::eGround, LOWER_LIMIT },
        { ::constants::FieldConstants::GridHeights::eIntake, LOWER_LIMIT },
        { ::constants::FieldConstants::GridHeights::eGroundSpit, LOWER_LIMIT },
        { ::constants::FieldConstants::GridHeights::eStopped, UPPER_LIMIT },
    };

    constexpr units::second_t ROLLER_DELAY = 0_ms;
    constexpr units::revolutions_per_minute_t ROLLER_DEADBAND = 0_rpm;
} // namespace constants

class intake {
public:
    /// @brief Initializes the motors and ensures break mode on the NEO.
    void init();

    /// @brief Updates SmartDashboard values, should be called in RobotPeriodic().
    void tick();

    /// @brief A boolean for whether the intake was last deployed or retracted.
    /// @return True if deployed, false otherwise
    bool is_deployed() {
        return is_deployed;
    }

    /// @brief A boolean for whether or not the intake has a game element
    /// @return True if the intake has a game element, false otherwise
    bool has_element() {
        return abs(_get_roller_avel().value()) < constants::ROLLER_DEADBAND.value();
    }

    units::degree_t get_angle {
        units::degree_t cw_plus = units::degree_t {thrubore_enc.get()};

        units::degree_t ccw_plus = 0_deg - cw_plus;

        return ccw_plus + constants::THRUBORE_ANGLE_OFFSET;
    }
private:
    /// @brief Stops the roller motors completely.
    void stop_rollers();

    /// @brief Sets the voltage of the roller motor. 
    /// @param volts The voltage at which the roller motor is set. 
    void set_roller(units::volt_t volts) 
    /// @brief Sets the voltage of the deploy motor.
    /// @param volts The voltage that should be applied to the deploy motor. 
    void set_deploy(units::volt_t volts);

    /// @brief Has the side effect of setting the deploy_setpoint and deploy_goal to the specified position.
    /// @param angle The desired position.
    /// @warning Bounds checking is not done for this function! Be cautious of its side effects.
    void set_deploy_goal(units::degree_t angle);
    
    /// @brief The deploy motor.
    rev::CANSparkMax deploy_motor {
        constants::DEPLOY_ID,
        rev::CANSparkMaxLowLevel::MotorType::BRUSHLESS
    };

    /// @brief The encoder that measures the absolute angle of the intake.
    frc::DutyCycleEncoder thrubore_enc { 0 };

    /// @brief The PID controller for the deploy motor.
    frc2::PIDController deploy_controller {
        constants::DEPLOY_P,
        constants::DEPLOY_I,
        constants::DEPLOY_D
    };

    frc::TrapezoidProfile<units::degrees>::Constraints deploy_constraints {
        0_deg_per_s,
        0_deg_per_s_sq
    };

    frc::TrapezoidProfile<units::degrees>::State deploy_goal;

    frc::TrapezoidProfile<units::degrees>::State deploy_setpoint;

    /// @brief The roller motor. 
    rev::CANSparkMax roller_motor {
        constants::ROLLER_ID,
        rev::CANSparkMaxLowLevel::MotorType::BRUSHLESS
    };

    /// @brief A boolean for whether or not the intake is deployed. 
    bool is_deployed = false;
} // class intake

} // namespace intake

} // namespace subsystems