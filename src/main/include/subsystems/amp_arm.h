#include <rev/CANSparkMax.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc2/command/SubsystemBase.h>

namespace subsystems {

namespace amp_arm {  
    
namespace constants {
    /// @brief The CAN ID for rotating the amp shoulder
    static constexpr int AMP_SHOULDER_CANID = 0;
            
    /// @brief The CAN ID for the amp extension
    static constexpr int AMP_EXTENSION_CANID = 0;
            
    /// @brief The CAN ID for the amp shot
    static constexpr int AMP_SHOT_CANID = 0;

    /// @brief PWM Slot ID for the encoder to be used to measure the angle of the arm on the shoulder.
    static constexpr int AMP_SHOULDER_PORTID = 0;

    /// @brief Gear Ratio of the Amp Extention
    static constexpr double EXTENSION_RATIO = 14.198;

    /// @brief 
    static constexpr double EXTENSION_DIAMETER = 1.88;
    };

class Amp_Arm : public frc2::SubsystemBase{
public:
    amp_arm();
    
    units::meter_t get_position();

private:    
    rev::CANSparkMax extensionMotor { constants::AMP_SHOT_CANID, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
    frc::DutyCycleEncoder thrubore_enc { 0 };
            
    /// Copy
    /// @brief The main motor for driving the rotation of the shoulder
    ctre::phoenix::motorcontrol::can::WPI_TalonFX motor { Constants::AMP_SHOULDER_CANID };

    /// @brief The absolute encoder used for locating the shoulder
    frc::AnalogEncoder encoder { constants::AMP_SHOULDER_PORTID };

    /// I haven't understand below yet
    /// @brief The trapezoidal profile constraints for the arm extension
    frc::TrapezoidProfile<units::meters>::Constraints constraints { 0_mps, 0_mps_sq };

    /// @brief The current goal to rotate the shoulder to
    frc::TrapezoidProfile<units::meters>::State extension_goal;

    /// @brief The current setpoint for the shoulder rotation
    frc::TrapezoidProfile<units::meters>::State extension_setpoint;

    frc2::sysid::SysIdRoutine sysid {
    frc2::sysid::Config {std::nullopt, std::nullopt, std::nullopt, std::nullopt},
    frc2::sysid::Mechanism {
        [this](units::volt_t volts) {
            AmpShoulder.set_drive_power(volts);
            AmpExtention.set_drive_power(volts);
            AmpShot.set_drive_power(volts);
        },
        [this](auto log) {
            log->motor("AmpShoulder")
                .voltage(AmpShoulder.get_drive_power())
                .velocity(units::meters_per_second_t{AmpShoulder.get_velocity()})
                .position(AmpShoulder.get_position().distance);
            log->motor("AmpExtention")
                .voltage(AmpExtention.get_drive_power())
                .velocity(units::meters_per_second_t{AmpExtention.get_velocity()})
                .position(AmpExtention.get_position().distance);
            log->motor("AmpShot")
                .voltage(AmpShot.get_drive_power())
                .velocity(units::meters_per_second_t{AmpShot.get_velocity()})
                .position(AmpShot.get_position().distance);
        },
    }
    };    
}; // class Amp_Arm

} // namespace amp_arm

} // namespace subsystems