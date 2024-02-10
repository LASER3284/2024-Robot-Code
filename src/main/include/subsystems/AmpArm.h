#include "Constants.h"
#include <rev/CANSparkMax.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/ElevatorFeedforward.h>

namespace subsystems {
    namespace AmpArm {
        class Constants{
        public:
            /// @brief The CAN ID for rotating the AmpShoulder
            static constexpr int AmpShoulderCANID = 0;
            
            /// @brief The CAN ID for the AmpExtention
            static constexpr int AmpExtentionCANID = 0;
            
            /// @brief The CAN ID for the AmpShot
            static constexpr int AmpShotCANID = 0;

            /// @brief PWM Slot ID for the encoder to be used to measure the angle of the arm on the shoulder.
            static constexpr int AmpShoulderPortID = 0;

            /// @brief Gear Ratio of the Amp Extention
            static constexpr double Extention_Ratio = 14.198;

            /// @brief 
            static constexpr double Extention_Diameter = 1.88;

        };

        class AmpArm : public frc2::SubsystemBase{
            public:
                AmpArm();

                units::meter_t GetPosition();
                ///

            private:
            double AmpExtentionManualPercentage = 0.0;
            double AmpShoulderManualPercentage = 0.0;


            rev::CANSparkMax AmpExtensionMotor { Constants::AmpExtentionCANID, rev::CANSparkMaxLowLevel::MotorType::kBrushless};
            frc::DutyCycleEncoder thruboreEnc { 0 };
            rev::CANSparkMax AmpShotMotor { Constants::AmpShotCANID, rev::CANSparkMaxLowLevel::MotorType::kBrushed};
            

            /// Copy
            /// @brief The main motor for driving the rotation of the shoulder
            ctre::phoenix::motorcontrol::can::WPI_TalonFX motor { Constants::AmpShoulderMotorID };

            /// @brief The absolute encoder used for locating the shoulder
            frc::AnalogEncoder encoder { Constants::AmpShoulderPortID };

            /// I haven't understand below yet
            /// @brief The trapezoidal profile constraints for the arm extension
            frc::TrapezoidProfile<units::meters>::Constraints constraints { 0_mps, 0_mps_sq };

            /// @brief The current goal to rotate the shoulder to
            frc::TrapezoidProfile<units::meters>::State extensionGoal;

            /// @brief The current setpoint for the shoulder rotation
            frc::TrapezoidProfile<units::meters>::State extensionSetpoint;



            frc2::sysid::SysIdRoutine sysid {
            frc2::sysid::Config {std::nullopt, std::nullopt, std::nullopt, std::nullopt},
            frc2::sysid::Mechanism {
            [this](units::volt_t volts) {
                AmpShoulder.set_drive_power(volts);
                AmpExtention.set_drive_power(volts);
                AmpShot.set_drive_power(volts);
            },
            [this](auto log) {
                log->Motor("AmpShoulder")
                    .voltage(AmpShoulder.get_drive_power())
                    .velocity(units::meters_per_second_t{AmpShoulder.get_velocity()})
                    .position(AmpShoulder.get_position().distance);
                log->Motor("AmpExtention")
                    .voltage(AmpExtention.get_drive_power())
                    .velocity(units::meters_per_second_t{AmpExtention.get_velocity()})
                    .position(AmpExtention.get_position().distance);
                log->Motor("AmpShot")
                    .voltage(AmpShot.get_drive_power())
                    .velocity(units::meters_per_second_t{AmpShot.get_velocity()})
                    .position(AmpShot.get_position().distance);
            },
            
            
              
        }

    }
}