#include <units/length.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <memory>
#include <string>
#include <frc/XboxController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/Timer.h>

#include <rev/CANSparkMax.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/AnalogEncoder.h>

namespace subsystems {
    namespace AmpArm {
        namespace constants {
            /// @brief The CAN ID for rotating the AmpShoulder
            constexpr int AmpShoulderCANID = 0;
            
            /// @brief The CAN ID for the AmpExtension
            constexpr int AmpExtensionCANID = 0;
            
            /// @brief The CAN ID for the AmpShot
            constexpr int AmpShotCANID = 0;

            /// @brief PWM Slot ID for the encoder to be used to measure the angle of the arm on the shoulder.
            constexpr int AmpShoulderPortID = 0;

            /// @brief Gear Ratio of the Amp Extension
            constexpr double Extension_Ratio = 14.198;

            /// @brief 
            constexpr double Extension_Diameter = 1.88;

            /// @brief Gear Ratio of the amp rotation
            constexpr double rotationRatio = (1 / 66.96);
        };

        class AmpArm : public frc2::SubsystemBase {
            public:
                AmpArm();

                void tick();

                void setRotationalGoal(units::degree_t);

                units::degree_t getShoulderRotation();

                units::degrees_per_second_t getShoulderVelocity();

                units::meter_t getExtensionPosition();
            
                units::meters_per_second_t getExtensionVelocity();
            private:
                units::degree_t shoulderRotationGoal = 0_deg;
                units::meter_t armExtensionGoal = 0_m;

                ctre::phoenix6::hardware::TalonFX extensionMotor { constants::AmpExtensionCANID };
                frc::DutyCycleEncoder thruboreEnc { 0 };
                
                rev::CANSparkMax shootMotor { constants::AmpShotCANID, rev::CANSparkMaxLowLevel::MotorType::kBrushed};
                
                /// @brief The main motor for driving the rotation of the shoulder
                ctre::phoenix6::hardware::TalonFX shoulderMotor { constants::AmpShoulderCANID };

                /// @brief The absolute encoder used for locating the shoulder
                frc::AnalogEncoder encoder { constants::AmpShoulderPortID };

                /// I haven't understand below yet
                frc::ProfiledPIDController<units::degrees> shoulderController {
                    0, 0, 0,
                    frc::TrapezoidProfile<units::degrees>::Constraints { 0_deg, 0_deg_per_s_sq }
                };
                frc::ArmFeedforward shoulderFeedforward { 0_V, 0_V, 0_V / 1_deg_per_s, 0_V / 1_deg_per_s_sq };
                units::degrees_per_second_t lastShoulderVelocity = 0_deg_per_s;

                frc::ProfiledPIDController<units::meters> extensionController {
                    0, 0, 0,
                    frc::TrapezoidProfile<units::meters>::Constraints { 0_mps, 0_mps_sq }
                };
                frc::ElevatorFeedforward extensionFeedforward { 0_V, 0_V, 0_V / 1_fps, 0_V / 1_fps_sq };
                units::meters_per_second_t lastExtensionVelocity = 0_mps;

                units::second_t lastTime = frc::Timer::GetFPGATimestamp();

                frc2::sysid::SysIdRoutine rotationSysId {
                    frc2::sysid::Config { std::nullopt, std::nullopt, std::nullopt, std::nullopt },
                    frc2::sysid::Mechanism {
                        [this](units::volt_t volts) {
                            shoulderMotor.SetVoltage(volts);
                        },
                        [this](auto log) {
                            log->Motor("ampShoulder")
                                .voltage(shoulderMotor.Get() * shoulderMotor.GetMotorVoltage().GetValue())
                                .velocity(shoulderMotor.GetVelocity().GetValue())
                                .position(units::turn_t { getShoulderRotation() });
                        },
                        this
                    }
                };

                frc2::sysid::SysIdRoutine extensionSysId {
                    frc2::sysid::Config { std::nullopt, std::nullopt, std::nullopt, std::nullopt },
                    frc2::sysid::Mechanism {
                        [this](units::volt_t volts) {
                            extensionMotor.SetVoltage(volts);
                        },
                        [this](auto log) {
                            log->Motor("ampExtension")
                                .voltage(extensionMotor.Get() * extensionMotor.GetMotorVoltage().GetValue())
                                .velocity(extensionMotor.GetVelocity().GetValue())
                                .position( getExtensionPosition() );
                        },
                        this
                    }
                };
        };
    } // class AmpArm
}