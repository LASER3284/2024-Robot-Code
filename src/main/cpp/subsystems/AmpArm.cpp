#include "subsystems/AmpArm.h"

using namespace ctre::phoenix6;

subsystems::AmpArm::AmpArm::AmpArm() {
    // shootMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    // extensionMotor.SetPosition(0);
}

void subsystems::AmpArm::AmpArm::tick() {
    lastShoulderVelocity = getShoulderVelocity();
    lastExtensionVelocity = getExtensionVelocity();

    lastTime = frc::Timer::GetFPGATimestamp();   
}

units::degree_t subsystems::AmpArm::AmpArm::getShoulderRotation() {
    return thruboreEnc.Get();
}

// TODO: WRITE THIS FUNCTION :)
units::degrees_per_second_t subsystems::AmpArm::AmpArm::getShoulderVelocity() {
    return 0_deg_per_s;
}

units::meter_t subsystems::AmpArm::AmpArm::getExtensionPosition() {
    auto rot = extensionMotor.GetPosition().GetValue();
    return units::meter_t { (rot / constants::Extension_Ratio) * constants::Extension_Diameter };
}

units::meters_per_second_t subsystems::AmpArm::AmpArm::getExtensionVelocity() {
    auto rotVelocity = extensionMotor.GetVelocity().GetValue();
    return units::meters_per_second_t { 
        ((rotVelocity / constants::Extension_Ratio) * constants::Extension_Diameter)
    };
}

void subsystems::AmpArm::AmpArm::setRotationalGoal(units::degree_t shoulder_goal) {
    shoulderController.SetGoal(shoulder_goal);
}

void subsystems::AmpArm::AmpArm::SetPositionAmpShot(unit::meter_t Extension_ampshot){
    positionController.SetSetpoint(Extension_AmpShot.value());
}

void subsystems::AmpArm::SetPositionTrap(unit::meter_t Extension_trap){
    positionController.SetSetpoint(Extension_trap.value());
}

void subsystems::AmpArm::AmpArm::AmpExtension() {
    frc::SmartDashboard::PutNumber("amp_extension_m", GetExtensionPosition().value());
    if(AmpExtensionManualPercentage != 0.0) {
        ///@brief Amp Extension
        if (getExtensionPosition() < 0.05m && manualPercentage == 0.0) {
            AmpExtensionMotor.Set(0.0);
        } else {
            frc::SmartDashboard::PutNumber("extension_output", AmpExtensionMotor.GetAppliedOutput() * 12);
            AmpExtensionMotor.SetVoltage(units::volt_t { 12 * manualPercentage });
        }
    }
    else {
        auto pidResult = extensionController.Calculate(getExtensionPosition(), armExtensionGoal);
        auto acceleration = (controller.GetSetpoint().velocity - lastSpeed) / (frc2::Timer::GetFPGATimestamp() - lastTime);
        auto feedforwardCalc = extensionFeedforward.Calculate(controller.GetSetpoint().velocity, acceleration);

        extensionMotor.SetVoltage(pidResult + feedforwardCalc.Calculate(controller.GetSetpoint().velocity, acceleration));
    }
}

void subsystems::AmpArm::AmpArm::AmpShoulder() {
    if(AmpShoulderManualPercentage != 0.0) {
            AmpShoulderMotor.SetVoltage(12_V * AmpShoulderManualPercentage);
            AmpShoulderSet = { GetShoulderRotation(), 0_deg_per_s };
            AmpShoulderReset = { GetShoulderRotation(), 0_deg_per_s };
        }
    else {
        if(units::math::abs(AmpShoulderGoal.position - GetShoulderRotation()) > 0_deg) {
            frc::TrapezoidProfile<units::radians> rotationalProfile { 
                rotationalConstraints, 
                AmpShoulderGoal,
                AmpShoulderSetpoint,
                };

            AmpShoulderSetpoint = rotationalProfile.Calculate(20_ms);
            }
        else {
            AmpShoulderSetpoint = { GetSHoulderRotation(), 0_deg_per_s };
            AmpShoulderGoal = { GetShoulderRotation(), 0_deg_per_s };
        }

        frc::SmartDashboard::PutNumber("AmpShoulderSetpoint_velocity", units::degrees_per_second_t(AmpShoulderSetpoint.velocity).value());
        frc::SmartDashboard::PutNumber("AmpShoulderSetpoint_position", units::degree_t(AmpShoulderSetpoint.position).value());
        frc::SmartDashboard::PutNumber("AmpShoulderGoal_position", units::degree_t(AmpShoulderGoal.position).value());

        AdjustFeedforward(
            kinematics::Kinematics::CalculateShoulderFeedforward(AmpShoulderSetpoint.position, AmpShoulderSetpoint.velocity)
        );

        const auto control_effort_v = angleController.Calculate(
            units::radian_t(GetShoulderRotation()).value(), 
            AmpShoulderGoal.position.value()
        );

        frc::SmartDashboard::PutNumber("AmpShoulder_effort_v", control_effort_v);
        frc::SmartDashboard::PutNumber("AmpShoulder_measurement", units::radian_t(GetShoulderRotation()).value());
        frc::SmartDashboard::PutNumber("AmpShoulder_setpoint", AmpShoulderSetpoint.position.value());

        motor.SetVoltage(feedforward);
    }
}

void subsystems::AmpArm::AmpArm::Send() {
    ///if (NotesDetector) {
    ///DC brushed SPARK MAX 
    ///}else{
    ///wait
    ///stop
    ///}
}

void subsystems::AmpArm::AmpArm::AmpIntake() {
    ///They are going to change the LimitSwitch to a sensor
    frc::DigitalInput input{0};
    auto NotesDetector = (input.Get());

    if (!NotesDetector) {
        AmpshotMotor.Set(1.00 * AmpIntakePower);  
    }else {
        AmpshotMotor.Set(0.00 * AmpIntakePower); 
    }
}

void subsystems::AmpArm::AmpArm::SetPositionAmp() {
    ///put values to "subsystems::AmpArm::AmpShoulde" and "subsystems::AmpArm::AmpExtension"
    
}

void subsystems::AmpArm::AmpArm::Shot(){
    AmpshotMotor.Set(0.00 * AmpshotPower);
}

void subsystems::AmpArm::AmpArm::SetPositionTrapShot(){
    ///put values to "subsystems::AmpArm::AmpShoulde" and "subsystems::AmpArm::AmpExtension"
}
    
void subsystems::AmpArm::AmpArm::Reset() {
    ///put values to "subsystems::AmpArm::AmpShoulde" and "subsystems::AmpArm::AmpExtension"
    
}
