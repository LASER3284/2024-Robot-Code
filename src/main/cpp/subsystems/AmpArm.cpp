#include "subsystems/AmpArm.h"

subsystems::arm::Arm::Arm() {
    AmpShotMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    set_position_goal1(get_position1());
    set_position_goal2(get_position2());
}

void intake::intake::tick() {
    frc::SmartDashboard::PutNumber("amp_shoulder_rotation", GetShoulderRotation().value());
    frc::SmartDashboard::PutNumber("amp_extention_position", GetExtentionPosition().value());
    frc::SmartDashboard::PutNumber("amp_shoulder_velocity", GetShoulderVelocity().value());
    frc::SmartDashboard::PutNumber("amp_extention_velocity", GetExtentionVelocity().value());
}

units::degree_t subsystems::AmpArm::GetShoulderRotation() {
    units::degree_t shoulder_deg = thruboreEnc.Get();
    return Shoulder_deg;
}

units::meter_t subsystems::AmpArm::GetExtentionPosition() {
    auto value = (AmpExtentionEncoder.GetPosition());
    return value;
}

units::inch_per_second_t subsystems::AmpArm::GetExtentionVelocity() {
    const units::inch_per_second_t velocity = units::inch_per_second_t
        (((AmpExtensionEncoder.GetVelocity() / Constants::Extention_Ratio) * Constants::Extention_Diameter/*3.14?*/).value()
    );
    return velocity;
}

units::degrees_per_second_t subsystems::AmpArm::GetRotationVelocity() {
    const units::degree_per_second_t velocity = units::inch_per_second_t
        ((AmpExtensionEncoder.GetVelocity())
    );
    return velocity;
}

void subsystems::AmpArm::SetRotationGoal(units::degree_t shoulder_goal) {
    if (shoulder_goal > 0_degree) {shoulder_goal = 0_degree; } ///put value
    ///controler
    AmpShoulderMotor = { shoulder_goal, 0_fps };
}

void subsystems::AmpArm::SetPositionAmpShot(unit::inch_t extention_ampshot){
    if (extention_ampshot > 0_in) {extention_ampshot = 0_in; } ///put value
    positionController.SetSetpoint(extention_ampshot.value());
    AmpShoulderMotor = { extention_ampshot, 0_fps };
}

void subsystems::AmpArm::SetPositionTrap(unit::inch_t extention_trapshot){
    if (extention_trapshot > 0_in) {extention_trapshot = 0_in; } ///put value
    positionController.SetSetpoint(extention_trap.value());
    AmpShoulderMotor = { extention_trapshot, 0_fps };
}

void subsystems::AmpArm::AmpExtention(){
    frc::SmartDashboard::PutNumber("amp_extension_m", GetExtentionPosition().value());
    if(AmpExtentionManualPercentage != 0.0){
        if (GetPosition() < 0.05m && manualPercentage == 0.0) {
                AmpExtensionMotor.Set(0.0);
        } else {
            frc::SmartDashboard::PutNumber("AmpArm_extention_volts", AmpExtensionMotor.GetAppliedOutput() * 12);
            AmpExtensionMotor.SetVoltage(units::volt_t { 12 * manualPercentage });
        }
            
    }else {
        frc::TrapezoidProfile<units::meters> AmpExtensionProfile { 
            constraints, 
            extensionGoal,
            extensionSetpoint,
        };

        extensionSetpoint = AmpExtensionProfile.Calculate(0_ms);
        frc::SmartDashboard::PutNumber("extensionSetpoint_vel", extensionSetpoint.velocity.value());
        frc::SmartDashboard::PutNumber("extensionSetpoint_pos", extensionSetpoint.position.value());
        const auto output_voltage = feedforward.Calculate(extensionSetpoint.velocity);
        frc::SmartDashboard::PutNumber("extensionF_v", output_voltage.value());
        AmpExtensionMotor.SetVoltage(
            units::volt_t(positionController.Calculate(GetExtentionPosition().value(), extensionSetpoint.position.value()))
            + output_voltage
        );
    }
}

void subsystems::AmpArm::AmpShoulder() {
    /// I'm going to add limit switch that override the encoder program
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

void subsystems::AmpArm::Send() {
    ///if (NotesDetector) {
    ///DC brushed SPARK MAX 
    ///}else{
    ///wait
    ///stop
    ///}
}

void subsystems::AmpArm::AmpIntake() {
    ///They are going to change the LimitSwitch to a sensor
    frc::DigitalInput input{0};
    auto NotesDetector = (input.Get());

    if (!NotesDetector){
        AmpshotMotor.Set(1.00 * AmpIntakePower);  
    }else {
        AmpshotMotor.Set(0.00 * AmpIntakePower); 
    }
}

void subsystems::AmpArm::SetPositionAmp() {
    ///put values to "subsystems::AmpArm::AmpShoulder" and "subsystems::AmpArm::AmpExtention"
    
}

void subsystems::AmpArm::Shot(){
    AmpshotMotor.Set(0.00 * AmpshotPower);
}

void subsystems::AmpArm::SetPositionTrapShot(){
    ///put values to "subsystems::AmpArm::AmpShoulder" and "subsystems::AmpArm::AmpExtention"
}
    
void subsystems::AmpArm::Reset() {
    ///put values to "subsystems::AmpArm::AmpShoulder" and "subsystems::AmpArm::AmpExtention"
}
