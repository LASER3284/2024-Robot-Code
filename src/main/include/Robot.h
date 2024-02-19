// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "subsystems/drive.h"
#include "subsystems/uselessthing.h"
#include "subsystems/intake.h"
#include "subsystems/AmpArm.h"
#include <frc2/command/button/CommandXboxController.h>

class Robot : public frc::TimedRobot {
public:
    void RobotInit() override;
    void RobotPeriodic() override;

    void AutonomousInit() override;
    void AutonomousPeriodic() override;

    void TeleopInit() override;
    void TeleopPeriodic() override;

    void DisabledInit() override;
    void DisabledPeriodic() override;

    void TestInit() override;
    void TestPeriodic() override;

    void SimulationInit() override;
    void SimulationPeriodic() override;

    frc2::CommandPtr intake_cmd() {
        return frc2::cmd::Parallel(
            intake.intake(),
            amp_arm.intake()
        );
    }

    enum SysIdChooser {
        QsFwd = 0,
        QsRev,
        DynFwd,
        DynRev,
        None
    };

    enum MechanismChooser {
        MechNone = 0,
        Drivetrain,
        AmpArmShoulder,
        AmpArmExtension
    };

private:
    std::shared_ptr<frc2::CommandXboxController> chassis_controller = std::make_shared<frc2::CommandXboxController>(0);
    frc2::CommandXboxController aux_controller {1};
    subsystems::drive::Drivetrain drive {chassis_controller};
    subsystems::amparm::AmpArm amp_arm{};

    subsystems::useless::Useless happy_face{};
    subsystems::intake::Intake intake{};

    frc2::CommandPtr auto_cmd = frc2::cmd::None();

    frc::SendableChooser<int> sysid_chooser;
    frc::SendableChooser<int> mech_chooser;
    int selected_mech = MechanismChooser::MechNone;

    int sysid_routine = SysIdChooser::None;

    frc::SendableChooser<std::string> auto_chooser;
};
