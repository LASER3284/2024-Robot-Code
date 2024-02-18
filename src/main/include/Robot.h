// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/SmartDashboard/SendableChooser.h>
#include "subsystems/drive.h"
#include "subsystems/uselessthing.h"
#include "subsystems/shooter.h"

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

    enum SysIdChooser {
        QsFwd = 0,
        QsRev,
        DynFwd,
        DynRev,
        None
    };

private:
    std::shared_ptr<frc::XboxController> chassis_controller = std::make_shared<frc::XboxController>(0);
    subsystems::drive::Drivetrain drive {chassis_controller};
    subsystems::useless::Useless happy_face{};
    subsystems::shooter::Shooter shooter{};

    frc2::CommandPtr auto_cmd = frc2::cmd::None();

    frc::SendableChooser<int> sysid_chooser;

    frc::SendableChooser<std::string> auto_chooser;
};
