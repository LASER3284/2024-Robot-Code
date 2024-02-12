// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include <filesystem>
#include <frc/Filesystem.h>

void Robot::RobotInit() {
    sysid_chooser.SetDefaultOption("No SysID", SysIdChooser::None);
    sysid_chooser.AddOption("Quasistatic Forward", SysIdChooser::QsFwd);
    sysid_chooser.AddOption("Quasistatic Reverse", SysIdChooser::QsRev);
    sysid_chooser.AddOption("Dynamic Forward", SysIdChooser::DynFwd);
    sysid_chooser.AddOption("Dynamic Reverse", SysIdChooser::DynRev);
    frc::SmartDashboard::PutData("SysIdChooser", &sysid_chooser);

    mech_chooser.SetDefaultOption("No Mechanism", MechanismChooser::MechNone);
    mech_chooser.AddOption("Drive Train", MechanismChooser::Drivetrain);
    frc::SmartDashboard::PutData("MechChooser", &mech_chooser);

    pathplanner::NamedCommands::registerCommand("useless", happy_face.add_one());

    std::string path = frc::filesystem::GetDeployDirectory() + "/pathplanner/autos";

    auto_chooser.SetDefaultOption("None", "None");

    for (const auto &file : std::filesystem::directory_iterator(path)) {
        std::string filename = file.path().string();
        filename = filename.substr(0, filename.size() - 6);
        auto_chooser.AddOption(filename, filename);
    }

    intake.init();
}

void Robot::RobotPeriodic() {
    drive.update_odometry();
    drive.update_nt();

    happy_face.tick();

    frc2::CommandScheduler::GetInstance().Run();
}

void Robot::AutonomousInit() {
    auto_cmd = drive.get_auto_path(auto_chooser.GetSelected());
    auto_cmd.Schedule();
}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
    drive.reset_odometry();
}

void Robot::TeleopPeriodic() {
    // snip

    if (chassis_controller->GetRightBumper()) {
        intake.activate(subsystems::intake::constants::DeployStates::SPIN);
    } else {
        intake.activate(subsystems::intake::constants::DeployStates::NOSPIN);
    }

    intake.tick();
    drive.tick(true);
}

void Robot::DisabledInit() {
    drive.cancel_sysid();
}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {
    selected_mech = mech_chooser.GetSelected();
}
void Robot::TestPeriodic() {
    switch (selected_mech) {
    case MechanismChooser::Drivetrain:
        drive.run_sysid(sysid_chooser.GetSelected());
        break;
    default:
        break;
    }
}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
