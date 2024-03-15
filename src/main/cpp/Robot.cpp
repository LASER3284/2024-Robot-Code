// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include "lib/tracktracker/TrackTracker.h"

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
    mech_chooser.AddOption("Shooter Pivot", MechanismChooser::ShooterPivot);
    mech_chooser.AddOption("Shooter Flywheel", MechanismChooser::ShooterFlywheel);
    mech_chooser.AddOption("Shooter Turret", MechanismChooser::ShooterTurret);
    mech_chooser.AddOption("Amp Arm Shoulder", MechanismChooser::AmpArmShoulder);
    mech_chooser.AddOption("Amp Arm Extension", MechanismChooser::AmpArmExtension);
    frc::SmartDashboard::PutData("MechChooser", &mech_chooser);

    pathplanner::NamedCommands::registerCommand("useless", happy_face.add_one());
    pathplanner::NamedCommands::registerCommand("shoot", std::move(auto_shoot()));
    pathplanner::NamedCommands::registerCommand("shoot start", std::move(auto_shoot2));
    pathplanner::NamedCommands::registerCommand("intake", std::move(intake_continuous()));
    // pathplanner::NamedCommands::registerCommand("amp", amp_score());

    std::string path = frc::filesystem::GetDeployDirectory() + "/pathplanner/autos";

    auto_chooser.SetDefaultOption("None", "None");

    for (const auto &file : std::filesystem::directory_iterator(path)) {
        std::string filename = file.path().filename().string();
        filename = filename.substr(0, filename.size() - 5);
        auto_chooser.AddOption(filename, filename);
    }

    frc::SmartDashboard::PutData("AutoChooser", &auto_chooser);

    tracktracker::TrackTracker::register_namedpoint("shoot", std::move(auto_shoot()));
    tracktracker::TrackTracker::register_namedpoint("goto1", tracker.generate({frc::Pose2d{{2.75_m, 4.1_m}, {0_deg}}, frc::ChassisSpeeds{}}));

    aux_controller.B()
        .WhileTrue(std::move(reverse_feed));
    aux_controller.X()
        .WhileTrue(intake_ignore());
    aux_controller.Y()
        .WhileTrue(drive.Run([this]() {
            drive.reset_pose_to_vision();
        }));
    aux_controller.A()
        .OnTrue(std::move(tele_track))
        .OnFalse(std::move(shooter_stable));
    aux_controller.POVUp(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()).Rising().IfHigh([this]() {
        if (!amp_prepscore.IsScheduled()) {
            amp_prepscore.Schedule();
        }
    });
    aux_controller.POVDown(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()).Rising().IfHigh([this]() {
        if (!amp_stop_l.IsScheduled()) {
            amp_stop_l.Schedule();
        }
    });
    aux_controller.RightBumper().WhileTrue(std::move(amp_spit));
    aux_controller.LeftBumper().OnTrue(std::move(tele_shoot));
    chassis_controller->LeftBumper().WhileTrue(intake_cmd());
    chassis_controller->A().WhileTrue(reverse_intake());
    chassis_controller->RightBumper().OnTrue(std::move(tele_sub_score));
    chassis_controller->Y().OnTrue(std::move(tele_creamy_shot));

    intake.init();
    shooter.init();
    amp_arm.init();
}

void Robot::RobotPeriodic() {
    drive.update_odometry();
    drive.update_nt();

    shooter.update_nt(drive.get_pose());

    amp_arm.update_nt();

    frc2::CommandScheduler::GetInstance().Run();
}

void Robot::AutonomousInit() {
    amp_arm.reset();
    auto choice = auto_chooser.GetSelected();

    if (choice != "None") {
        auto_cmd = drive.get_auto_path(choice);
    } else {
        auto_cmd = frc2::cmd::None();
    }
    auto_cmd.Schedule();
}
void Robot::AutonomousPeriodic() {
    shooter.tick();
    amp_arm.tick();
    intake.tick();
    drive.swerve_tick();
}

void Robot::TeleopInit() {
    amp_arm.reset();
}

void Robot::TeleopPeriodic() {
    amp_arm.tick();
    intake.tick();
    shooter.tick();
    drive.tick(true);
}

void Robot::DisabledInit() {
    frc2::CommandScheduler::GetInstance().CancelAll();

    drive.cancel_sysid();
    shooter.cancel_sysid();
    amp_arm.cancel_sysid();
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
        case MechanismChooser::ShooterFlywheel:
            shooter.run_sysid(sysid_chooser.GetSelected(), subsystems::shooter::constants::SubMech::Flywheel);
            break;
        case MechanismChooser::ShooterPivot:
            shooter.run_sysid(sysid_chooser.GetSelected(), subsystems::shooter::constants::SubMech::Pivot);
            break;
        case MechanismChooser::ShooterTurret:
            shooter.run_sysid(sysid_chooser.GetSelected(), subsystems::shooter::constants::SubMech::Turret);
            break;
        case MechanismChooser::AmpArmShoulder:
            amp_arm.run_sysid(sysid_chooser.GetSelected(), subsystems::amparm::constants::AmpArmSubmechs::ShoulderMech);
            break;
        case MechanismChooser::AmpArmExtension:
            amp_arm.run_sysid(sysid_chooser.GetSelected(), subsystems::amparm::constants::AmpArmSubmechs::ExtensionMech);
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
