// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "subsystems/drive.h"
#include "subsystems/uselessthing.h"
#include "subsystems/shooter.h"
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

    frc2::CommandPtr amp_prepscore = frc2::cmd::Sequence(
        shooter.down(),
        frc2::cmd::Wait(0.375_s),
        amp_arm.Run(
            [this]() {
                amp_arm.activate(subsystems::amparm::constants::States::AmpScore);
            }
        ).WithTimeout(1.2_s)
    );

    frc2::CommandPtr amp_stop = frc2::cmd::Sequence(shooter.down(), amp_arm.stop(), frc2::cmd::Wait(0.5_s), shooter.stable());

    frc2::CommandPtr amp_score = frc2::cmd::Sequence(
        shooter.down(),
        amp_arm.RunOnce(
            [this]() {
                amp_arm.activate(subsystems::amparm::constants::States::Spit);
            }
        ),
        frc2::cmd::Wait(1.33_s),
        amp_arm.stop()
    );

    frc2::CommandPtr intake_cmd =
        frc2::cmd::Parallel(
            intake.intake(),
            amp_arm.intake()
        );

    frc2::CommandPtr intake_continuous =
        frc2::cmd::Parallel(
            intake.intake_continuous(),
            amp_arm.Run([this]() {
                amp_arm.activate(subsystems::amparm::constants::States::Feed);
            }).Until([this]() {
                return shooter.has_piece();
            }),
            shooter.feed()
        );

    frc2::CommandPtr auto_shoot = 
        frc2::cmd::Sequence(
            intake.RunOnce([this]() {
                intake.activate(subsystems::intake::constants::DeployStates::NOSPIN);
            }),
            shooter.Run([this]() {
                shooter.activate(subsystems::shooter::constants::ShooterStates::TrackingIdle);
            }).Until([this]() {
                return shooter.in_place();
            }).BeforeStarting([this]() {
                shooter.activate(subsystems::shooter::constants::ShooterStates::TrackingIdle);
            }),
            shooter.RunOnce([this]() {
                shooter.activate(subsystems::shooter::constants::ShooterStates::TrackingIdle);
            }).WithTimeout(3_s),
            shooter.score(),
            shooter.stable()
        );
    frc2::CommandPtr auto_shoot2 = 
        frc2::cmd::Sequence(
            intake.RunOnce([this]() {
                intake.activate(subsystems::intake::constants::DeployStates::NOSPIN);
            }),
            shooter.Run([this]() {
                shooter.activate(subsystems::shooter::constants::ShooterStates::TrackingIdle);
            }).Until([this]() {
                return shooter.in_place();
            }).BeforeStarting([this]() {
                shooter.activate(subsystems::shooter::constants::ShooterStates::TrackingIdle);
            }).WithTimeout(3_s),
            shooter.score(),
            shooter.stable()
        );
    
    frc2::CommandPtr tele_shoot = shooter.score().WithTimeout(5_s);
    frc2::CommandPtr tele_feed = frc2::cmd::Parallel(shooter.feed(), amp_arm.feed());
    frc2::CommandPtr tele_track =
        frc2::cmd::Sequence(
            std::move(tele_feed),
            shooter.track()
        );
    frc2::CommandPtr shooter_stable = shooter.stable();

    frc2::CommandPtr reverse_feed = frc2::cmd::None();

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
        ShooterPivot,
        ShooterFlywheel,
        ShooterTurret,
        AmpArmShoulder,
        AmpArmExtension
    };

private:
    std::shared_ptr<frc2::CommandXboxController> chassis_controller = std::make_shared<frc2::CommandXboxController>(0);
    frc2::CommandXboxController aux_controller {1};
    subsystems::drive::Drivetrain drive {chassis_controller};
    subsystems::amparm::AmpArm amp_arm{};

    subsystems::useless::Useless happy_face{};
    subsystems::shooter::Shooter shooter{};
    subsystems::intake::Intake intake{};

    frc2::CommandPtr auto_cmd = frc2::cmd::None();

    frc::SendableChooser<int> sysid_chooser;
    frc::SendableChooser<int> mech_chooser;
    int selected_mech = MechanismChooser::MechNone;

    int sysid_routine = SysIdChooser::None;

    frc::SendableChooser<std::string> auto_chooser;
};
