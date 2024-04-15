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
#include "subsystems/arm.h"
#include <frc2/command/button/CommandXboxController.h>

#include "lib/tracktracker/TrackTracker.h"

#include "commands/SmarterShooter.h"

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

    frc2::CommandPtr amp_stop() {
        return frc2::cmd::Sequence(shooter.down(), amp_arm.stop(), shooter.stable());
    }

    frc2::CommandPtr amp_stop_l = amp_stop();

    frc2::CommandPtr amp_spit = amp_arm.spit();

    frc2::CommandPtr reverse_intake() {
        return frc2::cmd::Parallel(
            intake.reverse(),
            amp_arm.reverse()
        ).WithTimeout(6.5_s);
    }

    frc2::CommandPtr intake_cmd() {
        return frc2::cmd::Parallel(
            intake.intake(),
            amp_arm.RunEnd(
                [this]() {
                    amp_arm.activate(subsystems::amparm::constants::States::Feed);
                },
                [this]() {
                    amp_arm.activate(subsystems::amparm::constants::States::Stopped);
                }
            ).Until([this]() {
                return shooter.has_piece();
            }),
            shooter.feed()
        ).Until([this]() {
            return shooter.has_piece();
        });
    }

    frc2::CommandPtr intake_ignore() {
        return frc2::cmd::Parallel(
            intake.intake(),
            amp_arm.RunEnd(
                [this]() {
                    amp_arm.activate(subsystems::amparm::constants::States::Feed);
                },
                [this]() {
                    amp_arm.activate(subsystems::amparm::constants::States::Stopped);
                }).Until([this]() {
                return shooter.has_piece();
            }),
            shooter.feed()
        );
    }

    frc2::CommandPtr intake_continuous() {
        return frc2::cmd::Parallel(
            intake.intake_continuous(),
            amp_arm.Run([this]() {
                amp_arm.activate(subsystems::amparm::constants::States::Feed);
            }).Until([this]() {
                return shooter.has_piece();
            }),
            shooter.feed()
        ).WithTimeout(3_s);
    }

    frc2::CommandPtr intake_continuous_amp() {
        return frc2::cmd::Parallel(
            intake.intake_continuous(),
            amp_arm.Run([this]() {
                amp_arm.activate(subsystems::amparm::constants::States::Intake);
            }).Until([this]() {
                return amp_arm.has_piece();
            })
        ).WithTimeout(2.5_s);
    }

    frc2::CommandPtr auto_shoot() {
        return frc2::cmd::Sequence(
            intake.RunOnce([this]() {
                intake.activate(subsystems::intake::constants::DeployStates::NOSPIN);
            }),
            shooter.Run([this]() {
                shooter.activate(subsystems::shooter::constants::ShooterStates::TrackingIdle);
            }).Until([this]() {
                return shooter.in_place();
            }).BeforeStarting([this]() {
                //drive.reset_pose_to_vision();
                shooter.activate(subsystems::shooter::constants::ShooterStates::TrackingIdle);
            }).WithTimeout(1.75_s),
            shooter.RunOnce([this]() {
                shooter.activate(subsystems::shooter::constants::ShooterStates::TrackingIdle);
            }),
            shooter.force_score(),
            shooter.stable()
        );
    }

    frc2::CommandPtr spit() {
        return shooter.spit();
    }
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
            shooter.force_score(),
            shooter.stable()
        );
    
    frc2::CommandPtr tele_shoot = shooter.score();
   // frc2::CommandPtr tele_sub_score = shooter.sub_score();
    frc2::CommandPtr tele_feed() { return frc2::cmd::Parallel(shooter.feed(), amp_arm.feed()).Until([this]() { return shooter.has_piece(); }); }
    frc2::CommandPtr tele_track = shooter.track();
    frc2::CommandPtr tele_creamy_shot = shooter.creamy_shot();
    frc2::CommandPtr shooter_stable = shooter.stable();

    frc2::CommandPtr reverse_feed = frc2::cmd::Race(
            shooter.reverse_feed(),
            amp_arm.reverse()
        );

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
        Climi,
        AmpArmExtension
    };

private:
    tracktracker::TrackTracker tracker {
        [this](frc::ChassisSpeeds s) { drive.drive_robo(s); },
        [this]() { return drive.get_pose(); },
        {1, 0, 0},
        {1, 0, 0},
        {2, 0, 0},
        {{5_fps, 10_fps_sq}, {135_deg_per_s, 180_deg_per_s_sq}},
        []() { return frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed; },
        {&drive}
    };

    std::shared_ptr<frc2::CommandXboxController> chassis_controller = std::make_shared<frc2::CommandXboxController>(0);
    frc2::CommandXboxController aux_controller {1};
    subsystems::drive::Drivetrain drive {chassis_controller};
    subsystems::amparm::AmpArm amp_arm{};
    subsystems::climi::Climi climi{};

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
