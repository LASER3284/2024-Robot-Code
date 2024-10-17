/*
 * Many thanks to FRC Team 1706 "Ratchet Rockers" for posting their 2022 robot
 * code. Much of this command is inspired on their code, which was written in
 * Java.
 */

#include "commands/SmarterShooter.h"
#include "frc/DriverStation.h"
#include "frc/MathUtil.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "subsystems/shooter.h"
#include "units/angle.h"
#include "units/length.h"
#include "units/math.h"
#include "units/time.h"

using namespace subsystems;

commands::SmarterShooterCommand::SmarterShooterCommand(shooter::Shooter* shooter, drive::Drivetrain* drivetrain) {
    this->shooter = shooter;
    this->drivetrain = drivetrain;

    for (auto const &[k, v] : commands::TIMETABLE_MEASUREMENTS) {
        timetable.insert(k, v);
    }

    AddRequirements({shooter, drivetrain});
}

void commands::SmarterShooterCommand::Initialize() {
    shooter->activate(shooter::constants::Stopped);
}

void commands::SmarterShooterCommand::Execute() {
    frc::Translation2d goal = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed ? shooter::constants::GOAL_RED_POSITION : shooter::constants::GOAL_BLUE_POSITION;

    frc::ChassisSpeeds velocity = frc::ChassisSpeeds::FromRobotRelativeSpeeds(drivetrain->get_robo_speeds(), drivetrain->get_pose().Rotation());

    const frc::Translation2d rtog = goal - drivetrain->get_pose().Translation();

    const units::inch_t distance = rtog.Norm();

    units::second_t shot_time = timetable[distance];

    frc::Translation2d moving_goal {};

    for (int i = 0; i < 5; i++) {
        auto virtx = goal.X() - shot_time * velocity.vx;
        auto virty = goal.Y() - shot_time * velocity.vy;

        frc::Translation2d test_goal {virtx, virty};

        auto rtotg = test_goal - drivetrain->get_pose().Translation();

        units::second_t new_shot_time = timetable[rtotg.Norm()];

        if (units::math::abs(new_shot_time - shot_time) < 0.02_s) {
            i = 4;
        }

        if (i == 4) {
            moving_goal = test_goal;
        } else {
            shot_time = new_shot_time;
        }
    }

    // Enable Override of default shooter setup!
    shooter->activate(shooter::constants::ShooterStates::Override);

    shooter->or_flywheel_speed = shooter::constants::SHOT_VELOCITY;

    // Turret

    frc::Pose2d rpose = drivetrain->get_pose();

    frc::Translation2d moving_rtog = moving_goal - rpose.Translation();

    units::degree_t a = units::math::atan2(moving_rtog.Y(), moving_rtog.X());
    a -= rpose.Rotation().Degrees();

    a = frc::AngleModulus(a);

    shooter->or_turret_angle = a;


    units::degree_t p = units::math::atan2(shooter::constants::DELTA_Y, moving_rtog.Norm());

    if (moving_rtog.Norm() > 5_ft) {
     p += units::foot_t(pow(units::foot_t{moving_rtog.Norm()}.value(), 1.1)) * shooter::constants::PIVOT_CORRECTION;
    }

    p = frc::AngleModulus(p);

    shooter->or_pivot_angle = p;

    if (shooter->in_place() && shooter->has_piece()) {
        shooter->or_fire = true;
    }
}

bool commands::SmarterShooterCommand::IsFinished() {
    return !shooter->has_piece();
}

void commands::SmarterShooterCommand::End(bool interrupted) {
    shooter->activate(shooter::constants::ShooterStates::Stopped);

    shooter->or_fire = false;
}
