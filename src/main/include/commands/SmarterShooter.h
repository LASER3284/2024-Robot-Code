#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <map>
#include <wpi/interpolating_map.h>

#include "subsystems/shooter.h"
#include "subsystems/drive.h"
#include <units/time.h>
#include <units/length.h>

namespace commands {

const std::map<units::inch_t, units::second_t> TIMETABLE_MEASUREMENTS = {
    {100_in, 0.75_s}
};

class SmarterShooterCommand : public frc2::CommandHelper<frc2::Command, SmarterShooterCommand> {
public:
    SmarterShooterCommand(subsystems::shooter::Shooter* shooter, subsystems::drive::Drivetrain* drivetrain);

    void Initialize() override;

    void Execute() override;

    bool IsFinished() override;

    void End(bool interrupted) override;

private:
    subsystems::shooter::Shooter* shooter;
    subsystems::drive::Drivetrain* drivetrain;

    wpi::interpolating_map<units::inch_t, units::second_t> timetable;
};

}
