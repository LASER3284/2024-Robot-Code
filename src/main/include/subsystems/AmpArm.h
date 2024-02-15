#pragma once

#include "amparm/extension.h"
#include "amparm/roller.h"
#include "amparm/shoulder.h"

#include <rev/CANSparkMax.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/ElevatorFeedforward.h>

namespace subsystems {

namespace amparm {

namespace constants {
    enum AmpArmSubmechs {
        ShoulderMech,
        ExtensionMech
    };

    enum States {
        Stopped = 0,
        Feed,
        ReverseFeed,
        AmpScore,
        TrapScore
    };

    constexpr units::degree_t DOWN_ANGLE = 0_deg;
    constexpr units::inch_t DOWN_EXTENSION = 0_in;
    constexpr units::degree_t AMPSCORE_ANGLE = 90_deg;
    constexpr units::inch_t AMPSCORE_EXTENSION = 6_in;
    constexpr units::degree_t TRAPSCORE_ANGLE = 90_deg;
    constexpr units::inch_t TRAPSCORE_EXTESNION = 20_in;
}

class AmpArm {
public:
    void init();

    void tick();

    void update_nt();

    void run_sysid(int, constants::AmpArmSubmechs);

    void cancel_sysid();

    void activate(constants::States);

    bool has_piece() { return roller.has_piece(); }

    bool in_place() { return shoulder.in_place() && extension.in_place();}

private:
    constants::States state;
    Roller roller;
    Shoulder shoulder;
    Extension extension;
};

}

}