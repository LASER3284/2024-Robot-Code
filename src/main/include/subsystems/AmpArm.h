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

#include <frc2/command/Commands.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

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
        Intake,
        ReverseFeed,
        AmpScore,
        TrapScore,
        Spit
    };

    constexpr units::degree_t DOWN_ANGLE = -20_deg;
    constexpr units::inch_t DOWN_EXTENSION = 0_in;
    constexpr units::degree_t AMPSCORE_ANGLE = 91_deg;
    constexpr units::inch_t AMPSCORE_EXTENSION = 14.8_in;
    constexpr units::degree_t TRAPSCORE_ANGLE = 90_deg;
    constexpr units::inch_t TRAPSCORE_EXTESNION = 20_in;
}

class AmpArm : public frc2::SubsystemBase {
public:
    /// @brief Initializes submechanisms. Should be called in RobotInit.
    void init();

    /// @brief Updates submechanism voltage outputs. Should be called in Teleop
    /// and Auto Periodics.
    void tick();

    /// @brief Sets up the shoulder to not try to go past a certain goal on
    /// startup.
    /// @todo Call this for extension as well.
    void reset();

    /// @brief Updates the network tables values for each submechanism,
    /// including velocity. Should be called in RobotPeriodic.
    void update_nt();

    /// @brief Schedules the SysID command for the specified submechanism, if
    /// it's not already scheduled.
    void run_sysid(int, constants::AmpArmSubmechs);

    /// @brief Cancels all submechanism SysID routines.
    void cancel_sysid();

    /// @brief Updates the goal state.
    /// @see score
    /// @see intake
    /// @see reverse_feed
    void activate(constants::States);

    bool has_piece() { return roller.has_piece(); }

    /// @brief Returns true when both the shoulder and the extension have
    /// reached their goals.
    /// @return See brief :)
    bool in_place() { return shoulder.in_place() && extension.in_place();}

    /// @brief Runs a score routine, assuming a note is present.
    /// @return The corresponding CommandPtr that will run the routine.
    frc2::CommandPtr score() {
        return frc2::cmd::Sequence(
            this->Run([this]() {
                activate(constants::States::AmpScore);
            }).Until([this]() {
                return in_place();
            }).BeforeStarting([this]() {
                activate(constants::States::AmpScore);
            }),
            this->Run([this]() {
                activate(constants::States::AmpScore);
            }).Until([this]() {
                return in_place();
            }).BeforeStarting([this]() {
                activate(constants::States::AmpScore);
            }),
            frc2::cmd::Sequence(
                this->RunOnce([this]() {
                    activate(constants::States::Spit);
                }),
                frc2::cmd::Wait(1_s)
            ),
            this->Run([this]() {
                activate(constants::States::Stopped);
            }).Unless([this]() {
                return in_place();
            }).BeforeStarting([this]() {
                activate(constants::States::Stopped);
            })
        );
    }

    frc2::CommandPtr stop() {
        return this->Run([this]() {
            activate(constants::States::Stopped);
        }).WithTimeout(1.75_s);
    }

    /// @brief Spins the roller for intake on startup, stops when interrupted.
    frc2::CommandPtr intake() {
        return this->RunEnd(
            [this]() {
                activate(constants::States::Intake);
            },
            [this]() {
                activate(constants::States::Stopped);
            }
        );
    }

    frc2::CommandPtr intake_continuous() {
        return this->Run([this]() {
            activate(constants::States::Intake);
        }).Until([this]() {
            return has_piece();
        });
    }

    /// @brief Spins the roller for intake on startup, stops when interrupted.
    frc2::CommandPtr feed() {
        return frc2::cmd::Sequence(
            this->Run(
                [this]() {
                    activate(constants::States::Feed);
                }
            ).Until([this]() {
                return !has_piece();
            }).BeforeStarting([this]() {
                activate(constants::States::Feed);
            }),
            frc2::cmd::Wait(0.5_s),
            this->RunOnce([this]() {
                activate(constants::States::Stopped);
            })
        );
    }

    /// @brief Spins the roller for reverse feed on startup, stops when
    /// interrupted.
    frc2::CommandPtr reverse_feed() {
        return this->StartEnd(
            [this]() {
                activate(constants::States::ReverseFeed);
            },
            [this]() {
                activate(constants::States::Stopped);
            }
        );
    }

private:
    constants::States state;
    Roller roller;
    Shoulder shoulder;
    Extension extension;
};

}

}
