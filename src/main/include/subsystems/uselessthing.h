#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

namespace subsystems {

namespace useless {

/// @brief A useless mechanism that acts as a counter for checking auto stuff.
/// Eventually we'll do some real mechanisms, but we don't have them built yet
/// and this is useful for debugging anyway.
class Useless {
public:
    /// @brief Update NT4 data.
    void tick() {
        frc::SmartDashboard::PutNumber("Useless_counter", counter_thing);
    }

    /// @brief Increments the internal counter by amount `thing`.
    /// @param thing Amount to increment by.
    void do_count(int thing) {
        counter_thing += thing;
    }

    /// @brief Resets the internal counter to zero.
    void reset_count() {
        counter_thing = 0;
    }

    /// @brief Returns the value of the interal counter.
    /// @return Some integer value based on the counter value.
    int get_count() {
        return counter_thing;
    }

    frc2::CommandPtr add_one() {
        return frc2::cmd::RunOnce([this]() {do_count(1);});
    }

private:
    int counter_thing = 0;
};

}

}