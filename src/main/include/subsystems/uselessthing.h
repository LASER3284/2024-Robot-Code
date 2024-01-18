#pragma once

#include <frc/smartdashboard/SmartDashboard.h>

namespace subsystems {

namespace useless {

class Useless {
public:
    void tick() {
        if (++counter_thing > 50) {
            counter_thing = 0;
        }
        frc::SmartDashboard::PutNumber("Useless_counter", counter_thing);
    }

    void extra_count(int thing) {
        counter_thing += thing;
    }

private:
    int counter_thing = 0;
};

}

}