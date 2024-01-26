#pragma once

#include <frc/AddressableLED.h>
#include <frc/util/Color.h>

namespace subsystems {

namespace lights {

    namespace constants {

        /// @brief the amount of LEDs on the strip
        constexpr int STRIP_LENGTH = 0; // not actually 0 lol

    } // constants

    class LightHandler {
        LightHandler() {
            // ledstrip.SetLength(Constants::stripLength);

        }
    }; // LightHandler

} // lights

} // subsystems