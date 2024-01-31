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

        public:

        LightHandler() {
            led_strip.SetLength(constants::STRIP_LENGTH);
        }

        void set_color() {
        }


        private:

        frc::AddressableLED led_strip { 0 };
    }; // LightHandler

} // lights

} // subsystems