#pragma once

#include <frc/DigitalInput.h>
#include <rev/CANSparkMax.h>

namespace subsystems {

namespace amparm {

class Roller {
public:
    void init();

    /// @brief Returns a logical not of the DIO channel on the Rio b/c it's active low (or at least should be)
    /// @return Whether the sensor is detecting a piece in range.
    bool has_piece() const { return !piece_sensor.Get(); }

    void spin() { motor.Set(0.8); }

    void stop() { motor.Set(0.0); }

    void reverse() { motor.Set(-0.8); }

private:
    frc::DigitalInput piece_sensor {0};

    rev::CANSparkMax motor {16, rev::CANSparkLowLevel::MotorType::kBrushed};

}; // class Roller

} // namespace amparm

} // namespace subsystems