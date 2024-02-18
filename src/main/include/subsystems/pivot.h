#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/controller/PIDController.h>
#include <frc/DutyCycleEncoder.h>


namespace subsystems {

namespace pivot {

namespace constants {
    /// @brief this is the id of the pivot motor
    constexpr int PIVOT_ID = 94;
}

class pivot {
public:
    /// @brief set the point for the angle of the turn
    void set_angle(units::degree_t);
    units::degree_t get_angle();
    /// @brief keeps the idle position
    void idle_angle();
    /// @brief checks if at angle
    bool at_angle();
private:
    ///@brief this will be the angle to pivot is currently at 
    int at_pivot = 0;
    ///@brief this will be the angle the pivot wants to go to
    int set_pivot = 0;
    ///@brief this is the value of the voltage/power needed to keep the angle
    int hold_pivot = 0;
    /// @brief this is the absolute encoder
    frc::DutyCycleEncoder pivot_encoder { 0 };

};

}

}
