#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include "turret.h"
#include "flywheel.h"
#include "hood.h"
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>

namespace subsystems {

namespace shooter {

namespace constants {
    constexpr units::feet_per_second_squared_t GRAVITY = 32.175_fps_sq;

    const frc::Translation2d GOAL_POSITION {12_ft, 10_ft};

    constexpr units::foot_t GOAL_HEIGHT_DELTA = 5_ft;

    static int sgn(double val) {
        return val >= 0 ? 1 : -1;
    }
}

class Shooter {
public:
    Shooter(
        std::shared_ptr<turret::Turret>,
        std::shared_ptr<flywheel::Flywheel>,
        std::shared_ptr<hood::Hood>
    );

    /// @brief Updates the shooter. 
    /// @param pose The heading of the robot's velocity relative to the shortest
    /// line between the target and the robot. This pose also contains the robot
    /// position.
    /// @param robot_velocity The velocity of te drive train in ft/s.
    void tick(frc::Pose2d, units::feet_per_second_t);

private:
    std::shared_ptr<turret::Turret> turret_control;
    std::shared_ptr<flywheel::Flywheel> flywheel_control;
    std::shared_ptr<hood::Hood> hood_control;
};

}

}
