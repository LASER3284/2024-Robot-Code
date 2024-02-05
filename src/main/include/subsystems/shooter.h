
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include "turret.h"
#include "flywheel.h"
#include "pivot.h"
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>

namespace subsystems {

namespace shooter {

namespace constants {
    enum ShooterStates {
        TrackingIdle = 0,
        TrackShot,
        StableIdle,
        StableShot,
        LowStableShot,
        PrepFeeding,
        Stopped
    };

    constexpr units::feet_per_second_squared_t GRAVITY = 32.175_fps_sq;
    constexpr frc::Translation2d GOAL_POSITION { 12_ft, 10_ft};
    constexpr units::foot_t DELTA_Y = 5_ft;

    constexpr units::feet_per_second_t SHOT_VELOCITY = 72_fps;
    constexpr units::feet_per_second_t IDLE_VELOCITY = 20_fps;

    int sgn(double);
}

class Shooter {
public:
    Shooter(
        std::shared_ptr<turret::turret>,
        std::shared_ptr<flywheel::flywheel>,
        std::shared_ptr<pivot::pivot>
    );

    void tick(frc::Pose2d);

    void set_state(constants::ShooterStates s) { state = s; }

    constants::ShooterStates get_state() const { return state; }

private:
    constants::ShooterStates state = constants::ShooterStates::Stopped;

    std::shared_ptr<turret::turret> turret_control;
    std::shared_ptr<flywheel::flywheel> flywheel_control;
    std::shared_ptr<pivot::pivot> pivot_control;
};

}

}