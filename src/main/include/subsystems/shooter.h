
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

namespace subsystems{

namespace shooter{

namespace constants{
    const units::feet_per_second_squared_t GRAVITY = 32.175_fps_sq;
    const frc::Translation2d GOAL_POSITION { 12_ft, 10_ft};
    const units::foot_t GOAL_HEIGHT_DELTA = 5_ft;
}
class Shooter{
    public:
        Shooter(
            std::shared_ptr<turret::turret>,
            std::shared_ptr<flywheel::flywheel>,
            std::shared_ptr<pivot::pivot>

        );
        void tick(frc::Pose2d,units::feet_per_second_t);
        
    private:
    std::shared_ptr<turret::turret> turret_control;
    std::shared_ptr<flywheel::flywheel> flywheel_control;
    std::shared_ptr<pivot::pivot> pivot_control;
};
}
}

