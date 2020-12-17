#pragma once

#include "exports.h"
#include <kinverse/core/robot.h>
#include <kinverse/core/robot_type.h>

namespace kinverse {
  namespace factory {

    class KINVERSE_FACTORY_API RobotFactory {
     public:
      static core::Robot::Ptr create(core::RobotType robotType);

     private:
      static core::Robot::Ptr createKukaKR5ArcRobot();
    };

  }  // namespace factory
}  // namespace kinverse
