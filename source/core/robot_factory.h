#pragma once

#include "exports.h"
#include "robot.h"
#include "robot_type.h"

namespace kinverse {
  namespace core {

    class KINVERSE_CORE_API RobotFactory {
     public:
      static Robot::Ptr create(RobotType robotType);

     private:
      static Robot::Ptr createKukaKR5ArcRobot();
    };

  }  // namespace core
}  // namespace kinverse
