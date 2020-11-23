#pragma once

#include "exports.h"
#include "i_gizmo.h"
#include <core/robot.h>
#include "revolute_joint_gizmo.h"

namespace kinverse {
  namespace visualization {

    class KINVERSE_VISUALIZATION_API RobotGizmo : public IGizmo {
     public:
      using Ptr = std::shared_ptr<RobotGizmo>;
      using ConstPtr = std::shared_ptr<const RobotGizmo>;

      explicit RobotGizmo(const IGizmo* parentGizmo = nullptr, core::Robot::ConstPtr robot = nullptr);
      virtual ~RobotGizmo() = default;

      void setRobot(core::Robot::ConstPtr robot);
      core::Robot::ConstPtr getRobot() const;

      void setConfiguration(const std::vector<double>& axisValues);
      std::vector<double> getConfiguration() const;

      void draw(void* renderer) override;

     private:
      void robotStructureChanged();
      void robotConfigurationChanged();

      core::Robot::ConstPtr m_robot{ nullptr };
      std::vector<double> m_axisValues{};

      std::vector<RevoluteJointGizmo::Ptr> m_jointGizmos{};
      std::vector<CoordinateFrameGizmo::Ptr> m_linkGizmos{};
    };

  }  // namespace visualization
}  // namespace kinverse
