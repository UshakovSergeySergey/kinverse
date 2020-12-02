#pragma once

#include "exports.h"
#include "i_gizmo.h"
#include <core/robot.h>
#include "revolute_joint_gizmo.h"
#include "mesh_gizmo.h"

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

      void setMeshes(const std::vector<core::Mesh::ConstPtr>& jointMeshes);
      std::vector<core::Mesh::ConstPtr> getMeshes() const;

      void setConfiguration(const std::vector<double>& axisValues);
      std::vector<double> getConfiguration() const;

      void show(void* renderer) override;

      void showLinks(bool show);
      void showJoints(bool show);

     private:
      void robotStructureChanged();
      void robotConfigurationChanged();

      core::Robot::ConstPtr m_robot{ nullptr };
      std::vector<double> m_axisValues{};
      std::vector<core::Mesh::ConstPtr> m_meshes{};

      std::vector<RevoluteJointGizmo::Ptr> m_jointGizmos{};
      std::vector<CoordinateFrameGizmo::Ptr> m_linkGizmos{};
      std::vector<MeshGizmo::Ptr> m_jointMeshGizmos{};
      CoordinateFrameGizmo::Ptr m_endEffectorGizmo{ nullptr };
    };

  }  // namespace visualization
}  // namespace kinverse
