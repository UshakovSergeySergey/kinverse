#pragma once

#include "exports.h"
#include "joint_type.h"
#include "denavit_hartenberg_parameters.h"
#include "joint_constraints.h"
#include "mesh.h"

namespace kinverse {
  namespace core {

    class KINVERSE_CORE_API Robot {
     public:
      using Ptr = std::shared_ptr<Robot>;
      using ConstPtr = std::shared_ptr<const Robot>;

      void setDHTable(const std::vector<DenavitHartenbergParameters>& dhTable);
      std::vector<DenavitHartenbergParameters> getDHTable() const;

      void setJointConstraints(const std::vector<JointConstraints>& constraints);
      std::vector<JointConstraints> getJointConstraints() const;

      void setConfiguration(const std::vector<double>& configuration);
      std::vector<double> getConfiguration() const;

      /**
       * For example KUKA KR5 Arc has its first axis pointing down, if you'll build kinematic diagram for KUKA you'll see
       * that the whole robot seems to be upside down (end effector has negative z values).
       * But in real life KUKA's end effector has positive z values. It is because base transform rotates robot about X axis 180 degrees
       * In order to fix this it comes handy to add some base transform.
       * Do not confuse base transform with robot transform.
       * Base transform tells how we need to rotate and translate robot (its kinematic diagram) in order to get robots local coordinate frame.
       * Whether robot transform tells how is robot positioned and oriented in the world.
       */
      void setBaseTransform(const Eigen::Affine3d& transform);
      Eigen::Affine3d getBaseTransform() const;

      void setTransform(const Eigen::Affine3d& transform);
      Eigen::Affine3d getTransform() const;






      unsigned int getNumberOfJoints() const;
      unsigned int getNumberOfLinks() const;

      std::vector<Eigen::Affine3d> getJointCoordinateFrames() const;
      std::vector<Eigen::Affine3d> getLinkCoordinateFrames() const;

      Eigen::MatrixXd computeJacobian() const;
      Eigen::MatrixXd computePositionJacobian() const;
      Eigen::MatrixXd computeInverseJacobian(const Eigen::MatrixXd& jacobian) const;

     private:
      std::vector<double> getAxisValues(const std::vector<double>& axisValues) const;

      Eigen::Affine3d m_baseTransform{ Eigen::Affine3d::Identity() };

      std::vector<DenavitHartenbergParameters> m_dhTable{};
      std::vector<JointConstraints> m_constraints{};
      std::vector<double> m_configuration{};
    };

  }  // namespace core
}  // namespace kinverse
