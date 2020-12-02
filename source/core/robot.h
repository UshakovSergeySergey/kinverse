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

      void setBaseTransform(const Eigen::Affine3d& transform);
      Eigen::Affine3d getBaseTransform() const;

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
