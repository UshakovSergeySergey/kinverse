#pragma once

#include "exports.h"
#include "joint_type.h"
#include "denavit_hartenberg_parameters.h"
#include "joint_constraints.h"

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

      void addJoint(const Eigen::Vector3d& jointAxis, const Eigen::Vector3d& displacementVector, JointType jointType);

      unsigned int getNumberOfJoints() const;
      unsigned int getNumberOfLinks() const;

      std::vector<Eigen::Affine3d> getJointCoordinateFrames(const std::vector<double>& axisValues = {}) const;
      std::vector<Eigen::Affine3d> getLinkCoordinateFrames(const std::vector<double>& axisValues = {}) const;

     private:
      std::vector<double> getAxisValues(const std::vector<double>& axisValues) const;

      std::vector<DenavitHartenbergParameters> m_dhTable{};
      std::vector<JointConstraints> m_constraints{};
      std::vector<double> m_configuration{};
    };

  }  // namespace core
}  // namespace kinverse
