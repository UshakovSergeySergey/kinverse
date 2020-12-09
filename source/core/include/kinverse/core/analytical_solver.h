﻿#pragma once

#include "exports.h"
#include "robot.h"

namespace kinverse {
  namespace core {

    class KINVERSE_CORE_API AnalyticalSolver {
     public:
      using Ptr = std::shared_ptr<AnalyticalSolver>;
      using ConstPtr = std::shared_ptr<const AnalyticalSolver>;

      explicit AnalyticalSolver(Robot::ConstPtr robot);
      std::vector<std::vector<double>> solve(const Eigen::Affine3d& endEffectorTransform) const;
      std::vector<double> solveUnrefactored(const Eigen::Affine3d& endEffectorTransform) const;

     private:
      Eigen::Affine3d convertWorldToLocal(const Eigen::Affine3d& transform) const;
      Eigen::Vector3d computeWristPosition(const Eigen::Affine3d& targetTransform) const;
      Eigen::Vector3d getFirstJointZAxis() const;

      double computeGamma() const;
      Eigen::Matrix3d computeWristOrientation(double theta1, double theta2, double theta3) const;

      Robot::ConstPtr m_robot{ nullptr };
    };

  }  // namespace core
}  // namespace kinverse