#pragma once

#include "exports.h"
#include "robot.h"

namespace kinverse {
  namespace core {

    class KINVERSE_CORE_API AnalyticalSolver {
     public:
      using Ptr = std::shared_ptr<AnalyticalSolver>;
      using ConstPtr = std::shared_ptr<const AnalyticalSolver>;

      explicit AnalyticalSolver(Robot::ConstPtr robot);
      std::vector<std::vector<double>> solve(const Eigen::Affine3d& endEffectorTransform);

     private:
      double computeGamma() const;
      double getDistanceBetweenSecondAndThirdJoints() const;
      double getDistanceBetweenThirdJointAndWristPosition() const;
      Eigen::Affine3d convertWorldToLocal(const Eigen::Affine3d& transform) const;
      Eigen::Vector3d computeWristPosition(const Eigen::Affine3d& targetTransform) const;
      std::vector<std::vector<double>> solveForPosition() const;
      Eigen::Vector3d getFirstJointZAxis() const;
      std::vector<std::vector<double>> solvePosition(double theta1, bool facingForward) const;
      void getTriangle(double theta1, double& a, double& b, double& c) const;
      Eigen::Vector3d computeSecondJointPosition(double theta1) const;
      std::vector<std::vector<double>> solveForOrientation(const std::vector<double>& configuration) const;
      Eigen::Matrix3d computeWristOrientation(double theta1, double theta2, double theta3) const;

      Robot::ConstPtr m_robot{ nullptr };
      /**
       * @brief Stores angle between robot's Z4 axis and direction from third joint to wrist position.
       */
      double m_gamma{ 0.0 };
      double m_distanceFromSecondToThirdJoint{ 0.0 };
      double m_distanceFromThirdJointToWrist{ 0.0 };
      Eigen::Affine3d m_targetTransform{ Eigen::Affine3d::Identity() };
      Eigen::Vector3d m_wristPosition{ Eigen::Vector3d::Zero() };
    };

  }  // namespace core
}  // namespace kinverse
