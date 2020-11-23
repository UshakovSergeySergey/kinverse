﻿#pragma once

#include "exports.h"
#include "joint_type.h"

namespace kinverse {
  namespace core {

    /**
     * @class DenavitHartenbergParameters
     * @brief This class represents structure that stores Denavit-Hartenberg parameters.
     * It allows to easily transform DH parameters to Eigen::Affine transform.
     * Transform is parameterized with the axis value (angle or distance depending on the joint type).
     */
    class KINVERSE_CORE_API DenavitHartenbergParameters {
     public:
      /**
       * @brief Simple constructor.
       * @param[in] jointType - type of joint
       * @param[in] d - displacement along the previous joint's Z axis
       * @param[in] theta - angle in radians about the previous joint's Z axis
       * @param[in] r - displacement along the rotated X axis
       * @param[in] alpha - angle in radians about the rotated X axis
       */
      DenavitHartenbergParameters(JointType jointType, double d, double theta, double r, double alpha);

      /**
       * @brief This constructor is provided in order to simplify the computation of DH parameters.
       * If you don't know how to compute these parameters, or don't want to bother yourself with geometry, use this helper constructor.
       * Joint axis (axis of rotation in case of revolute joint, or direction of linear motion in case of prismatic joint) is simply an infinite line in 3d.
       * Line in 3d can be described with (point, direction) pair. This constructor takes this pair and converts it into valid Denavit-Hartenberg parameters.
       * Note that direction matters: 'direction' and '-direction' describe different joints. E.g. for revolute joint rotation axis would be the same, but
       * positive direction of rotation will differ.
       * @param[in] jointType - type of joint (revolute or prismatic)
       * @param[in] position - you can pass any point that belong to the joint's axis.
       * @param[in] direction - direction in which joint's axis is pointing
       */
      DenavitHartenbergParameters(JointType jointType, const Eigen::Vector3d& position, const Eigen::Vector3d& direction);

      /**
       * @brief This method sets the joint type described with these DH parameters
       * @param[in] jointType - joint type (revolute or prismatic)
       */
      void setJointType(JointType jointType);

      /**
       * @brief Returns type of joint described with these DH parameters.
       */
      JointType getJointType() const;

      /**
       * @brief Returns Denavit-Hartenberg transform from the previous joint's coordinate frame to the current joint coordinate frame.
       * Transform is parameterized with angle/distance depending on the joint type.
       * For example, if you want to get transform of the revolute joint coordinate frame when axis angle equals Pi, just pass this angle as parameter.
       * @param[in] value - joint axis value. If joint type is Prismatic, then it represents displacement along Z axis. If joint type is Revolute, then it
       * represents angle in radians about Z axis.
       * @return Returns transform from the previous joint's coordinate frame to the current joint coordinate frame
       */
      Eigen::Affine3d getTransform(double value = 0.0) const;

     private:
      /**
       * @brief Stores type of joint described by Denavit-Hartenberg parameters.
       */
      JointType m_jointType{ JointType::Revolute };

      /**
       * @brief Stores displacement along the previous joint's Z axis
       */
      double m_d{ 0.0 };

      /**
       * @brief Stores angle in radians about the previous joint's Z axis
       */
      double m_theta{ 0.0 };

      /**
       * @brief Stores displacement along the rotated X axis
       */
      double m_r{ 0.0 };

      /**
       * @brief Stores angle in radians about the rotated X axis
       */
      double m_alpha{ 0.0 };
    };

  }  // namespace core
}  // namespace kinverse
