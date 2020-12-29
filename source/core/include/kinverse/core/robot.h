/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2020, Sergey Ushakov
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include "exports.h"
#include "denavit_hartenberg_parameters.h"
#include "joint_constraints.h"
#include "mesh.h"

namespace kinverse {
  namespace core {

    /**
     * @brief Forward declaration of ForwardKinematicsSolver.
     * This is an internal class that is used by Robot in order to solve FK problem
     */
    class ForwardKinematicsSolver;

    /**
     * @class Robot
     * @brief This is one of the main classes in the kinverse library.
     * It is stores information about robot (joints, constraints, meshes for visualization, etc).
     * This class is also solving forward kinematics problem. In order to do this it uses library's internal class
     * called ForwardKinematicsSolver.
     * Note that inverse kinematics problem is solved outside this class via InverseKinematicsSolver's.
     */
    class KINVERSE_CORE_API Robot : public std::enable_shared_from_this<Robot> {
     public:
      /**
       * @brief Smart pointer to @p Robot
       */
      using Ptr = std::shared_ptr<Robot>;

      /**
       * @brief Smart pointer to const @p Robot
       */
      using ConstPtr = std::shared_ptr<const Robot>;

      /**
       * @brief This method sets robots Denavit-Hartenberg table. Each row in the table represents a single joint.
       * @param[in] dhTable - Denavit-Hartenberg table
       */
      void setDHTable(const std::vector<DenavitHartenbergParameters>& dhTable);

      /**
       * @brief Returns Denavit-Hartenberg table of the robot.
       */
      std::vector<DenavitHartenbergParameters> getDHTable() const;

      /**
       * @brief This method sets joint constraints. Number of elements must match number of rows in DH table.
       * @param[in] constraints - list of joint constraints
       */
      void setJointConstraints(const std::vector<JointConstraints>& constraints);

      /**
       * @brief Returns list of joint constraints. Number of constraints corresponds to number of robot's joints.
       */
      std::vector<JointConstraints> getJointConstraints() const;

      /**
       * @brief This method sets meshes that must be used for visualization.
       * Array length must be equal to 'numberOfJoints + 2':
       * - first mesh in the list corresponds to the robots static base;
       * - next we have a set of meshes one per each joint;
       * - the last one is the mesh used for robot tcp.
       * Each entry of this array can be set to nullptr, but the length must match the rule described above.
       * It is also allowed to set empty array as input to this method.
       *
       * Note that robot doesn't know how to visualize itself. In order to visualize it you can either
       * use kinverse::visualization module, or implement visualization yourself.
       *
       * Even though these meshes are not used in the calculations, robot stores them
       * because they are directly related to him, as well as meshes for collision detection.
       *
       * @param[in] meshes - meshes for visualization
       */
      void setMeshes(const std::vector<Mesh::ConstPtr>& meshes);

      /**
       * @brief Returns list of meshes that must be used for visualization.
       */
      std::vector<Mesh::ConstPtr> getMeshes() const;

      /**
       * @brief This method sets position and orientation of the robot relative to world coordinate space.
       * @param[in] transform - robot transform
       */
      void setTransform(const Eigen::Affine3d& transform);

      /**
       * @brief Returns transform containing robots position and orientation relative to world coordinate space.
       */
      Eigen::Affine3d getTransform() const;

      /**
       * @brief This method sets robot base transform (transform from robot
       * local coordinate frame to first joint coordinate frame).
       * For example KUKA KR5 Arc has its first axis pointing down,
       * if you'll build kinematic diagram for KUKA you'll see
       * that the whole robot seems to be upside down (end effector has negative z values).
       * But in real life KUKA's end effector has positive z values.
       * It is because base transform rotates robot about X axis 180 degrees
       * In order to fix this it comes handy to add some base transform.
       * Do not confuse base transform with robot transform.
       * Base transform tells how we need to rotate and translate robot (its kinematic diagram)
       * in order to get robots local coordinate frame.
       * Whether robot transform tells how is robot positioned and oriented in the world.
       * @param[in] transform - robot's base transform
       */
      void setBaseTransform(const Eigen::Affine3d& transform);

      /**
       * @brief Returns robot's base transform.
       */
      Eigen::Affine3d getBaseTransform() const;

      /**
       * @brief Sets robot TCP (Tool Center Point) transform. Usually, robots have
       * some tool mounted on their flange (e.g. grippers, welding tools, etc.).
       * This transform tells how the tool coordinate frame is positioned and rotated relative to robot's flange.
       * @param[in] transform - TCP transform
       */
      void setTCPTransform(const Eigen::Affine3d& transform);

      /**
       * @brief Returns robot's TCP transform
       */
      Eigen::Affine3d getTCPTransform() const;

      /**
       * @brief This method sets robot's configuration.
       * Note that this method doesn't check for constraint violation.
       * It also initiates forward kinematics problem solving, so after this method is done
       * it update @p m_endEffectorTransform and @p m_linkCoordinateFrames
       */
      void setConfiguration(const Eigen::VectorXd& configuration);

      /**
       * @brief Returns robot's configuration.
       */
      Eigen::VectorXd getConfiguration() const;

      /**
       * @brief Returns number of joints this robot have.
       */
      unsigned int getNumberOfJoints() const;

      /**
       * @brief This method returns end effector transform corresponding to the current configuration
       * (transform from the world coordinate frame to TCP coordinate frame).
       * Note that end effector transform updates each time you call @p setConfiguration method.
       */
      Eigen::Affine3d getEndEffectorTransform() const;

      /**
       * @brief This method returns intermediate transforms obtained during finding solution for forward kinematics.
       * Array contains transforms from local to world coordinate space of the next coordinate frames
       * {BASE, A1, ..., AN, FLANGE, TCP}.
       * Note that this array is updated each time you call @p setConfiguration method.
       */
      std::vector<Eigen::Affine3d> getLinkCoordinateFrames() const;

     private:
      /**
       * @brief Stores position and orientation of the robot in the world space.
       */
      Eigen::Affine3d m_transform{ Eigen::Affine3d::Identity() };

      /**
       * @brief Stores transform from the robot's local coordinate space to robot's first joint coordinate space.
       */
      Eigen::Affine3d m_baseTransform{ Eigen::Affine3d::Identity() };

      /**
       * @brief Stores transform from the robot flange coordinate space to TCP coordinate space.
       */
      Eigen::Affine3d m_tcpTransform{ Eigen::Affine3d::Identity() };

      /**
       * @brief Stores Denavit-Hartenberg table that describes robot joints.
       */
      std::vector<DenavitHartenbergParameters> m_dhTable{};

      /**
       * @brief Array of joint constraints.
       */
      std::vector<JointConstraints> m_constraints{};

      /**
       * @brief Array of meshes used for robot visualization.
       * Array is assumed to have length equal to 'numberOfJoints + 2':
       * - first mesh in the list corresponds to the robots static base;
       * - next we have a set of meshes one per each joint;
       * - the last one is the mesh used for robot tcp.
       */
      std::vector<Mesh::ConstPtr> m_meshes{};

      /**
       * @brief Stores solver used for solving forward kinematics problem
       */
      std::shared_ptr<ForwardKinematicsSolver> m_fkSolver{ nullptr };

      /**
       * @brief Stores current configuration of the robot's joints.
       */
      Eigen::VectorXd m_configuration{};

      /**
       * @brief Contains transform describing how TCP coordinate frame
       * is positioned and rotated relative to world coordinate space.
       */
      Eigen::Affine3d m_endEffectorTransform{ Eigen::Affine3d::Identity() };

      /**
       * @brief This list stores intermediate results of the FK solver.
       * It contains transforms from local to world coordinate space of the next coordinate frames
       * {BASE, A1, ..., AN, FLANGE, TCP}
       */
      std::vector<Eigen::Affine3d> m_linkCoordinateFrames{};
    };

  }  // namespace core
}  // namespace kinverse
