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

#include "stdafx.h"
#include "test_robot.h"
#include <kinverse/core/robot.h>
#include <kinverse/core/denavit_hartenberg_parameters.h>

namespace kinverse {
  namespace core {

    TEST_F(TestRobot, GetDHTable_WhenDefaultConstructed_ReturnsEmptyTable) {
      // arrange
      const auto robot = std::make_shared<Robot>();

      // act
      const auto dhTable = robot->getDHTable();

      // assert
      EXPECT_TRUE(dhTable.empty());
    }

    TEST_F(TestRobot, GetDHTable_WhenDHTableIsSet_ReturnsProvidedDHTable) {
      // arrange
      const std::vector<DenavitHartenbergParameters> expectedDHTable{
        { JointType::Prismatic, 123.234, 89.2349, 98734.10900347, 239487.12 },  //
        { JointType::Revolute, 34.1234, 2343.3457, 787.232, 9789.2343 },        //
        { JointType::Revolute, 54.0173, 95.03921, 34.340019, 132.048727 }       //
      };

      const auto robot = std::make_shared<Robot>();
      robot->setDHTable(expectedDHTable);

      // act
      const auto dhTable = robot->getDHTable();

      // assert
      ASSERT_EQ(dhTable.size(), expectedDHTable.size());
      for (auto rowCounter = 0u; rowCounter < dhTable.size(); ++rowCounter) {
        EXPECT_EQ(dhTable[rowCounter], expectedDHTable[rowCounter]);
      }
    }

    TEST_F(TestRobot, GetJointConstraints_WhenDefaultConstructed_ReturnsEmptyListOfConstraints) {
      // arrange
      const auto robot = std::make_shared<Robot>();

      // act
      const auto constraints = robot->getJointConstraints();

      // assert
      EXPECT_TRUE(constraints.empty());
    }

    TEST_F(TestRobot, GetJointConstraints_WhenJointConstraintsAreSet_ReturnsProvidedJointConstraints) {
      // arrange
      const std::vector<JointConstraints> expectedConstraints{
        JointConstraints{ 23.092, -234.0032, 552.82 },      //
        JointConstraints{ 54.0345, 34.1983498, 544.892 },   //
        JointConstraints{ 343.0943, 3095.08934, 891.9834 }  //
      };

      const auto robot = std::make_shared<Robot>();
      robot->setJointConstraints(expectedConstraints);

      // act
      const auto constraints = robot->getJointConstraints();

      // assert
      ASSERT_EQ(constraints.size(), expectedConstraints.size());
      for (auto jointCounter = 0u; jointCounter < constraints.size(); ++jointCounter) {
        EXPECT_EQ(constraints[jointCounter], expectedConstraints[jointCounter]);
      }
    }

    TEST_F(TestRobot, GetMeshes_WhenDefaultConstructed_ReturnsEmptyListOfMeshes) {
      // arrange
      const auto robot = std::make_shared<Robot>();

      // act
      const auto meshes = robot->getMeshes();

      // assert
      EXPECT_TRUE(meshes.empty());
    }

    TEST_F(TestRobot, GetMeshes_WhenMeshesAreSet_ReturnsProvidedMeshList) {
      // arrange
      const std::vector<Mesh::ConstPtr> expectedMeshes{
        std::make_shared<Mesh>(Mesh::Vertices{ { 0.0, 0.0, 0.0 } }, Mesh::Faces{}),               //
        std::make_shared<Mesh>(Mesh::Vertices{ { 1.0, 2.0, 3.0 } }, Mesh::Faces{ { 0, 1, 2 } }),  //
        std::make_shared<Mesh>(Mesh::Vertices{}, Mesh::Faces{ { 3, 7, 1 } })                      //
      };

      const auto robot = std::make_shared<Robot>();
      robot->setMeshes(expectedMeshes);

      // act
      const auto meshes = robot->getMeshes();

      // assert
      ASSERT_EQ(meshes.size(), expectedMeshes.size());
      for (auto meshCounter = 0u; meshCounter < meshes.size(); ++meshCounter) {
        EXPECT_EQ(meshes[meshCounter], expectedMeshes[meshCounter]);
        EXPECT_EQ(*meshes[meshCounter], *expectedMeshes[meshCounter]);
      }
    }

    TEST_F(TestRobot, GetTransform_WhenDefaultConstructed_ReturnsIdentityTransform) {
      // arrange
      const auto robot = std::make_shared<Robot>();

      // act
      const auto transform = robot->getTransform();

      // assert
      EXPECT_TRUE(transform.isApprox(Eigen::Affine3d::Identity()));
    }

    TEST_F(TestRobot, GetTransform_WhenTransformIsSet_ReturnsProvidedTransform) {
      // arrange
      const Eigen::Affine3d expectedTransform = Eigen::Translation3d(Eigen::Vector3d::Random()) * Eigen::Quaterniond::UnitRandom();

      const auto robot = std::make_shared<Robot>();
      robot->setTransform(expectedTransform);

      // act
      const auto transform = robot->getTransform();

      // assert
      EXPECT_TRUE(transform.isApprox(expectedTransform));
    }

    TEST_F(TestRobot, SetTransform_WhenTransformIsNotFinite_ThrowsException) {
      // arrange
      const Eigen::Affine3d transform = Eigen::Translation3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0) *
                                        Eigen::AngleAxisd(std::numeric_limits<double>::infinity(), Eigen::Vector3d::UnitZ());
      const auto robot = std::make_shared<Robot>();

      // act assert
      EXPECT_THROW(robot->setTransform(transform), std::domain_error);
    }

    TEST_F(TestRobot, GetBaseTransform_WhenDefaultConstructed_ReturnsIdentityTransform) {
      // arrange
      const auto robot = std::make_shared<Robot>();

      // act
      const auto transform = robot->getBaseTransform();

      // assert
      EXPECT_TRUE(transform.isApprox(Eigen::Affine3d::Identity()));
    }

    TEST_F(TestRobot, GetBaseTransform_WhenBaseTransformIsSet_ReturnsProvidedTransform) {
      // arrange
      const Eigen::Affine3d expectedTransform = Eigen::Translation3d(Eigen::Vector3d::Random()) * Eigen::Quaterniond::UnitRandom();

      const auto robot = std::make_shared<Robot>();
      robot->setBaseTransform(expectedTransform);

      // act
      const auto transform = robot->getBaseTransform();

      // assert
      EXPECT_TRUE(transform.isApprox(expectedTransform));
    }

    TEST_F(TestRobot, SetBaseTransform_WhenBaseTransformIsNotFinite_ThrowsException) {
      // arrange
      const Eigen::Affine3d transform = Eigen::Translation3d(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0) *
                                        Eigen::AngleAxisd(std::numeric_limits<double>::infinity(), Eigen::Vector3d::UnitZ());
      const auto robot = std::make_shared<Robot>();

      // act assert
      EXPECT_THROW(robot->setBaseTransform(transform), std::domain_error);
    }

    TEST_F(TestRobot, GetNumberOfJoints_WhenDefaultConstructed_ReturnsZero) {
      // arrange
      const auto robot = std::make_shared<Robot>();

      // act
      const auto numberOfJoints = robot->getNumberOfJoints();

      // assert
      EXPECT_EQ(numberOfJoints, 0);
    }

    TEST_F(TestRobot, GetNumberOfJoints_WhenDHTableIsSet_ReturnsNumberOfRowsInDHTable) {
      // arrange
      const std::vector<DenavitHartenbergParameters> dhTable{
        { JointType::Prismatic, 123.234, 89.2349, 98734.10900347, 239487.12 },  //
        { JointType::Revolute, 34.1234, 2343.3457, 787.232, 9789.2343 },        //
        { JointType::Revolute, 54.0173, 95.03921, 34.340019, 132.048727 }       //
      };

      const auto robot = std::make_shared<Robot>();
      robot->setDHTable(dhTable);

      // act
      const auto numberOfJoints = robot->getNumberOfJoints();

      // assert
      EXPECT_EQ(numberOfJoints, dhTable.size());
    }

    TEST_F(TestRobot, GetNumberOfLinks_WhenDefaultConstructed_ReturnsOne) {
      // arrange
      const auto robot = std::make_shared<Robot>();

      // act
      const auto numberOfLinks = robot->getNumberOfLinks();

      // assert
      EXPECT_EQ(numberOfLinks, 1);
    }

    TEST_F(TestRobot, GetNumberOfLinks_WhenDHTableIsSet_ReturnsNumberOfRowsInDHTablePlusOne) {
      // arrange
      const std::vector<DenavitHartenbergParameters> dhTable{
        { JointType::Prismatic, 123.234, 89.2349, 98734.10900347, 239487.12 },  //
        { JointType::Revolute, 34.1234, 2343.3457, 787.232, 9789.2343 },        //
        { JointType::Revolute, 54.0173, 95.03921, 34.340019, 132.048727 }       //
      };

      const auto robot = std::make_shared<Robot>();
      robot->setDHTable(dhTable);

      // act
      const auto numberOfLinks = robot->getNumberOfLinks();

      // assert
      EXPECT_EQ(numberOfLinks, dhTable.size() + 1);
    }

  }  // namespace core
}  // namespace kinverse
