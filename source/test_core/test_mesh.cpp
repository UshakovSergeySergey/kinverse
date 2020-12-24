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
#include "test_mesh.h"

namespace kinverse {
  namespace core {

    TEST_F(TestMesh, GetNumberOfVertices_WhenMeshDefaultCreated_ReturnsZero) {
      // arrange
      const auto mesh = std::make_shared<Mesh>();

      // act
      const auto numberOfVertices = mesh->getNumberOfVertices();

      // assert
      EXPECT_EQ(numberOfVertices, 0);
    }

    TEST_F(TestMesh, GetNumberOfVertices_WhenVertexListIsSet_ReturnsNumberOfVerticesInThisList) {
      // arrange
      const std::vector<Eigen::Vector3d> vertices{
        Eigen::Vector3d::Zero(),   //
        Eigen::Vector3d::UnitX(),  //
        Eigen::Vector3d::UnitY(),  //
        Eigen::Vector3d::UnitZ()   //
      };
      const auto mesh = std::make_shared<Mesh>();
      mesh->setVertices(vertices);

      // act
      const auto numberOfVertices = mesh->getNumberOfVertices();

      // assert
      EXPECT_EQ(numberOfVertices, vertices.size());
    }

    TEST_F(TestMesh, GetNumberOfFaces_WhenMeshDefaultCreated_ReturnsZero) {
      // arrange
      const auto mesh = std::make_shared<Mesh>();

      // act
      const auto numberOfFaces = mesh->getNumberOfFaces();

      // assert
      EXPECT_EQ(numberOfFaces, 0);
    }

    TEST_F(TestMesh, GetNumberOfFaces_WhenFaceListIsSet_ReturnsNumberOfFacesInThisList) {
      // arrange
      const std::vector<std::vector<unsigned long long>> faces{
        { 0, 1, 2, 3 },  //
        {},              //
        { 0, 1, 2 }      //
      };
      const auto mesh = std::make_shared<Mesh>();
      mesh->setFaces(faces);

      // act
      const auto numberOfFaces = mesh->getNumberOfFaces();

      // assert
      EXPECT_EQ(numberOfFaces, faces.size());
    }

    TEST_F(TestMesh, GetVertices_WhenMeshDefaultCreated_ReturnsEmptyList) {
      // arrange
      const auto mesh = std::make_shared<Mesh>();

      // act
      const auto vertices = mesh->getVertices();

      // assert
      EXPECT_EQ(vertices.size(), 0);
    }

    TEST_F(TestMesh, GetVertices_WhenVertexListIsSet_ReturnsThisVertexList) {
      // arrange
      const std::vector<Eigen::Vector3d> expectedVertices{
        Eigen::Vector3d::Zero(),                  //
        Eigen::Vector3d::UnitZ(),                 //
        Eigen::Vector3d{ 1.2345, 6.789, 0.1234 }  //
      };
      const auto mesh = std::make_shared<Mesh>();
      mesh->setVertices(expectedVertices);

      // act
      const auto vertices = mesh->getVertices();

      // assert
      ASSERT_EQ(vertices.size(), expectedVertices.size());

      for (auto vertexCounter = 0u; vertexCounter < vertices.size(); ++vertexCounter) {
        const Eigen::Vector3d& vertex = vertices[vertexCounter];
        const Eigen::Vector3d& expectedVertex = expectedVertices[vertexCounter];
        EXPECT_TRUE(vertex.isApprox(expectedVertex));
      }
    }

    TEST_F(TestMesh, GetFaces_WhenMeshDefaultCreated_ReturnsEmptyList) {
      // arrange
      const auto mesh = std::make_shared<Mesh>();

      // act
      const auto faces = mesh->getFaces();

      // assert
      EXPECT_EQ(faces.size(), 0);
    }

    TEST_F(TestMesh, GetFaces_WhenFaceListIsSet_ReturnsThisFaceList) {
      // arrange
      const std::vector<std::vector<unsigned long long>> expectedFaces{
        { 0, 1, 2, 3 },  //
        {},              //
        { 0, 1, 2 }      //
      };
      const auto mesh = std::make_shared<Mesh>();
      mesh->setFaces(expectedFaces);

      // act
      const auto faces = mesh->getFaces();

      // assert
      ASSERT_EQ(faces.size(), expectedFaces.size());
      for (auto faceCounter = 0u; faceCounter < faces.size(); ++faceCounter) {
        const auto& face = faces[faceCounter];
        const auto& expectedFace = expectedFaces[faceCounter];
        ASSERT_EQ(face.size(), expectedFace.size());
        for (auto indexCounter = 0u; indexCounter < face.size(); ++indexCounter) {
          EXPECT_EQ(face[indexCounter], expectedFace[indexCounter]);
        }
      }
    }

    TEST_P(TestMesh, EqualityOperator_WorksCorrectly) {
      // arrange
      const auto lhs = std::get<0>(GetParam());
      const auto rhs = std::get<1>(GetParam());
      const bool expectedResult = std::get<2>(GetParam());

      // act
      const auto result = lhs == rhs;

      // assert
      EXPECT_EQ(result, expectedResult);
    }

    TEST_P(TestMesh, InequalityOperator_WorksCorrectly) {
      // arrange
      const auto lhs = std::get<0>(GetParam());
      const auto rhs = std::get<1>(GetParam());
      const bool expectedResult = !std::get<2>(GetParam());

      // act
      const auto result = lhs != rhs;

      // assert
      EXPECT_EQ(result, expectedResult);
    }

  }  // namespace core
}  // namespace kinverse
