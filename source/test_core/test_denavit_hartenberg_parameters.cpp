#include "stdafx.h"
#include "test_denavit_hartenberg_parameters.h"
#include <kinverse/core/denavit_hartenberg_parameters.h>

namespace kinverse {
  namespace core {

    TEST_F(TestDenavitHartenbergParameters, GetMatrix_IfJointTypeIsRevolute_ReturnsValidMatrix) {
      // arrange
      const double d = 100.0;
      const double theta = 0.123 * M_PI;
      const double r = 200.0;
      const double alpha = 0.456 * M_PI;
      const double angle = 0.789 * M_PI;
      const Eigen::Matrix4d expectedMatrix = getDHMatrix(d, theta, r, alpha, angle, 0.0);

      DenavitHartenbergParameters dhParameters(JointType::Revolute, d, theta, r, alpha);

      // act
      const Eigen::Affine3d transform = dhParameters.getTransform(angle);

      // assert
      EXPECT_TRUE(transform.matrix().isApprox(expectedMatrix));
    };

    TEST_F(TestDenavitHartenbergParameters, GetMatrix_IfJointTypeIsPrismatic_ReturnsValidMatrix) {
      // arrange
      const double d = 100.0;
      const double theta = 0.123 * M_PI;
      const double r = 200.0;
      const double alpha = 0.456 * M_PI;
      const double distance = 123.0;
      const Eigen::Matrix4d expectedMatrix = getDHMatrix(d, theta, r, alpha, 0.0, distance);

      DenavitHartenbergParameters dhParameters(JointType::Prismatic, d, theta, r, alpha);

      // act
      const Eigen::Affine3d transform = dhParameters.getTransform(distance);

      // assert
      EXPECT_TRUE(transform.matrix().isApprox(expectedMatrix));
    };

    TEST_F(TestDenavitHartenbergParameters, GetMatrix_IfDHParametersAreNotFinite_) {
      // arrange
      DenavitHartenbergParameters dhParameters(JointType::Prismatic,
                                               std::numeric_limits<double>::infinity(),
                                               std::numeric_limits<double>::quiet_NaN(),
                                               std::numeric_limits<double>::quiet_NaN(),
                                               std::numeric_limits<double>::infinity());

      // act
      const Eigen::Affine3d transform = dhParameters.getTransform();

      // ASSERT
      EXPECT_FALSE(transform.matrix().allFinite());
    };

  }  // namespace core
}  // namespace kinverse
