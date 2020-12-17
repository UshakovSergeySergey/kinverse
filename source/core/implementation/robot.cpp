#include "stdafx.h"
#include "../include/kinverse/core/robot.h"
#include <kinverse/math/math.h>

void kinverse::core::Robot::setDHTable(const std::vector<DenavitHartenbergParameters>& dhTable) {
  m_dhTable = dhTable;
}

std::vector<kinverse::core::DenavitHartenbergParameters> kinverse::core::Robot::getDHTable() const {
  return m_dhTable;
}

void kinverse::core::Robot::setJointConstraints(const std::vector<JointConstraints>& constraints) {
  m_constraints = constraints;
}

std::vector<kinverse::core::JointConstraints> kinverse::core::Robot::getJointConstraints() const {
  return m_constraints;
}

void kinverse::core::Robot::setConfiguration(const std::vector<double>& configuration) {
  m_configuration = std::vector<double>(configuration.size(), 0.0);
  for (auto jointCounter = 0u; jointCounter < m_constraints.size(); ++jointCounter) {
    const double clampedAngle = m_constraints[jointCounter].clampAxisValue(configuration[jointCounter]);
    m_configuration[jointCounter] = clampedAngle;
  }
}

std::vector<double> kinverse::core::Robot::getConfiguration() const {
  return getAxisValues(m_configuration);
}

unsigned int kinverse::core::Robot::getNumberOfJoints() const {
  return static_cast<unsigned int>(m_dhTable.size());
}

unsigned int kinverse::core::Robot::getNumberOfLinks() const {
  return getNumberOfJoints() + 1;
}

std::vector<Eigen::Affine3d> kinverse::core::Robot::getJointCoordinateFrames() const {
  const auto axisValues = getAxisValues(m_configuration);

  std::vector<Eigen::Affine3d> frames{};

  Eigen::Affine3d transform = m_baseTransform;
  for (auto jointCounter = 0u; jointCounter < getNumberOfJoints(); jointCounter++) {
    const auto& dhParameters = m_dhTable[jointCounter];
    const double axisValue = axisValues[jointCounter];

    Eigen::Affine3d jointTransform;
    if (dhParameters.getJointType() == JointType::Revolute)
      jointTransform = Eigen::AngleAxisd(axisValue, Eigen::Vector3d::UnitZ());
    else
      jointTransform = Eigen::Translation3d(0.0, 0.0, axisValue);

    const Eigen::Affine3d currentTransform = transform * jointTransform;
    frames.push_back(currentTransform);

    transform = transform * dhParameters.getTransform(axisValue);
  }

  return frames;
}

std::vector<Eigen::Affine3d> kinverse::core::Robot::getLinkCoordinateFrames() const {
  // Number of links is always equal to number of joints + 1
  // The first link (also known as base frame or inertial frame) is always the origin.
  // The last link is the end-effector.

  const auto axisValues = getAxisValues(m_configuration);

  std::vector<Eigen::Affine3d> frames{};

  Eigen::Affine3d transform = m_baseTransform;

  frames.push_back(transform);

  for (auto jointCounter = 0u; jointCounter < getNumberOfJoints(); jointCounter++) {
    const auto& dhParameters = m_dhTable[jointCounter];
    const double axisValue = axisValues[jointCounter];

    const Eigen::Affine3d currentTransform = dhParameters.getTransform(axisValue);
    transform = transform * currentTransform;
    frames.push_back(transform);
  }

  return frames;
}

std::vector<double> kinverse::core::Robot::getAxisValues(const std::vector<double>& axisValues) const {
  if (axisValues.empty()) {
    // if empty vector was provided then assume zero configuration
    return std::vector<double>(getNumberOfJoints(), 0.0);
  }

  // otherwise check that number of joints and values matches
  if (axisValues.size() != m_dhTable.size()) {
    std::stringstream ss;
    ss << "Robot has " << getNumberOfJoints() << " joints, but only " << axisValues.size() << " axis values were given!";
    throw std::invalid_argument(ss.str());
  }

  return axisValues;
}

/*
std::vector<Eigen::Affine3d> kinverse::core::Robot::getFrames() const {
  Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), jointAxis);
  Eigen::Affine3d transform;
  transform = Eigen::Translation3d(displacementVector) * rotation;
  m_frames.push_back(transform);

  return m_frames;
}
*/
/*
     public:
      getEndEffectorCoordinateFrame();
      / **
       * @param[in] jointAxis - axis of revolution or direction of motion in case of prismatic joint
       * @param[in] displacementVector - displacement vector from the previous frame
       * @param[in] jointType - joint type (revolute or prismatic)
       * @param[in] jointName - joint name (e.g. A1 or A2)
       * /

      std::vector<Eigen::Affine3d> getFrames() const;
*/

/*
number of rows in jacobian matrix       = 6 (xyzabc)
number of columns in jacobian matrix    = number of joints




linear
rotational
*/

Eigen::MatrixXd kinverse::core::Robot::computePositionJacobian() const {
  // Jacobian matrix defines the relationship between joint velocities and end-effector velocities.
  const unsigned int numberOfRows = 3;
  const unsigned int numberOfColumns = getNumberOfJoints();

  Eigen::MatrixXd jacobian(numberOfRows, numberOfColumns);
  jacobian.setZero();

  const auto linkFrames = getLinkCoordinateFrames();
  const Eigen::Vector3d endEffectorPosition = linkFrames.back().translation();
  for (auto jointCounter = 0u; jointCounter < getNumberOfJoints(); ++jointCounter) {
    const Eigen::Vector3d jointAxis = linkFrames[jointCounter].rotation().col(2);

    Eigen::Vector3d linear;

    if (m_dhTable[jointCounter].getJointType() == JointType::Prismatic) {
      linear = jointAxis;
    } else {
      const Eigen::Vector3d jointPosition = linkFrames[jointCounter].translation();

      linear = jointAxis.cross(endEffectorPosition - jointPosition);
    }

    jacobian.col(jointCounter) << linear;
  }

  return jacobian;
}

Eigen::MatrixXd kinverse::core::Robot::computeJacobian() const {
  // Jacobian matrix defines the relationship between joint velocities and end-effector velocities.
  const unsigned int numberOfRows = 6;
  const unsigned int numberOfColumns = getNumberOfJoints();

  Eigen::MatrixXd jacobian(numberOfRows, numberOfColumns);
  jacobian.setZero();

  const auto linkFrames = getLinkCoordinateFrames();
  const Eigen::Vector3d endEffectorPosition = linkFrames.back().translation();
  for (auto jointCounter = 0u; jointCounter < getNumberOfJoints(); ++jointCounter) {
    const Eigen::Vector3d jointAxis = linkFrames[jointCounter].rotation().col(2);

    Eigen::Vector3d linear;
    Eigen::Vector3d rotational;

    if (m_dhTable[jointCounter].getJointType() == JointType::Prismatic) {
      linear = jointAxis;
      rotational = { 0.0, 0.0, 0.0 };
    } else {
      const Eigen::Vector3d jointPosition = linkFrames[jointCounter].translation();

      linear = jointAxis.cross(endEffectorPosition - jointPosition);
      rotational = jointAxis;
    }

    jacobian.col(jointCounter) << linear, rotational;
  }

  return jacobian;

  /*
   * compute the Jacobian
   * compute pseudoinverse of the Jacobian
   * deltaAxes = pseudoInverseJacobian * deltaEndEffector
   * axes = axes + alpha * deltaAxes
   */
}

Eigen::MatrixXd kinverse::core::Robot::computeInverseJacobian(const Eigen::MatrixXd& jacobian) const {
  if (jacobian.cols() == jacobian.rows())
    return jacobian.inverse();

  return jacobian.transpose();
  return (jacobian.transpose() * jacobian).inverse() * jacobian.transpose();

  //  return jacobian.transpose();

  // matrix is not square, compute pseudoinverse jacobian
  //  return (jacobian.transpose() * jacobian).inverse() * jacobian.transpose();
}

void kinverse::core::Robot::setBaseTransform(const Eigen::Affine3d& transform) {
  m_baseTransform = transform;
}

Eigen::Affine3d kinverse::core::Robot::getBaseTransform() const {
  return m_baseTransform;
}

void kinverse::core::Robot::setMeshes(const std::vector<Mesh::ConstPtr>& meshes) {
  m_meshes = meshes;
}

std::vector<kinverse::core::Mesh::ConstPtr> kinverse::core::Robot::getMeshes() const {
  return m_meshes;
}
