#include "stdafx.h"
#include "main_window.h"
#include <kinverse/visualization/coordinate_frame_gizmo.h>
#include <kinverse/math/math.h>
#include <kinverse/factory/robot_factory.h>

kinverse::simulator::MainWindow::MainWindow(QWidget* parent) : QMainWindow{ parent } {
  m_ui.setupUi(this);

  initializeVisualizer();
  initializeRobot();
  initializeGizmos();

  // change robot appearance
  QObject::connect(m_ui.radioButton_showRobot, &QRadioButton::toggled, this, &MainWindow::onRobotAppearanceChanged);
  QObject::connect(m_ui.radioButton_showKinematicDiagram, &QRadioButton::toggled, this, &MainWindow::onRobotAppearanceChanged);

  // change robot axis values
  QObject::connect(m_ui.doubleSpinBox_A1, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onAxisValueChanged);
  QObject::connect(m_ui.doubleSpinBox_A2, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onAxisValueChanged);
  QObject::connect(m_ui.doubleSpinBox_A3, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onAxisValueChanged);
  QObject::connect(m_ui.doubleSpinBox_A4, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onAxisValueChanged);
  QObject::connect(m_ui.doubleSpinBox_A5, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onAxisValueChanged);
  QObject::connect(m_ui.doubleSpinBox_A6, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onAxisValueChanged);

  onAxisValueChanged();
  onRobotAppearanceChanged();
}

void kinverse::simulator::MainWindow::initializeVisualizer() {
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;
  m_ui.vtkWidget->SetRenderWindow(renderWindow);
  m_ui.vtkWidget->GetRenderWindow()->AddRenderer(renderer);
  m_ui.vtkWidget->show();

  m_kinverseVisualizer = std::make_shared<visualization::KinverseVisualizer>(reinterpret_cast<void*>(&renderWindow), reinterpret_cast<void*>(&renderer));
}

void kinverse::simulator::MainWindow::initializeRobot() {
  m_robot = factory::RobotFactory::create(core::RobotType::KukaKR5Arc);

  const std::vector<double> configuration{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  m_robot->setConfiguration(configuration);
}

void kinverse::simulator::MainWindow::initializeGizmos() {
  auto worldFrameGizmo = std::make_shared<visualization::CoordinateFrameGizmo>(nullptr, Eigen::Affine3d::Identity(), "world");
  m_kinverseVisualizer->addGizmo(worldFrameGizmo);

  m_kinematicDiagramGizmo = std::make_shared<visualization::KinematicDiagramGizmo>(nullptr, m_robot);
  m_kinverseVisualizer->addGizmo(m_kinematicDiagramGizmo);

  m_robotGizmo = std::make_shared<visualization::RobotGizmo>(nullptr, m_robot);
  m_kinverseVisualizer->addGizmo(m_robotGizmo);
}

void kinverse::simulator::MainWindow::onAxisValueChanged() const {
  const std::vector<double> configuration = extractRobotConfigurationFromGui();

  m_robot->setConfiguration(configuration);
  m_kinematicDiagramGizmo->updateRobotConfiguration();
  m_robotGizmo->updateRobotConfiguration();

  if (configurationViolatesConstraints(configuration)) {
    const std::vector<double> clampedConfiguration = m_robot->getConfiguration();
    updateAxesValuesGui(clampedConfiguration);
  }

  updateEndEffectorTransformGui();
}

void kinverse::simulator::MainWindow::onRobotAppearanceChanged() {
  m_showKinematicDiagram = m_ui.radioButton_showKinematicDiagram->isChecked();

  if (m_showKinematicDiagram) {
    m_kinverseVisualizer->addGizmo(m_kinematicDiagramGizmo);
    m_kinverseVisualizer->removeGizmo(m_robotGizmo);
  } else {
    m_kinverseVisualizer->removeGizmo(m_kinematicDiagramGizmo);
    m_kinverseVisualizer->addGizmo(m_robotGizmo);
  }
}

std::vector<double> kinverse::simulator::MainWindow::extractRobotConfigurationFromGui() const {
  const std::vector<double> configuration{ math::degreesToRadians(m_ui.doubleSpinBox_A1->value()), math::degreesToRadians(m_ui.doubleSpinBox_A2->value()),
                                           math::degreesToRadians(m_ui.doubleSpinBox_A3->value()), math::degreesToRadians(m_ui.doubleSpinBox_A4->value()),
                                           math::degreesToRadians(m_ui.doubleSpinBox_A5->value()), math::degreesToRadians(m_ui.doubleSpinBox_A6->value()) };
  return configuration;
}

void kinverse::simulator::MainWindow::updateEndEffectorTransformGui() const {
  const Eigen::Affine3d endEffectorTransform = m_robot->getLinkCoordinateFrames().back();

  Eigen::Vector3d xyz;
  Eigen::Vector3d abc;
  math::toXYZABC(endEffectorTransform, xyz, abc);
  abc = abc * 180.0 * M_1_PI;

  m_ui.doubleSpinBox_X->setValue(xyz.x());
  m_ui.doubleSpinBox_Y->setValue(xyz.y());
  m_ui.doubleSpinBox_Z->setValue(xyz.z());

  m_ui.doubleSpinBox_A->setValue(abc.x());
  m_ui.doubleSpinBox_B->setValue(abc.y());
  m_ui.doubleSpinBox_C->setValue(abc.z());
}

void kinverse::simulator::MainWindow::updateAxesValuesGui(const std::vector<double>& configuration) const {
  m_ui.doubleSpinBox_A1->setValue(math::radiansToDegrees(configuration[0]));
  m_ui.doubleSpinBox_A2->setValue(math::radiansToDegrees(configuration[1]));
  m_ui.doubleSpinBox_A3->setValue(math::radiansToDegrees(configuration[2]));
  m_ui.doubleSpinBox_A4->setValue(math::radiansToDegrees(configuration[3]));
  m_ui.doubleSpinBox_A5->setValue(math::radiansToDegrees(configuration[4]));
  m_ui.doubleSpinBox_A6->setValue(math::radiansToDegrees(configuration[5]));
}

bool kinverse::simulator::MainWindow::configurationViolatesConstraints(const std::vector<double>& configuration) const {
  bool violatesConstraints = false;

  const auto constraints = m_robot->getJointConstraints();
  for (auto axisCounter = 0u; axisCounter < configuration.size(); ++axisCounter) {
    violatesConstraints |= constraints[axisCounter].violatesRangeConstraint(configuration[axisCounter]);
  }

  return violatesConstraints;
}

///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////

void kinverse::simulator::MainWindow::solveIK() {
  const auto getEndEffectorPosition = [this]() -> Eigen::VectorXd {
    const Eigen::Affine3d transform = m_robot->getLinkCoordinateFrames().back();

    Eigen::Vector3d sourceXYZ;
    Eigen::Vector3d sourceABC;
    math::toXYZABC(transform, sourceXYZ, sourceABC);

    Eigen::VectorXd source(6);
    source(0) = sourceXYZ.x();
    source(1) = sourceXYZ.y();
    source(2) = sourceXYZ.z();
    source(3) = sourceABC.x();
    source(4) = sourceABC.y();
    source(5) = sourceABC.z();

    return source;
  };

  const auto getAxisConfiguration = [this]() -> Eigen::VectorXd {
    const std::vector<double> configuration = m_robot->getConfiguration();
    Eigen::VectorXd configurationVector(6);
    configurationVector(0) = configuration[0];
    configurationVector(1) = configuration[1];
    configurationVector(2) = configuration[2];
    configurationVector(3) = configuration[3];
    configurationVector(4) = configuration[4];
    configurationVector(5) = configuration[5];
    return configurationVector;
  };

  const auto setAxisConfiguration = [this](const Eigen::VectorXd& configurationVector) {
    std::vector<double> configuration(6, 0);
    configuration[0] = configurationVector(0);
    configuration[1] = configurationVector(1);
    configuration[2] = configurationVector(2);
    configuration[3] = configurationVector(3);
    configuration[4] = configurationVector(4);
    configuration[5] = configurationVector(5);
    m_robot->setConfiguration(configuration);
  };

  const auto getDistance = [](const Eigen::VectorXd& source, const Eigen::VectorXd& target) -> double {
    // const Eigen::VectorXd difference = target - source;
    // return std::sqrt(difference(0) * difference(0) + difference(1) * difference(1) + difference(2) * difference(2));
    return (target - source).norm();
  };

  Eigen::VectorXd target(6);
  target(0) = m_ui.doubleSpinBox_X->value();
  target(1) = m_ui.doubleSpinBox_Y->value();
  target(2) = m_ui.doubleSpinBox_Z->value();
  target(3) = math::degreesToRadians(m_ui.doubleSpinBox_A->value());
  target(4) = math::degreesToRadians(m_ui.doubleSpinBox_B->value());
  target(5) = math::degreesToRadians(m_ui.doubleSpinBox_C->value());

  const double epsilon = 0.001;
  const double descentSpeed = 0.01;
  int epochs = 0;

  Eigen::VectorXd source = getEndEffectorPosition();
  while (getDistance(source, target) > epsilon) {
    {
      epochs++;
      std::cout << "epoch number: " << epochs << std::endl;
      std::cout << "source: (" << source(0) << ", " << source(1) << ", " << source(2) << ", " << source(3) << ", " << source(4) << ", " << source(5) << ")"
                << std::endl;
      std::cout << "target: (" << target(0) << ", " << target(1) << ", " << target(2) << ", " << target(3) << ", " << target(4) << ", " << target(5) << ")"
                << std::endl;
      std::cout << "distance: " << getDistance(source, target) << std::endl;
      std::cout << std::endl;
    }

    const Eigen::MatrixXd jacobian = m_robot->computeJacobian();
    // std::cout << std::endl << jacobian << std::endl << std::endl;
    const Eigen::MatrixXd inverseJacobian = m_robot->computeInverseJacobian(jacobian);
    // std::cout << std::endl << inverseJacobian << std::endl << std::endl;

    //    Eigen::VectorXd difference(3);
    //    difference(0) = target(0) - source(0);
    //    difference(1) = target(1) - source(1);
    //    difference(2) = target(2) - source(2);
    //    const Eigen::VectorXd delta = inverseJacobian * difference;

    const Eigen::VectorXd delta = inverseJacobian * (target - source);

    Eigen::VectorXd configuration = getAxisConfiguration();
    configuration = configuration + descentSpeed * delta;

    setAxisConfiguration(configuration);

    source = getEndEffectorPosition();

    emit ggg();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}
