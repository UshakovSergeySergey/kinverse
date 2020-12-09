#include "stdafx.h"
#include "main_window.h"
#include <kinverse/visualization/coordinate_frame_gizmo.h>
#include <kinverse/math/math.h>
#include <kinverse/core/robot_factory.h>

kinverse::simulator::MainWindow::MainWindow(QWidget* parent) : QMainWindow{ parent } {
  m_ui.setupUi(this);

  initializeVisualizer();
  initializeRobot();
  initializeGizmos();

  // m_ui.doubleSpinBox_A2->setValue(-90.0);
  // m_ui.doubleSpinBox_A3->setValue(90.0);

  m_ui.doubleSpinBox_A1->setValue(22.854);
  m_ui.doubleSpinBox_A2->setValue(-80.0);
  m_ui.doubleSpinBox_A3->setValue(80.0);
  m_ui.doubleSpinBox_A4->setValue(0.073);
  m_ui.doubleSpinBox_A5->setValue(22.879);
  m_ui.doubleSpinBox_A6->setValue(119.070);
  {
    const Eigen::Vector3d xyz{ 930.8209, -392.3756, 1066.1743 };
    const Eigen::Vector3d abcSergey{ math::degreesToRadians(157.1152), math::degreesToRadians(-112.8790), math::degreesToRadians(60.9208) };
    const Eigen::Vector3d abcSergeyEqualViktor{ math::degreesToRadians(79.3494), math::degreesToRadians(-153.3355), math::degreesToRadians(64.2112) };
    const Eigen::Vector3d abcViktor{ math::degreesToRadians(-100.651), math::degreesToRadians(-26.664), math::degreesToRadians(-115.788) };
    const Eigen::Affine3d sergey = kinverse::math::fromXYZABC(xyz, abcSergeyEqualViktor);
    const Eigen::Affine3d viktor = kinverse::math::fromXYZABC(xyz, abcViktor);

    std::cout << sergey.isApprox(viktor) << std::endl;
    std::cout << "sergey" << std::endl;
    std::cout << sergey.rotation() << std::endl;
    std::cout << "viktor" << std::endl;
    std::cout << viktor.rotation() << std::endl;
    auto sergeyGizmo = std::make_shared<visualization::CoordinateFrameGizmo>(nullptr, sergey, "sergey", 1, std::array<std::string, 3>{ "xs", "ys", "zs" });
    auto viktorGizmo = std::make_shared<visualization::CoordinateFrameGizmo>(nullptr, viktor, "viktor", 1, std::array<std::string, 3>{ "xv", "yv", "zv" });
    m_kinverseVisualizer->addGizmo(sergeyGizmo);
    m_kinverseVisualizer->addGizmo(viktorGizmo);
  }

  QObject::connect(m_ui.doubleSpinBox_A1, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateRobot);
  QObject::connect(m_ui.doubleSpinBox_A2, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateRobot);
  QObject::connect(m_ui.doubleSpinBox_A3, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateRobot);
  QObject::connect(m_ui.doubleSpinBox_A4, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateRobot);
  QObject::connect(m_ui.doubleSpinBox_A5, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateRobot);
  QObject::connect(m_ui.doubleSpinBox_A6, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateRobot);

  updateRobot();

  // QObject::connect(m_ui.doubleSpinBox_X, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateEndEffector);
  // QObject::connect(m_ui.doubleSpinBox_Y, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateEndEffector);
  // QObject::connect(m_ui.doubleSpinBox_Z, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateEndEffector);
  // QObject::connect(m_ui.doubleSpinBox_A, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateEndEffector);
  // QObject::connect(m_ui.doubleSpinBox_B, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateEndEffector);
  // QObject::connect(m_ui.doubleSpinBox_C, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateEndEffector);

  // QObject::connect(this, &MainWindow::ggg, this, &MainWindow::fff);
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
  m_robot = core::RobotFactory::create(core::RobotType::KukaKR5Arc);

  const std::vector<double> configuration{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  m_robot->setConfiguration(configuration);
}

void kinverse::simulator::MainWindow::initializeGizmos() {
  auto worldFrameGizmo = std::make_shared<visualization::CoordinateFrameGizmo>(nullptr, Eigen::Affine3d::Identity(), "world");
  m_kinverseVisualizer->addGizmo(worldFrameGizmo);

  m_robotGizmo = std::make_shared<visualization::RobotGizmo>(nullptr, m_robot);
  m_kinverseVisualizer->addGizmo(m_robotGizmo);
}

void kinverse::simulator::MainWindow::updateGizmo() {
  {
    // const double width = m_ui.doubleSpinBox->value();
    // const double height = m_ui.doubleSpinBox_2->value();
    // const double depth = m_ui.doubleSpinBox_3->value();
    // m_cube->setWidth(width);
    // m_cube->setHeight(height);
    // m_cube->setDepth(depth);

    // const int c = static_cast<int>(m_ui.doubleSpinBox->value() * 255.0 / 100.0);
    // m_cube->setColor(visualization::Color{ c, 0, 0, 255 });

    // Eigen::Affine3d transform;
    // transform = Eigen::Translation3d(m_ui.doubleSpinBox->value(), 0.0, 0.0);
    // m_cube->setTransform(transform);
  }

  {
    // const double height = m_ui.doubleSpinBox->value();
    // const double radius = m_ui.doubleSpinBox_2->value();
    // m_cylinder->setHeight(height);
    // m_cylinder->setRadius(radius);

    // const int c = static_cast<int>(m_ui.doubleSpinBox->value() * 255.0 / 100.0);
    // m_cylinder->setColor(visualization::Color{ c, 0, 0, 255 });

    // Eigen::Affine3d transform;
    // transform = Eigen::Translation3d(m_ui.doubleSpinBox->value(), 0.0, 0.0);
    // m_cylinder->setTransform(transform);
  }

  {
    // const int c = static_cast<int>(m_ui.doubleSpinBox->value() * 255.0 / 100.0);
    // m_text3d->setColor(visualization::Color{ c, 0, 0, 255 });

    // const std::string text = std::to_string(m_ui.doubleSpinBox->value());
    // m_text3d->setText(text);

    // const double fontSize = m_ui.doubleSpinBox->value();
    // m_text3d->setFontSize(fontSize);

    // Eigen::Affine3d transform;
    // transform = Eigen::Translation3d(m_ui.doubleSpinBox->value(), 0.0, 0.0);
    // m_text3d->setTransform(transform);

    // const bool faceTowardsCamera = m_ui.checkBox->isChecked();
    // m_text3d->setFaceTowardsCamera(faceTowardsCamera);

    // const bool viewportConstScale = m_ui.checkBox_2->isChecked();
    // m_text3d->setViewportConstScale(viewportConstScale);
  }

  {
    // Eigen::Affine3d transform;
    // transform = Eigen::Translation3d(m_ui.doubleSpinBox->value(), 0.0, 0.0);
    // m_coordinateFrame->setTransform(transform);

    // const std::string caption = std::to_string(m_ui.doubleSpinBox->value());
    // m_coordinateFrame->setCaption(caption);

    // const double scale = m_ui.doubleSpinBox->value();
    // m_coordinateFrame->setScale(scale);

    // std::array<std::string, 3> labels{
    //  //
    //  std::to_string(m_ui.doubleSpinBox->value()),    //
    //  std::to_string(m_ui.doubleSpinBox_2->value()),  //
    //  std::to_string(m_ui.doubleSpinBox_3->value())   //
    //};
    // m_coordinateFrame->setAxesLabels(labels);
  }

  {
    // Eigen::Affine3d transform;
    // transform = Eigen::Translation3d(m_ui.doubleSpinBox->value(), 0.0, 0.0);
    // m_revoluteJoint->setTransform(transform);

    // const unsigned int index = static_cast<unsigned int>(std::floor(m_ui.doubleSpinBox->value()));
    // m_revoluteJoint->setJointIndex(index);
  }
}

void kinverse::simulator::MainWindow::updateRobot() {
  const std::vector<double> configuration{ math::degreesToRadians(m_ui.doubleSpinBox_A1->value()), math::degreesToRadians(m_ui.doubleSpinBox_A2->value()),
                                           math::degreesToRadians(m_ui.doubleSpinBox_A3->value()), math::degreesToRadians(m_ui.doubleSpinBox_A4->value()),
                                           math::degreesToRadians(m_ui.doubleSpinBox_A5->value()), math::degreesToRadians(m_ui.doubleSpinBox_A6->value()) };

  m_robot->setConfiguration(configuration);

  const std::vector<double> clampedConfiguration = m_robot->getConfiguration();

  const auto constraints = m_robot->getJointConstraints();

  if (constraints[0].violatesRangeConstraint(configuration[0]))
    m_ui.doubleSpinBox_A1->setValue(math::radiansToDegrees(clampedConfiguration[0]));
  if (constraints[1].violatesRangeConstraint(configuration[1]))
    m_ui.doubleSpinBox_A2->setValue(math::radiansToDegrees(clampedConfiguration[1]));
  if (constraints[2].violatesRangeConstraint(configuration[2]))
    m_ui.doubleSpinBox_A3->setValue(math::radiansToDegrees(clampedConfiguration[2]));
  if (constraints[3].violatesRangeConstraint(configuration[3]))
    m_ui.doubleSpinBox_A4->setValue(math::radiansToDegrees(clampedConfiguration[3]));
  if (constraints[4].violatesRangeConstraint(configuration[4]))
    m_ui.doubleSpinBox_A5->setValue(math::radiansToDegrees(clampedConfiguration[4]));
  if (constraints[5].violatesRangeConstraint(configuration[5]))
    m_ui.doubleSpinBox_A6->setValue(math::radiansToDegrees(clampedConfiguration[5]));

  m_robotGizmo->setConfiguration(configuration);

  {
    const Eigen::Affine3d endEffectorTransform = m_robot->getLinkCoordinateFrames().back();

    Eigen::Vector3d xyz;
    Eigen::Vector3d abc;
    math::toXYZABC(endEffectorTransform, xyz, abc);
    abc = abc * 180.0 / M_PI;

    m_ui.doubleSpinBox_X->setValue(xyz.x());
    m_ui.doubleSpinBox_Y->setValue(xyz.y());
    m_ui.doubleSpinBox_Z->setValue(xyz.z());

    m_ui.doubleSpinBox_A->setValue(abc.x());
    m_ui.doubleSpinBox_B->setValue(abc.y());
    m_ui.doubleSpinBox_C->setValue(abc.z());
  }
}

void kinverse::simulator::MainWindow::updateEndEffector() {
  m_thread = std::thread(&MainWindow::solveIK, this);
}

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

void kinverse::simulator::MainWindow::fff() {
  m_robotGizmo->setConfiguration(m_robot->getConfiguration());
}
