#include "stdafx.h"
#include "main_window.h"
#include <visualization/coordinate_frame_gizmo.h>
#include <math/math.h>

kinverse::simulator::MainWindow::MainWindow(QWidget* parent) : QMainWindow{ parent } {
  m_ui.setupUi(this);

  initializeVisualizer();
  initializeRobot();
  initializeGizmos();

  QObject::connect(m_ui.doubleSpinBox_A1, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateRobot);
  QObject::connect(m_ui.doubleSpinBox_A2, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateRobot);
  QObject::connect(m_ui.doubleSpinBox_A3, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateRobot);
  QObject::connect(m_ui.doubleSpinBox_A4, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateRobot);
  QObject::connect(m_ui.doubleSpinBox_A5, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateRobot);
  QObject::connect(m_ui.doubleSpinBox_A6, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::updateRobot);
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
  m_robot = std::make_shared<core::Robot>();

  const std::vector<core::DenavitHartenbergParameters> dhTable{
    { core::JointType::Revolute, -400.0, 0.0, 180.0, M_PI_2 },   //
    { core::JointType::Revolute, 0.0, 0.0, 600.0, 0.0 },         //
    { core::JointType::Revolute, 0.0, -M_PI_2, 120.0, M_PI_2 },  //
    { core::JointType::Revolute, -620.0, 0.0, 0.0, -M_PI_2 },    //
    { core::JointType::Revolute, 0.0, 0.0, 0.0, M_PI_2 },        //
    { core::JointType::Revolute, -115, 0.0, 0.0, 0.0 }           //
  };
  m_robot->setDHTable(dhTable);

  const std::vector<core::JointConstraints> jointConstraints{
    { math::degreesToRadians(154.0), math::degreesToRadians(-155.0), math::degreesToRadians(155.0) },  //
    { math::degreesToRadians(154.0), math::degreesToRadians(-180.0), math::degreesToRadians(65.0) },   //
    { math::degreesToRadians(228.0), math::degreesToRadians(-15.0), math::degreesToRadians(158.0) },   //
    { math::degreesToRadians(343.0), math::degreesToRadians(-350.0), math::degreesToRadians(350.0) },  //
    { math::degreesToRadians(384.0), math::degreesToRadians(-130.0), math::degreesToRadians(130.0) },  //
    { math::degreesToRadians(721.0), math::degreesToRadians(-350.0), math::degreesToRadians(350.0) }   //
  };
  m_robot->setJointConstraints(jointConstraints);

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
}
