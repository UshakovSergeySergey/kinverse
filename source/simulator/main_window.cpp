#include "stdafx.h"
#include "main_window.h"
#include <kinverse/visualization/coordinate_frame_gizmo.h>
#include <kinverse/math/math.h>
#include <kinverse/factory/robot_factory.h>

kinverse::simulator::MainWindow::MainWindow(QWidget* parent) : QMainWindow{ parent } {
  m_ui.setupUi(this);

  initializeVisualizer();
  initializeRobot();
  initializeIKSolver();
  initializeGizmos();
  initializeProgressIcon();

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

  // change end effector transform
  QObject::connect(m_ui.doubleSpinBox_X, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onFindAnalyticalSolution);
  QObject::connect(m_ui.doubleSpinBox_Y, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onFindAnalyticalSolution);
  QObject::connect(m_ui.doubleSpinBox_Z, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onFindAnalyticalSolution);
  QObject::connect(m_ui.doubleSpinBox_A, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onFindAnalyticalSolution);
  QObject::connect(m_ui.doubleSpinBox_B, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onFindAnalyticalSolution);
  QObject::connect(m_ui.doubleSpinBox_C, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onFindAnalyticalSolution);

  // solve IK analytically
  QObject::connect(m_ui.pushButton_solveInverseKinematics, &QPushButton::pressed, this, &MainWindow::onFindAnalyticalSolution);

  // update list of IK solutions
  qRegisterMetaType<std::vector<std::vector<double>>>("std::vector<std::vector<double>>");
  QObject::connect(this, &MainWindow::solvedIKSignal, this, &MainWindow::onAnalyticalSolutionFound);

  // select solution
  QObject::connect(m_ui.tableWidget_ikSolutions->selectionModel(), &QItemSelectionModel::selectionChanged, this, &MainWindow::onSolutionSelected);

  onAxisValueChanged();
  onRobotAppearanceChanged();
  onFindAnalyticalSolution();
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

  const Eigen::VectorXd configuration = Eigen::VectorXd::Zero(6);
  m_robot->setConfiguration(configuration);
}

void kinverse::simulator::MainWindow::initializeIKSolver() {
  m_ikSolver = std::make_shared<core::AnalyticalSolver>(m_robot);
}

void kinverse::simulator::MainWindow::initializeGizmos() {
  auto worldFrameGizmo = std::make_shared<visualization::CoordinateFrameGizmo>(nullptr, Eigen::Affine3d::Identity(), "world");
  m_kinverseVisualizer->addGizmo(worldFrameGizmo);

  m_kinematicDiagramGizmo = std::make_shared<visualization::KinematicDiagramGizmo>(nullptr, m_robot);
  m_kinverseVisualizer->addGizmo(m_kinematicDiagramGizmo);

  m_robotGizmo = std::make_shared<visualization::RobotGizmo>(nullptr, m_robot);
  m_kinverseVisualizer->addGizmo(m_robotGizmo);
}

void kinverse::simulator::MainWindow::initializeProgressIcon() {
  QMovie* movie = new QMovie(":/simulator-resources/loading-icon");
  if (movie->isValid()) {
    movie->setScaledSize(m_ui.label_progressIcon->size());
    m_ui.label_progressIcon->setMovie(movie);
    movie->start();
  } else {
    m_ui.label_progressIcon->setText("Processing...");
  }
}

void kinverse::simulator::MainWindow::onAxisValueChanged() const {
  const Eigen::VectorXd configuration = getGuiAxisValues();

  m_robot->setConfiguration(configuration);
  m_kinematicDiagramGizmo->updateRobotConfiguration();
  m_robotGizmo->updateRobotConfiguration();

  if (configurationViolatesConstraints(configuration)) {
    const Eigen::VectorXd clampedConfiguration = m_robot->getConfiguration();
    setGuiAxisValues(clampedConfiguration);
  }

  return;
  const Eigen::Affine3d endEffectorTransform = m_robot->getEndEffectorTransform();
  setGuiEndEffectorTransform(endEffectorTransform);
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

void kinverse::simulator::MainWindow::onFindAnalyticalSolution() {
  enableGui(false);
  m_thread = std::thread(&MainWindow::solveIK, this);
}

void kinverse::simulator::MainWindow::onAnalyticalSolutionFound(const std::vector<std::vector<double>>& solutions) {
  if (m_thread.joinable())
    m_thread.join();

  //@todo this is a dirty hack. we need to use Eigen::VectorXd everywhere
  std::vector<Eigen::VectorXd> convertedSolutions{};
  for (const auto& solution : solutions) {
    Eigen::VectorXd convertedSolution(solution.size());
    for (int i = 0; i < solution.size(); ++i)
      convertedSolution(i) = solution[i];
    convertedSolutions.push_back(convertedSolution);
  }

  updateListOfIKSolutionsGui(convertedSolutions);
  enableGui(true);
}

void kinverse::simulator::MainWindow::onSolutionSelected(const QItemSelection& selected, const QItemSelection&) {
  if (selected.indexes().empty())
    return;

  const auto selectedSolutionIndex = selected.indexes().front().row();
  const auto numberOfJoints = m_robot->getNumberOfJoints();

  Eigen::VectorXd configuration(numberOfJoints);
  for (auto jointCounter = 0u; jointCounter < numberOfJoints; ++jointCounter) {
    const auto item = m_ui.tableWidget_ikSolutions->item(selectedSolutionIndex, jointCounter);
    const auto str = item->text().toStdString();
    const double jointValue = math::degreesToRadians(std::stod(str));
    configuration(jointCounter) = jointValue;
  }

  setGuiAxisValues(configuration);

  m_robot->setConfiguration(configuration);
  m_robotGizmo->updateRobotConfiguration();
  m_kinematicDiagramGizmo->updateRobotConfiguration();
}

void kinverse::simulator::MainWindow::enableGui(bool enabled) const {
  m_ui.widget_guiControls->setEnabled(enabled);
  if (enabled) {
    m_ui.label_progressIcon->hide();
  } else {
    m_ui.label_progressIcon->show();
  }
}

Eigen::VectorXd kinverse::simulator::MainWindow::getGuiAxisValues() const {
  Eigen::VectorXd configuration(6);
  configuration << math::degreesToRadians(m_ui.doubleSpinBox_A1->value()), math::degreesToRadians(m_ui.doubleSpinBox_A2->value()),
      math::degreesToRadians(m_ui.doubleSpinBox_A3->value()), math::degreesToRadians(m_ui.doubleSpinBox_A4->value()),
      math::degreesToRadians(m_ui.doubleSpinBox_A5->value()), math::degreesToRadians(m_ui.doubleSpinBox_A6->value());
  return configuration;
}

void kinverse::simulator::MainWindow::setGuiAxisValues(const Eigen::VectorXd& configuration) const {
  m_ui.doubleSpinBox_A1->setValue(math::radiansToDegrees(configuration(0)));
  m_ui.doubleSpinBox_A2->setValue(math::radiansToDegrees(configuration(1)));
  m_ui.doubleSpinBox_A3->setValue(math::radiansToDegrees(configuration(2)));
  m_ui.doubleSpinBox_A4->setValue(math::radiansToDegrees(configuration(3)));
  m_ui.doubleSpinBox_A5->setValue(math::radiansToDegrees(configuration(4)));
  m_ui.doubleSpinBox_A6->setValue(math::radiansToDegrees(configuration(5)));
}

Eigen::Affine3d kinverse::simulator::MainWindow::getGuiEndEffectorTransform() const {
  const Eigen::Vector3d xyz{ m_ui.doubleSpinBox_X->value(), m_ui.doubleSpinBox_Y->value(), m_ui.doubleSpinBox_Z->value() };
  Eigen::Vector3d abc{ m_ui.doubleSpinBox_A->value(), m_ui.doubleSpinBox_B->value(), m_ui.doubleSpinBox_C->value() };
  abc = abc * M_PI / 180.0;
  return math::fromXYZABC(xyz, abc);
}

void kinverse::simulator::MainWindow::setGuiEndEffectorTransform(const Eigen::Affine3d& endEffectorTransform) const {
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

void kinverse::simulator::MainWindow::updateListOfIKSolutionsGui(const std::vector<Eigen::VectorXd>& solutions) const {
  const auto numberOfSolutions = solutions.size();
  const auto numberOfJoints = m_robot->getNumberOfJoints();

  // update solution table
  m_ui.tableWidget_ikSolutions->clear();
  m_ui.tableWidget_ikSolutions->setRowCount(numberOfSolutions);
  m_ui.tableWidget_ikSolutions->setColumnCount(numberOfJoints);

  for (auto solutionCounter = 0u; solutionCounter < numberOfSolutions; ++solutionCounter) {
    for (auto jointCounter = 0u; jointCounter < numberOfJoints; ++jointCounter) {
      const double jointValue = math::radiansToDegrees(solutions[solutionCounter][jointCounter]);
      m_ui.tableWidget_ikSolutions->setItem(solutionCounter, jointCounter, new QTableWidgetItem(std::to_string(jointValue).c_str()));
    }
  }

  // select current solution
  const auto computeDistance = [](const Eigen::VectorXd& conf1, const Eigen::VectorXd& conf2) -> double { return (conf1 - conf2).norm(); };

  double minDistance = std::numeric_limits<double>::max();
  int currentConfigurationIndex = -1;
  const auto currentConfiguration = m_robot->getConfiguration();
  for (auto solutionCounter = 0u; solutionCounter < numberOfSolutions; ++solutionCounter) {
    const double currentDistance = computeDistance(currentConfiguration, solutions[solutionCounter]);
    if (currentDistance < minDistance) {
      minDistance = currentDistance;
      currentConfigurationIndex = solutionCounter;
    }
  }

  m_ui.tableWidget_ikSolutions->selectRow(currentConfigurationIndex);
}

bool kinverse::simulator::MainWindow::configurationViolatesConstraints(const Eigen::VectorXd& configuration) const {
  bool violatesConstraints = false;

  const auto constraints = m_robot->getJointConstraints();
  for (auto axisCounter = 0u; axisCounter < configuration.size(); ++axisCounter) {
    violatesConstraints |= constraints[axisCounter].violatesRangeConstraint(configuration[axisCounter]);
  }

  return violatesConstraints;
}

void kinverse::simulator::MainWindow::solveIK() {
  const bool takeTransformFromGui = m_ui.radioButton_endEffectorFromGUI->isChecked();

  Eigen::Affine3d endEffectorTransform = m_robot->getEndEffectorTransform();
  if (takeTransformFromGui)
    endEffectorTransform = getGuiEndEffectorTransform();

  try {
    const std::vector<std::vector<double>> solutions = m_ikSolver->solve(endEffectorTransform);
    emit solvedIKSignal(solutions);
    return;
  } catch (std::exception& exception) {
    std::cerr << "Failed to solve IK:" << std::endl;
    std::cerr << exception.what() << std::endl;
  } catch (...) {
    std::cerr << "Failed to solve IK:" << std::endl;
    std::cerr << "Caught unknown exception!" << std::endl;
  }

  emit solvedIKSignal({});
}
