#pragma once

#include "ui_main_window.h"
#include <kinverse/visualization/kinverse_visualizer.h>
#include <kinverse/visualization/robot_gizmo.h>
#include <kinverse/visualization/kinematic_diagram_gizmo.h>
#include <kinverse/core/robot.h>
#include <kinverse/core/analytical_solver.h>

namespace kinverse {
  namespace simulator {

    class MainWindow : public QMainWindow {
      Q_OBJECT

     signals:
      void solvedIKSignal(const std::vector<std::vector<double>>& solutions);

     public:
      explicit MainWindow(QWidget* parent = Q_NULLPTR);
      virtual ~MainWindow() = default;

     private:
      void initializeVisualizer();
      void initializeRobot();
      void initializeIKSolver();
      void initializeGizmos();
      void initializeProgressIcon();

      void onAxisValueChanged() const;
      void onRobotAppearanceChanged();
      void onFindAnalyticalSolution();
      void onAnalyticalSolutionFound(const std::vector<std::vector<double>>& solutions);
      void onSolutionSelected(const QItemSelection& selected, const QItemSelection& deselected);

      void enableGui(bool enabled) const;
      std::vector<double> extractRobotConfigurationFromGui() const;
      bool configurationViolatesConstraints(const std::vector<double>& configuration) const;
      void updateAxesValuesGui(const std::vector<double>& configuration) const;
      void updateEndEffectorTransformGui() const;
      void updateListOfIKSolutionsGui(const std::vector<std::vector<double>>& solutions) const;

      void solveIK();

      Ui::MainWindow m_ui;

      visualization::KinverseVisualizer::Ptr m_kinverseVisualizer{ nullptr };
      visualization::RobotGizmo::Ptr m_robotGizmo{ nullptr };
      visualization::KinematicDiagramGizmo::Ptr m_kinematicDiagramGizmo{ nullptr };

      core::Robot::Ptr m_robot{ nullptr };
      core::AnalyticalSolver::Ptr m_ikSolver{ nullptr };
      bool m_showKinematicDiagram{ true };
      std::thread m_thread{};
    };

  }  // namespace simulator
}  // namespace kinverse
