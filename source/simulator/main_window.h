#pragma once

#include "ui_main_window.h"
#include <kinverse/visualization/kinverse_visualizer.h>
#include <kinverse/visualization/robot_gizmo.h>
#include <kinverse/visualization/kinematic_diagram_gizmo.h>
#include <kinverse/core/robot.h>

namespace kinverse {
  namespace simulator {

    class MainWindow : public QMainWindow {
      Q_OBJECT

     signals:
      void solvedIKSignal();

     public:
      explicit MainWindow(QWidget* parent = Q_NULLPTR);
      virtual ~MainWindow() = default;

     private:
      void initializeVisualizer();
      void initializeRobot();
      void initializeGizmos();
      void initializeProgressIcon();

      void onAxisValueChanged() const;
      void onRobotAppearanceChanged();
      void onFindAnalyticalSolution();
      void onAnalyticalSolutionFound();

      void enableGui(bool enabled) const;
      std::vector<double> extractRobotConfigurationFromGui() const;
      bool configurationViolatesConstraints(const std::vector<double>& configuration) const;
      void updateAxesValuesGui(const std::vector<double>& configuration) const;
      void updateEndEffectorTransformGui() const;

      void solveIK();

      Ui::MainWindow m_ui;
      visualization::KinverseVisualizer::Ptr m_kinverseVisualizer{ nullptr };
      visualization::RobotGizmo::Ptr m_robotGizmo{ nullptr };
      visualization::KinematicDiagramGizmo::Ptr m_kinematicDiagramGizmo{ nullptr };
      core::Robot::Ptr m_robot{ nullptr };
      bool m_showKinematicDiagram{ true };
      std::thread m_thread{};
    };

  }  // namespace simulator
}  // namespace kinverse
