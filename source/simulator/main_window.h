#pragma once

#include "ui_main_window.h"
#include <visualization/kinverse_visualizer.h>
#include <visualization/robot_gizmo.h>
#include <core/robot.h>

namespace kinverse {
  namespace simulator {

    class MainWindow : public QMainWindow {
      Q_OBJECT

     public:
      explicit MainWindow(QWidget* parent = Q_NULLPTR);
      virtual ~MainWindow() = default;

     private:
      void initializeVisualizer();
      void initializeRobot();
      void initializeGizmos();
      void updateGizmo();

      void updateRobot();

      Ui::MainWindow m_ui;
      visualization::KinverseVisualizer::Ptr m_kinverseVisualizer{ nullptr };
      visualization::RobotGizmo::Ptr m_robotGizmo{ nullptr };
      core::Robot::Ptr m_robot{ nullptr };
    };

  }  // namespace simulator
}  // namespace kinverse
