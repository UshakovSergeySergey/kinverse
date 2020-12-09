#pragma once

#include "ui_main_window.h"
#include <kinverse/visualization/kinverse_visualizer.h>
#include <kinverse/visualization/robot_gizmo.h>
#include <kinverse/core/robot.h>

namespace kinverse {
  namespace simulator {

    class MainWindow : public QMainWindow {
      Q_OBJECT

     signals:
      void ggg();

     public:
      explicit MainWindow(QWidget* parent = Q_NULLPTR);
      virtual ~MainWindow() = default;

     private:
      void initializeVisualizer();
      void initializeRobot();
      void initializeGizmos();
      void updateGizmo();

      void updateRobot();
      void updateEndEffector();
      void solveIK();
      void fff();
      std::thread m_thread;

      Ui::MainWindow m_ui;
      visualization::KinverseVisualizer::Ptr m_kinverseVisualizer{ nullptr };
      visualization::RobotGizmo::Ptr m_robotGizmo{ nullptr };
      core::Robot::Ptr m_robot{ nullptr };
    };

  }  // namespace simulator
}  // namespace kinverse
