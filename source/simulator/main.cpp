#include "stdafx.h"
#include "main_window.h"

int main(int argc, char** argv) {
  int exitCode = -1;

  try {
    QApplication application(argc, argv);

    QCoreApplication::setOrganizationName("Sergey Ushakov");
    QCoreApplication::setApplicationName("kinverse simulator");
    QCoreApplication::setApplicationVersion("0.0.1");

    auto mainWindow = std::make_shared<kinverse::simulator::MainWindow>();
    mainWindow->show();

    exitCode = application.exec();
  } catch (std::exception& exception) {
    std::cerr << "Exception caught:" << std::endl;
    std::cerr << exception.what() << std::endl;
    std::cerr << "Application will now quit!" << std::endl;
  } catch (...) {
    std::cerr << "Exception caught!" << std::endl;
    std::cerr << "Application will now quit!" << std::endl;
  }

  return exitCode;
}
