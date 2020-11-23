#pragma once

#pragma warning(disable : 4251)  // Warning C4251 class needs to have dll - interface to be used by clients of class
#pragma warning(disable : 4275)  // Warning C4275 non dll - interface class used as base for dll - interface class

#pragma warning(push)

#include "vtkAutoInit.h"
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);

#include <memory>
#include <vector>
#include <array>
#include <string>
#include <thread>

#define _USE_MATH_DEFINES
#include <cmath>

#include <Eigen/Geometry>

#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkNamedColors.h>
#include <vtkRenderWindow.h>
#include <vtkAxesActor.h>
#include <vtkProp.h>
#include <vtkTransform.h>
#include <vtkCaptionActor2D.h>
#include <vtkCylinderSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkCubeSource.h>
#include <vtkTextActor3D.h>
#include <vtkTextProperty.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>

#pragma warning(pop)
