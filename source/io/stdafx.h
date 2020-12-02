﻿#pragma once

#pragma warning(disable : 4251)  // Warning C4251 class needs to have dll - interface to be used by clients of class
#pragma warning(disable : 4275)  // Warning C4275 non dll - interface class used as base for dll - interface class

#pragma warning(push)

#include <memory>
#include <vector>
#include <filesystem>

#include <Eigen/Geometry>

#include <vtkPLYReader.h>
#include <vtkOBJReader.h>
#include <vtkSTLReader.h>

#pragma warning(pop)
