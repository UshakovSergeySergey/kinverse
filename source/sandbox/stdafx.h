#pragma once

#pragma warning(disable : 4251)  // Warning C4251 class needs to have dll - interface to be used by clients of class
#pragma warning(disable : 4275)  // Warning C4275 non dll - interface class used as base for dll - interface class

#pragma warning(push)

#define _USE_MATH_DEFINES
#include <cmath>

#include <memory>
#include <vector>
#include <array>
#include <thread>

#include <Eigen/Geometry>

#pragma warning(pop)
