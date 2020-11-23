#pragma once

#ifdef EXPORT_KINVERSE_MATH
#define KINVERSE_MATH_API __declspec(dllexport)
#else
#define KINVERSE_MATH_API __declspec(dllimport)
#endif
