#pragma once

#ifdef EXPORT_KINVERSE_VISUALIZATION
#define KINVERSE_VISUALIZATION_API __declspec(dllexport)
#else
#define KINVERSE_VISUALIZATION_API __declspec(dllimport)
#endif
