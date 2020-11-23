#pragma once

#ifdef EXPORT_KINVERSE_CORE
#define KINVERSE_CORE_API __declspec(dllexport)
#else
#define KINVERSE_CORE_API __declspec(dllimport)
#endif
