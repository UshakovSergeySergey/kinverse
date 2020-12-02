#pragma once

#ifdef EXPORT_KINVERSE_IO
#define KINVERSE_IO_API __declspec(dllexport)
#else
#define KINVERSE_IO_API __declspec(dllimport)
#endif
