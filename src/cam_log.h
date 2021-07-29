#ifndef _CAM_LOG_H
#define _CAM_LOG_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <stdlib.h>
#ifndef TAG
#define TAG "CAM"
#endif
//  #define ENABLE_CAM_DEBUG
// #define ENABLE_CAM_WARN
// #define ENABLE_CAM_INFO
// #define ENABLE_CAM_ERROR
// #define ENABLE_CAM_TRACE

	extern void cam_printf(const char *format, ...);
/**
 * @brief Debug level logging macro.
 *
 * Macro to expose function, line number as well as desired log message.
 */
#ifdef ENABLE_CAM_DEBUG
#define CAM_DEBUG(...)                                       \
	{                                                        \
		cam_printf("%s: ", TAG);                             \
		cam_printf("DEBUG:   %s L#%d ", __func__, __LINE__); \
		cam_printf(__VA_ARGS__);                             \
		cam_printf("\n");                                    \
	}
#else
#define CAM_DEBUG(...)
#endif

/**
 * @brief Debug level trace logging macro.
 *
 * Macro to print message function entry and exit
 */
#ifdef ENABLE_CAM_TRACE
#define FUNC_ENTRY                                                  \
	{                                                               \
		cam_printf("FUNC_ENTRY:   %s L#%d \n", __func__, __LINE__); \
	}
#define FUNC_EXIT                                                  \
	{                                                              \
		cam_printf("FUNC_EXIT:   %s L#%d \n", __func__, __LINE__); \
	}
#define FUNC_EXIT_RC(x)                                                                \
	{                                                                                  \
		cam_printf("FUNC_EXIT:   %s L#%d Return Code : %d \n", __func__, __LINE__, x); \
		return x;                                                                      \
	}
#else
#define FUNC_ENTRY

#define FUNC_EXIT
#define FUNC_EXIT_RC(x) \
	{                   \
		return x;       \
	}
#endif

/**
 * @brief Info level logging macro.
 *
 * Macro to expose desired log message.  Info messages do not include automatic function names and line numbers.
 */
#ifdef ENABLE_CAM_INFO
#define CAM_INFO(...)              \
	{                              \
		cam_printf("[%s]: ", TAG); \
		cam_printf(__VA_ARGS__);   \
		cam_printf("\n\r");        \
	}
#else
#define CAM_INFO(...)
#endif

/**
 * @brief Warn level logging macro.
 *
 * Macro to expose function, line number as well as desired log message.
 */
#ifdef ENABLE_CAM_WARN
#define CAM_WARN(...)                                      \
	{                                                      \
		cam_printf("%s: ", TAG);                           \
		cam_printf("WARN:  %s L#%d ", __func__, __LINE__); \
		cam_printf(__VA_ARGS__);                           \
		cam_printf("\n\r");                                \
	}
#else
#define CAM_WARN(...)
#endif

/**
 * @brief Error level logging macro.
 *
 * Macro to expose function, line number as well as desired log message.
 */
#ifdef ENABLE_CAM_ERROR
#define CAM_ERROR(...)                                     \
	{                                                      \
		cam_printf("%s: ", TAG);                           \
		cam_printf("ERROR: %s L#%d ", __func__, __LINE__); \
		cam_printf(__VA_ARGS__);                           \
		cam_printf("\n\r");                                \
	}
#else
#define CAM_ERROR(...)
#endif

#ifdef __cplusplus
}
#endif

#endif // _CAM_LOG_H