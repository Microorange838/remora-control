/*************************************************************
 * @file log.h
 * @author MicroOrange
 * @brief 
 * @date 2022/12/29
 *************************************************************/

#ifndef ORANGE_LOG_H
#define ORANGE_LOG_H


#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include "string.h"
#include "stdbool.h"
#include "sorg_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ORANGE_LOG_ON 1
#define ORANGE_LOG_OFF 0
#define ORANGE_USE_LOG 1

#define ORANGE_LOG_TIMESTAMP_ON 1
#define ORANGE_LOG_TIMESTAMP_OFF 0
#define ORANGE_USE_TIMESTAMP 1

#define ORANGE_LOG_STR_BUFFER_SIZE 256
#define ORANGE_LOG_DEV_BUFFER_SIZE 3

typedef enum {
    LogLevelNone = 1,
    LogLevelError = 2,
    LogLevelWarn = 3,
    LogLevelInfo = 4,
    LogLevelDebug = 5,
    LogLevelTrace = 6,
} OrangeLogLevel;

typedef void (*OrangeLogPuts)(char *data, uint32_t len);
typedef uint32_t (*OrangeLogGetTimeStamp)(void);

typedef struct {
    OrangeLogLevel log_level;
    OrangeLogPuts log_puts;
    OrangeLogGetTimeStamp log_timestamp;
    bool is_on;
} OrangeLogDevice;

typedef struct {
    uint32_t dev_amount;
    OrangeLogDevice list[ORANGE_LOG_DEV_BUFFER_SIZE];
    OrangeLogLevel max_log_level;
} OrangeLogDeviceList;


void orange_log_println(OrangeLogLevel log_level, const char* tag, const char* format, ...);
void orange_log_print_raw(OrangeLogLevel log_level, const char* format, ...);
void orange_log_print(OrangeLogLevel log_level, const char* tag, const char* format, ...);
void orange_log_mat(double *mat, uint32_t row, uint32_t col, uint8_t after_point,OrangeLogLevel log_level, const char* tag);

/**
 * register a log device to orange_log
 * @param device The device to register
 */
void orange_log_device_init(OrangeLogDevice device);

/**
 * init orange log
 */
void orange_log_init();

/** Log raw
 *
 * @param      tag     The application tag
 * @param      format  The format
 * @param      ...     VA Args
 */
#define LOGRAW_ERROR(format, ...) \
    orange_log_print_raw(LogLevelError, format, ##__VA_ARGS__)
#define LOGRAW_WARN(format, ...) \
    orange_log_print_raw(LogLevelWarn, format, ##__VA_ARGS__)
#define LOGRAW_INFO(format, ...) \
    orange_log_print_raw(LogLevelInfo, format, ##__VA_ARGS__)
#define LOGRAW_DEBUG(format, ...) \
    orange_log_print_raw(LogLevelDebug, format, ##__VA_ARGS__)
#define LOGRAW_TRACE(format, ...) \
    orange_log_print_raw(LogLevelTrace, format, ##__VA_ARGS__)

/** Log with new line
 *
 * @param      tag     The application tag
 * @param      format  The format
 * @param      ...     VA Args
 */
#define LOGLN_ERROR(tag, format, ...) \
    orange_log_println(LogLevelError, tag, format, ##__VA_ARGS__)
#define LOGLN_WARN(tag, format, ...) \
    orange_log_println(LogLevelWarn, tag, format, ##__VA_ARGS__)
#define LOGLN_INFO(tag, format, ...) \
    orange_log_println(LogLevelInfo, tag, format, ##__VA_ARGS__)
#define LOGLN_DEBUG(tag, format, ...) \
    orange_log_println(LogLevelDebug, tag, format, ##__VA_ARGS__)
#define LOGLN_TRACE(tag, format, ...) \
    orange_log_println(LogLevelTrace, tag, format, ##__VA_ARGS__)

/** Log
 *
 * @param      tag     The application tag
 * @param      format  The format
 * @param      ...     VA Args
 */
#define LOG_ERROR(tag, format, ...) \
    orange_log_print(LogLevelError, tag, format, ##__VA_ARGS__)
#define LOG_WARN(tag, format, ...) \
    orange_log_print(LogLevelWarn, tag, format, ##__VA_ARGS__)
#define LOG_INFO(tag, format, ...) \
    orange_log_print(LogLevelInfo, tag, format, ##__VA_ARGS__)
#define LOG_DEBUG(tag, format, ...) \
    orange_log_print(LogLevelDebug, tag, format, ##__VA_ARGS__)
#define LOG_TRACE(tag, format, ...) \
    orange_log_print(LogLevelTrace, tag, format, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif //orange_log_h
