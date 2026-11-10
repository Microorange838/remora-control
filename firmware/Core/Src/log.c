/*************************************************************
 * @file log.c
 * @author MicroOrange
 * @brief 
 * @date 2022/12/29
 *************************************************************/
#include "log.h"
#include "usart.h"

/**
 * list for all log devices
 */
OrangeLogDeviceList log_dev_list;


/**
 *
 */
#if (ORANGE_USE_TIMESTAMP == ORANGE_LOG_TIMESTAMP_ON)
const char *stamp_format = "%d-[%s][%s] ";
#else
const char *stamp_format = "[%s][%s] ";
#endif


#define USART_TIMEOUT 1000
/**
  *****************************************************************************
  * @brief  This function Transmit data via USART
  *
  * @param[in]	data    : data to be transmitted
  * @param[in]	dataLen : length of data to be transmitted
  *
  * @retval ERR_INVALID_HANDLE : in case the UART HW is not initalized yet
  * @retval others             : HAL status
  *****************************************************************************
  */
void logUsartTrans(char *data, uint32_t dataLen)
{

    HAL_UART_Transmit(&huart5, data, dataLen, USART_TIMEOUT);
}

void orange_log_init(){
    log_dev_list.max_log_level = LogLevelNone;
    log_dev_list.dev_amount = 0;
    OrangeLogDevice device;
    device.log_level = LogLevelTrace;
    device.log_puts = logUsartTrans;
    device.log_timestamp = HAL_GetTick;
    device.is_on = true;
    orange_log_device_init(device);
}


void orange_log_print_raw(OrangeLogLevel log_level, const char* format, ...){
#if (ORANGE_USE_LOG == ORANGE_LOG_ON)
    if (log_dev_list.max_log_level >= log_level){
        char str_buf[ORANGE_LOG_STR_BUFFER_SIZE];
        for (int i = 0; i < log_dev_list.dev_amount; ++i) {
            OrangeLogDevice *device = &log_dev_list.list[i];
            if (device->is_on){
                memset(str_buf, 0, ORANGE_LOG_STR_BUFFER_SIZE);
                va_list args;
                va_start(args, format);
                vsnprintf(str_buf, ORANGE_LOG_STR_BUFFER_SIZE, format, args);
                va_end(args);
                device->log_puts(str_buf, strlen(str_buf));
            }
        }
    }
#endif
}

void orange_log_println(OrangeLogLevel log_level, const char* tag, const char* format, ...){
#if (ORANGE_USE_LOG == ORANGE_LOG_ON)
    if (log_dev_list.max_log_level >= log_level){
        const char *log_letter = "";
        char str_buf[ORANGE_LOG_STR_BUFFER_SIZE];

        switch(log_level) {
            case LogLevelError:
                log_letter = "Error";
                break;
            case LogLevelWarn:
                log_letter = "Warn";
                break;
            case LogLevelInfo:
                log_letter = "Info";
                break;
            case LogLevelDebug:
                log_letter = "Debug";
                break;
            case LogLevelTrace:
                log_letter = "Trace";
                break;
            default:
                break;
        }

        for (int i = 0; i < log_dev_list.dev_amount; ++i) {
            OrangeLogDevice *device = &log_dev_list.list[i];
            if (device->is_on){
                memset(str_buf, 0, ORANGE_LOG_STR_BUFFER_SIZE);
                #if (ORANGE_USE_TIMESTAMP == ORANGE_LOG_TIMESTAMP_ON)
                snprintf(str_buf, ORANGE_LOG_STR_BUFFER_SIZE,
                        stamp_format,
                        device->log_timestamp(),
                        tag,
                        log_letter);
                #else
                snprintf(str_buf, ORANGE_LOG_STR_BUFFER_SIZE,
                        stamp_format,
                        tag,
                        log_letter);
                #endif
                device->log_puts(str_buf, strlen(str_buf));

                memset(str_buf, 0, ORANGE_LOG_STR_BUFFER_SIZE);
                va_list args;
                va_start(args, format);
                vsnprintf(str_buf, ORANGE_LOG_STR_BUFFER_SIZE, format, args);
                va_end(args);
                device->log_puts(str_buf, strlen(str_buf));

                device->log_puts("\r\n", 2);
            }
        }
    }
#endif
}

void orange_log_print(OrangeLogLevel log_level, const char* tag, const char* format, ...){
#if (ORANGE_USE_LOG)
    if (log_dev_list.max_log_level >= log_level){
        const char *log_letter = "";
        char str_buf[ORANGE_LOG_STR_BUFFER_SIZE];

        switch(log_level) {
            case LogLevelError:
                log_letter = "Error";
                break;
            case LogLevelWarn:
                log_letter = "Warn";
                break;
            case LogLevelInfo:
                log_letter = "Info";
                break;
            case LogLevelDebug:
                log_letter = "Debug";
                break;
            case LogLevelTrace:
                log_letter = "Trace";
                break;
            default:
                break;
        }

        for (int i = 0; i < log_dev_list.dev_amount; ++i) {
            OrangeLogDevice *device = &log_dev_list.list[i];
            if (device->is_on){
                memset(str_buf, 0, ORANGE_LOG_STR_BUFFER_SIZE);
#if (ORANGE_USE_TIMESTAMP == ORANGE_LOG_TIMESTAMP_ON)
                snprintf(str_buf, ORANGE_LOG_STR_BUFFER_SIZE,
                        stamp_format,
                        device->log_timestamp(),
                        tag,
                        log_letter);
#else
                snprintf(str_buf, ORANGE_LOG_STR_BUFFER_SIZE,
                         stamp_format,
                         tag,
                         log_letter);
#endif
                device->log_puts(str_buf, strlen(str_buf));

                memset(str_buf, 0, ORANGE_LOG_STR_BUFFER_SIZE);
                va_list args;
                va_start(args, format);
                vsnprintf(str_buf, ORANGE_LOG_STR_BUFFER_SIZE, format, args);
                va_end(args);
                device->log_puts(str_buf, strlen(str_buf));

            }
        }
    }
#endif
}


void orange_log_device_init(OrangeLogDevice device){
    log_dev_list.list[log_dev_list.dev_amount] = device;
    log_dev_list.dev_amount++;
    for (int i = 0; i < log_dev_list.dev_amount; ++i) {
        log_dev_list.max_log_level = MAX(log_dev_list.max_log_level,
                log_dev_list.list[i].log_level);

    }

}

void orange_log_mat(double *mat, uint32_t row, uint32_t col, uint8_t after_point,OrangeLogLevel log_level, const char* tag){
    orange_log_println(log_level, tag, "");
    uint32_t max_len=0,temp_len;
    char format_control[16] = {0};

    for (int i = 0; i < row * col; ++i) {
        temp_len = ceil(log10(mat[i]));
        if (max_len<temp_len)
            max_len = temp_len;
    }
    max_len += after_point;
    max_len += 2;
    sprintf(format_control, "%%%lu.%uf", max_len, after_point);
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < col; ++j) {
            orange_log_print_raw(log_level, format_control, mat[i*col+j]);
        }
        orange_log_print_raw(log_level, "\r\n");
    }
}



