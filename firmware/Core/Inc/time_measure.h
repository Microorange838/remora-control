//
// Created by Gmail on 2023/11/15.
//

#ifndef REMORARL_TIME_MEASURE_H
#define REMORARL_TIME_MEASURE_H
#include "usart.h"

#define  TIME_MEASURE_MAX_PROBE 128

typedef struct {
    char name[32];
    uint64_t start_tick;
    uint64_t call_count;
    double average_tick;
    double average_call_second;
    double total_tick;

} time_measure_probe_t;

void time_measure_record_start(const char *probe_name);
void time_measure_record_stop(const char *probe_name);
void time_measure_summary();

#define time_measure_func(probe_name, call_func) \
    do{                              \
    time_measure_record_start(probe_name);       \
    call_func;                                   \
    time_measure_record_stop(probe_name);\
    }while(0)
#endif //REMORARL_TIME_MEASURE_H
