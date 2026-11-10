//
// Created by Gmail on 2023/11/15.
//
#include "time_measure.h"
#include "string.h"
#include "log.h"
#include "sorg_utils.h"

time_measure_probe_t time_measure_probe_list[TIME_MEASURE_MAX_PROBE];

/*
 * change this when migrating
 */
uint64_t time_measure_get_tick(){
    return tick_get_us();
}

uint32_t time_measure_get_second(){
    return HAL_GetTick()/1000;
}

void time_measure_puts(const char *context){
    LOGLN_INFO("time_trace", "%s", context);
}


void time_measure_init_probe(const char *name, time_measure_probe_t *probe){
    strcpy(probe->name, name);
    probe->call_count = 0;
    probe->average_tick = 0;
    probe->total_tick = 0;
}

void time_measure_search_probe(const char *name, time_measure_probe_t **probe){
    uint32_t i;
    for (i = 0; i < TIME_MEASURE_MAX_PROBE; ++i) {
        if (strcmp(name, time_measure_probe_list[i].name) == 0){
            *probe = &time_measure_probe_list[i];
            return;
        } else if(time_measure_probe_list[i].name[0] == 0){
            time_measure_init_probe(name, &time_measure_probe_list[i]);
            *probe = &time_measure_probe_list[i];
            return;
        }
    }
}

void time_measure_update_probe(time_measure_probe_t *probe){
    probe->average_tick = probe->total_tick / (double)probe->call_count;
    probe->average_call_second = (double)probe->call_count / (double)time_measure_get_second();
}

void time_measure_record_start(const char *probe_name){
    time_measure_probe_t *probe = NULL;
    time_measure_search_probe(probe_name, &probe);
    probe->start_tick = time_measure_get_tick();
}

void time_measure_record_stop(const char *probe_name){
    uint64_t probe_tick;
    time_measure_probe_t *probe = NULL;
    time_measure_search_probe(probe_name, &probe);
    probe_tick = time_measure_get_tick() - probe->start_tick;
    probe->total_tick += (double)probe_tick;
    probe->call_count += 1;
    time_measure_update_probe(probe);
}

void time_measure_summary(){
    int i;
    char output_str_buf[128];
    time_measure_puts("********Probes*********");
    for (i = 0; i < TIME_MEASURE_MAX_PROBE; ++i) {
        if (time_measure_probe_list[i].name[0] == 0)
            break;
        memset(output_str_buf, 0, 64);
        sprintf(output_str_buf, "->%s:", time_measure_probe_list[i].name);
        time_measure_puts(output_str_buf);
        memset(output_str_buf, 0, 64);
        sprintf(output_str_buf, "average_tick:%f ### average_call:%f ### total_call:%lld", time_measure_probe_list[i].average_tick, time_measure_probe_list[i].average_call_second, time_measure_probe_list[i].call_count);
        time_measure_puts(output_str_buf);

    }
    time_measure_puts("***********************");
}
