/*************************************************************
 * @file motion_planner.h
 * @author MicroOrange
 * @brief 运动规划器层 反馈与速度
 * @date 2022.2.23
 *************************************************************/


#ifndef INC_113REBUILD_MOTION_PLANNER_H
#define INC_113REBUILD_MOTION_PLANNER_H
#include "fish_controller.h"

enum {
    MOTION_MODE_THREEDIMROTATE,
    MOTION_MODE_TEST1,
    MOTION_MODE_TEST2,
    MOTION_MODE_STOP,
    MOTION_MODE_NECKPITCH,
    MOTION_MODE_PID_ROLL,
    MOTION_MODE_DETACH_S1,
    MOTION_MODE_DETACH_S2,
    MOTION_MODE_FRONT,
    MOTION_MODE_FRONTPITCH,
    MOTION_MODE_REMOTE,
    MOTION_MODE_NETWORK1
};
void Motion_Init();


void Motion_SetStop(fish_t *remora);
void Motion_SetTest1(fish_t *remora);
void Motion_TestSetFreq(double freq);
void Motion_SetThreeDimRotate(double target_y, double target_p, double target_r);

void Motion_Update(fish_t *remora);

void Motion_ParaTest2(int para_index, double value);
void Motion_ParaThreeDimRotate(int para_index, double value);

void Motion_SetModeSimple(int mode);
#endif //INC_113REBUILD_MOTION_PLANNER_H
