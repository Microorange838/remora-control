#include <para_service.h>

/*************************************************************
 * @file para_service.c
 * @author MicroOrange
 * @brief 参数管理、接收与储存
 * @date 2022.2.23
 *************************************************************/
#include "fish_controller.h"
#include "motion_planner.h"
#include "speed_convert_layer_v2.h"



int _index_between(int index, int lower, int upper){
    if (index>=lower && index<=upper)
        return 1;
    else
        return 0;
}

void Para_SetPara(int para_index, double value){
    if (para_index == PARA_MOTION_MODE)
        Motion_SetModeSimple((int)value);
    else if (_index_between(para_index, PARA_CPG_LEFT_PECTROL_AMP_DYNAMIC, PARA_CPG_TAIL_PHASE)){
        CPG_SetPara(para_index, value);
    } else if (_index_between(para_index, PARA_MOTION_THREEDIMROTATE_PID_R_KP, PARA_MOTION_THREEDIMROTATE_Y)){
        Motion_ParaThreeDimRotate(para_index, value);
    } else if (_index_between(para_index, PARA_CONVERT_kf, PARA_CONVERT_k_tail1_2_offset)){
        ConvertLayer_Setpara(para_index, value);
    } else if (_index_between(para_index, PARA_MOTION_MODE_TEST2_Vf, PARA_REMOTE_CHANNEL_8)){
        Motion_ParaTest2(para_index, value);
    }
}
