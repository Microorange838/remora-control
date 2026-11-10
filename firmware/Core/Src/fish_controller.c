/*************************************************************
 * @file fish.c
 * @author MicroOrange
 * @brief 鱼体部分控制
 * @date 2022.1.13
 *************************************************************/

#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "pid.h"
#include "motor.h"
#include "motor.h"
#include "fish_controller.h"
#include "IMU.h"
#include "cpg_controller.h"
#include "wireless_model.h"
#include "para_service.h"
#include "speed_convert_layer_v2.h"
#include "motion_planner.h"
#include "stm32h7xx_hal.h"
#include "atk_ms53l1m.h"
#include "sorg_utils.h"
#include "log.h"
#include "network_app.h"
#include "time_measure.h"
#include "sorg_utils.h"
#include "p8864_sensor.h"
#include "p8864_plane_solver.h"
#include "declareFunctions.h"

/**
 * 鱼体声明与外设声明
 */
fish_t remora;


/**
 * 常量声明
 */


/**
 * 各模块初始化函数
 */

/**
 * HTIM2
 * CH1   PA0->TAIL2
 * CH2   PA1->TAIL1
 * CH3   PA2->R_PEC
 * CH4   PA3->L_PEC
 *
 * HTIM3
 * CH1   PA6->NECK_YAW
 * CH2   PA7->DISC
 * CH3   PB0->NECK_ROLL
 * CH4   PB1->NECK_PITCH
 *
 */
void Fish_MotorInit(){
    remora.motor_right_pectrol = Motor_gen(htim2, TIM_CHANNEL_2, 270, 500, 2000, 0);
    remora.motor_right_pectrol.degree_offset = 135;
    remora.motor_left_pectrol = Motor_gen(htim3, TIM_CHANNEL_4, 350, 500, 2000, 0);
    remora.motor_right_pectrol.mirrored = 1;
    remora.motor_right_pectrol.mirror_center = 135;
    remora.motor_tail1 = Motor_gen(htim4, TIM_CHANNEL_2, 120, 500, 2000, 0);
    remora.motor_tail1.degree_offset += 4;
    remora.motor_tail2 = Motor_gen(htim4, TIM_CHANNEL_1, 270, 500, 2000, 0);
    remora.motor_tail2.degree_offset += 2;
    remora.motor_neck_pitch = Motor_gen(htim2, TIM_CHANNEL_3, 270, 500, 2000, 0);
    remora.motor_neck_roll = Motor_gen(htim3, TIM_CHANNEL_2 , 350, 500, 2000, 175);
    remora.motor_neck_yaw = Motor_gen(htim2, TIM_CHANNEL_4, 270, 500, 2000, 0);

    remora.motor_disc = Motor_gen(htim3, TIM_CHANNEL_1, 350, 500, 2000, 175);

}
void Fish_PIDInit(){
    PID_init(&remora.pid_yaw, PID_POSITION, 0.05, 0.00, 0, 50, 100);
    PID_init(&remora.pid_pitch, PID_POSITION, 0.1, 0.00, 0, 20, 100);
    PID_init(&remora.pid_roll_speed, PID_POSITION, 0.05, 0.00, 0, 50, 100);
    PID_init(&remora.pid_roll_mid, PID_POSITION, 0.4, 0.00, 0, 50, 100);

}
void Fish_CPGInit(){
    CPG_NodeInit(&remora.cpg_left_pectrol, 0, 0, 0, 2 ,2);
    CPG_NodeInit(&remora.cpg_right_pectrol, 0, 0, 0, 2 ,2);
    CPG_CreateLink(&remora.cpg_left_pectrol, &remora.cpg_right_pectrol, 0, 0);
    CPG_CreateLink(&remora.cpg_right_pectrol, &remora.cpg_left_pectrol, 0, 0);
    CPG_NodeInit(&remora.cpg_tail1, 0 , 0, 0, 2, 2);
    CPG_NodeInit(&remora.cpg_tail2, 0 , 0, 0, 2, 2);
    CPG_CreateLink(&remora.cpg_tail1, &remora.cpg_tail2, 0.8, PI);
    CPG_CreateLink(&remora.cpg_tail2, &remora.cpg_tail1, 0.8, -PI);

    Bumper_IsotropyInit(&remora.bumper_neck_pitch, 5, 0);
    Bumper_IsotropyInit(&remora.bumper_disc, 5, 0);

    remora.bumper_disc.output_maxlimit = 100;
    remora.bumper_disc.output_minlimit = -60;
    remora.bumper_disc.mode = BUMPER_MODE_SPEED;

    remora.bumper_neck_pitch.output_maxlimit = 135;
    remora.bumper_neck_pitch.output_minlimit = -120;
    remora.bumper_neck_pitch.mode = BUMPER_MODE_SPEED;
}
void Fish_HardwareInit(){

    remora.pid_pitch.set = 0;

    remora.pid_roll_speed.set = 0;

    HAL_TIM_Base_Start_IT(&htim16);  //最后使能定时器，防止Update函数预先执行

}


void Fish_Init(){
    HAL_Delay(2000);
    LOGLN_INFO("Fish", "Init Started");
    time_measure_record_start("Fish_Init");
    Fish_MotorInit();
    Fish_PIDInit();
    Fish_CPGInit();
    IMU_Init();
//    Network_Init();
    atk_ms53l1m_uart_init();
    RemoraNN_6t1_Init();
    Fish_HardwareInit();
    WirelessModel_Init();
    ConvertLayer_Init();
    Motion_Init();
    Motion_SetModeSimple(MOTION_MODE_REMOTE);
    //ParaInit放在最后 启动定时器
    time_measure_record_stop("Fish_Init");
    LOGLN_INFO("Fish", "Init OK");
}

/**
 * 模块更新部分
 */

void Fish_CPGUpdate(){
    CPG_UpdateNode(&remora.cpg_left_pectrol);
    CPG_UpdateNode(&remora.cpg_right_pectrol);
    CPG_UpdateNode(&remora.cpg_tail1);
    CPG_UpdateNode(&remora.cpg_tail2);
    Bumper_IsotropyUpdate(&remora.bumper_neck_pitch);
    Bumper_IsotropyUpdate(&remora.bumper_disc);
}

void Fish_CPGToMotor(){
    Motor_setPositionDegree(&remora.motor_left_pectrol, remora.cpg_left_pectrol.output);
    Motor_setPositionDegree(&remora.motor_right_pectrol, remora.cpg_right_pectrol.output);
    Motor_setPositionDegree(&remora.motor_tail2, remora.cpg_tail1.output);
//    Motor_setPositionDegree(&remora.motor_tail2, remora.cpg_tail2.output);
    Motor_setPositionDegree(&remora.motor_neck_pitch, remora.bumper_neck_pitch.output);
//    Motor_setPositionDegree(&remora.motor_disc, remora.bumper_disc.output);
}


//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)		//定义 timer4 定时器中断
//{
//    static int Tail_delay = 0;
//    static int pec_delay = 0;
//    static int Neck_delay = 200000;
//    uint16_t atk_dist;
//    static int scaler10 = 0;
//    static int scaler1000 = 0;
//    static int deg = -60;
//
//
//    if(htim->Instance == TIM16)
//    {   scaler10++;
//        scaler1000++;
//        if (scaler1000 > 2000){
//
//            scaler1000 = 0;
//        }
//
//        if (scaler10 >= 20){
//            scaler10 = 0;
//            Fish_CPGUpdate();
//            Fish_CPGToMotor();
//            Motion_Update(&remora);
//        }
//
//    }
//    if (htim->Instance == TIM5){
//        tick_us_callback();
//    }
//}

void CPG_SetPara(int para_index, double value){
    switch (para_index) {
        case PARA_CPG_LEFT_PECTROL_AMP_DYNAMIC:
            //临时变更 测试使用
            remora.cpg_left_pectrol.amp_dynamic_factor = value;
            break;
        case PARA_CPG_LEFT_PECTROL_OFFSET_DYNAMIC:
            remora.cpg_left_pectrol.offset_dynamic_factor = value;
            break;
        case PARA_CPG_RIGHT_PECTROL_AMP_DYNAMIC:
            remora.cpg_right_pectrol.amp_dynamic_factor = value;
            break;
        case PARA_CPG_RIGHT_PECTROL_OFFSET_DYNAMIC:
            remora.cpg_right_pectrol.offset_dynamic_factor = value;
            break;
        case PARA_CPG_TAIL1_AMP_DYNAMIC:
            remora.cpg_tail1.amp_dynamic_factor = value;
            break;
        case PARA_CPG_TAIL1_OFFSET_DYNAMIC:
            remora.cpg_tail1.offset_dynamic_factor = value;
            break;
        case PARA_CPG_TAIL2_AMP_DYNAMIC:
            remora.cpg_tail2.amp_dynamic_factor = value;
            break;
        case PARA_CPG_TAIL2_OFFSET_DYNAMIC:
            remora.cpg_tail2.offset_dynamic_factor = value;
            break;
        case PARA_CPG_TAIL_PHASE:
            CPG_SetLinkPara(&remora.cpg_tail2, &remora.cpg_tail1, 0.5, value);
            CPG_SetLinkPara(&remora.cpg_tail1, &remora.cpg_tail2, 0.5, -value);
            break;
        default:
            break;
    }
}


