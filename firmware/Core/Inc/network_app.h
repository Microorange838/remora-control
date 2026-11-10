/*************************************************************
 * @file network.h
 * @author MicroOrange
 * @brief 
 * @date 2023/11/9
 *************************************************************/

#ifndef REMORARL_NETWORK_H
#define REMORARL_NETWORK_H

#include "remora_nn_6t1.h"

/* Calibrated factor converting the ranging reading (mm) to centimetres. */
#define NN_DIST_SCALE       (0.65f / 70.0f)
/* Scale from the controller output (deg) to the neck servo target. */
#define NN_NECK_TO_BUMPER   (7.0f)
/* Sampling period of the finite differences. */
#define NN_SAMPLE_DT        (0.2f)

void Network_Send(float angle, float height);
void Network_Read(float *angle);
void Network_Recv_Callback();
void Network_Init();

void RemoraNN_6t1_Init();
ai_float RemoraNN_6t1_run(ai_float disc_deg, ai_float distance1_mm,
                          ai_float disc_vel, ai_float disc_acc,
                          ai_float height_vel, ai_float height_acc);

#endif //REMORARL_NETWORK_H
