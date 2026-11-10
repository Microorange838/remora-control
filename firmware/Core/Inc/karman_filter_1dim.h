//
// Created by Gmail on 2024/7/1.
//

#ifndef REMORARL43_KARMAN_FILTER_1DIM_H
#define REMORARL43_KARMAN_FILTER_1DIM_H

typedef struct
{

    float LastP;//上次估算协方差 初始化值为0.02
    float Now_P;//当前估算协方差 初始化值为0
    float out;//卡尔曼滤波器输出 初始化值为0
    float Kg;//卡尔曼增益 初始化值为0
    float Q;//过程噪声协方差 初始化值为0.001
    float R;//观测噪声协方差 初始化值为0.543
}KFP_1D;//Kalman Filter parameter

void kalmanFilter_1D_init(KFP_1D *kfp,float P_init, float Q, float R);
float kalmanFilter_1D(KFP_1D *kfp,float input);


#endif //REMORARL43_KARMAN_FILTER_1DIM_H
