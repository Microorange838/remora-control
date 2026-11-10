/*************************************************************
 * @file speed_convert_layer_v2.h
 * @author MicroOrange
 * @brief 速度转换层
 * @date 2022.1.31
 *************************************************************/

#ifndef INC_113REBUILD_SPEED_CONVERT_LAYER_V2_H
#define INC_113REBUILD_SPEED_CONVERT_LAYER_V2_H


typedef struct {
    double Vf,Vd,Vy,Vr,Vp;
    double kf,kd,ky,kr,kp;

    double Vf_threshold, kf_tail;
    double Vy_threshold, ky_tail_offset, ky_tail_front;

    double k_pec_amp, k_pec_freq;
    double k_tail1_amp,k_tail2_amp;
    double k_tail1_freq,k_tail2_freq;
    double k_tail1_2_offset;
} speed_convert_layer_v2_t;

void ConvertLayer_Update(fish_t *remora,double Vf,double Vd,double Vy,double Vr,double Vp);
void ConvertLayer_Setpara(int para_index, double value);
void ConvertLayer_Init();

#endif //INC_113REBUILD_SPEED_CONVERT_LAYER_V2_H
