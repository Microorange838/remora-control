///*************************************************************
// * @file speed_convert_layer.h
// * @author MicroOrange
// * @brief 速度转换层
// * @date 2022.1.19
// *************************************************************/
//
//
//#ifndef INC_113REBUILD_SPEED_CONVERT_LAYER_H
//#define INC_113REBUILD_SPEED_CONVERT_LAYER_H
//
///**
// * 速度转换层参数声明
// * 速度 m/s
// * Vfront
// * Vroll
// * Vdive
// * Vpitch
// * Vyaw
// *
// * amp_vf_to_pec_k,b 前游速度-胸鳍振幅一次拟合 degree/(m/s)
// * amp_vf_to_tail_k,b 前游速度-尾鳍振幅一次拟合 degree/(m/s)
// * amp_vf_threshold1 定速推/胸鳍推/胸尾推 切换阈值 m/s
// *
// * freq_vf_to_pec_k,b 前游速度-胸鳍频率一次拟合 1/(m/s)
// * freq_vf_to_tail_k,b 前游速度-尾鳍频率一次拟合 1/(m/s)
// * amp_vf_threshold1 定速推/胸鳍推/胸尾推 切换阈值 m/s
// *
// * offset_vd_to_pectoral_k,b 下潜-胸鳍偏置一次拟合 degree/(m/s)
// * offset_vr_to_pectoral_k,b 滚转-胸鳍偏置一次拟合 degree/(m/s)
// *
// * speed_vp_to_neck 滚转-脖子速度(-threshold)线性拟合 (degree/s)/(m/s)
// *
// * pos_vr_to_neck_threshold 脖子运动的滚转速度阈值 m/s
// * pos_vr_to_neck_k 脖子角度-滚转速度的一次拟合 degree/(m/s)
// *
// * offset_vy_to_tail1,1 偏航速度-第1,2尾部速度偏置拟合 degree/(m/s)
// *
// * amp_vf_weight vf转换胸鳍振幅-胸鳍振幅叠加权重 1
// * amp_vr_weight 同上， 但未经过转换  degree/(m/s)
// * amp_vd_weight
// *
// * 同上 1 1/(m/s)
// */
//typedef struct type_speed_convert{
//    double Vf,Vr,Vd,Vp,Vy;
//
//    double amp_vf_to_pectoral_k, amp_vf_to_pectoral_b;
//    double amp_vf_to_tail_k, amp_vf_to_tail_b;
//    double amp_vf_threshold1, amp_vf_threshold2;
//
//    double freq_vf_to_pectoral_k, freq_vf_to_pectoral_b;
//    double freq_vf_to_tail_k, freq_vf_to_tail_b;
//    double freq_vf_threshold1, freq_vf_threshold2;
//
//    double  offset_vd_to_pectoral_k,offset_vd_to_pectoral_b;
//    double  offset_vr_to_pectoral_k,offset_vr_to_pectoral_b;
//
//    double speed_vp_to_neck;
//
//    double pos_vr_to_neck_threshold;
//    double pos_vr_to_neck_k;
//
//    double offset_vy_to_tail1;
//    double offset_vy_to_tail2;
//
//    double amp_vf_weight;
//    double amp_vr_weight;
//    double amp_vd_weight;
//
//    double freq_vf_weight;
//    double freq_vr_weight;
//    double freq_vd_weight;
//
//    double factor_tail1_to_tail2;
//
//} speed_convert_layer_t;
//
//void Convert_LayerInit();
//void Convert_SetSpeed(fish_t *remora, double Vf, double Vr, double Vd, double Vp, double Vy);
//
//
//#endif //INC_113REBUILD_SPEED_CONVERT_LAYER_H
