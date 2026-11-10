///*************************************************************
// * @file speed_convert_layer.h
// * @author MicroOrange
// * @brief 速度转换层
// * @date 2022.1.19
// *************************************************************/
//
//#include "tim.h"
//#include "usart.h"
//#include "gpio.h"
//#include "string.h"
//#include "stdio.h"
//#include "pid.h"
//#include "motor.h"
//#include "motor.h"
//#include "fish_controller.h"
//#include "JY901.h"
//#include "cpg_controller.h"
//#include "speed_convert_layer.h"
//
//speed_convert_layer_t convertLayer;
//
//void _Convert_VfThreshold(
//        double vf,
//        double threshold1,
//        double threshold2,
//        double kp,
//        double bp,
//        double kt,
//        double bt,
//        double *Pout,
//        double *Tout){
//    if (vf<threshold1){
//        *Pout=0;
//        *Tout=0;
//    } else if (vf <= threshold2){
//        *Pout = kp*vf+kp;
//        *Tout = 0;
//    } else{
//        *Pout = kp*vf+bp;
//        *Tout = kt*vf+bt;
//    }
//}
//
//double _Convert_VrNeckRollAngle(double vr, double threshold, double k){
//    if (vr > threshold)
//        return (k * (vr-threshold));
//    else if(vr < -threshold)
//        return (k * (vr+threshold));
//    else
//        return 0;
//}
//
//void Convert_UpdateToController(fish_t *remora){
//    double amp_vf_to_pec, amp_vf_to_tail, freq_vf_to_pec, freq_vf_to_tail;
//    //计算Vf对于胸尾FreqAmp分量
//    _Convert_VfThreshold(
//            convertLayer.Vf,
//            convertLayer.amp_vf_threshold1,
//            convertLayer.amp_vf_threshold2,
//            convertLayer.amp_vf_to_pectoral_k,
//            convertLayer.amp_vf_to_pectoral_b,
//            convertLayer.amp_vf_to_tail_b,
//            convertLayer.amp_vf_to_tail_k,
//            &amp_vf_to_pec,
//            &amp_vf_to_tail);
//    _Convert_VfThreshold(
//            convertLayer.Vf,
//            convertLayer.freq_vf_threshold1,
//            convertLayer.freq_vf_threshold2,
//            convertLayer.freq_vf_to_pectoral_k,
//            convertLayer.freq_vf_to_pectoral_b,
//            convertLayer.freq_vf_to_tail_b,
//            convertLayer.freq_vf_to_tail_k,
//            &freq_vf_to_pec,
//            &freq_vf_to_tail);
//
//    double pec_freq = freq_vf_to_pec*convertLayer.freq_vf_weight
//                      +convertLayer.Vd * convertLayer.freq_vd_weight
//                      +convertLayer.Vr * convertLayer.freq_vr_weight;
//    double pec_amp = amp_vf_to_pec*convertLayer.amp_vf_weight
//                     +convertLayer.Vd * convertLayer.amp_vd_weight
//                     +convertLayer.Vr * convertLayer.amp_vr_weight;
//
//    double left_pec_offset = convertLayer.Vd*convertLayer.offset_vd_to_pectoral_b
//            +convertLayer.offset_vd_to_pectoral_b
//            + convertLayer.Vr*convertLayer.offset_vr_to_pectoral_b
//            +convertLayer.offset_vr_to_pectoral_b;
//
//    double right_pec_offset = convertLayer.Vd*convertLayer.offset_vd_to_pectoral_b
//                         +convertLayer.offset_vd_to_pectoral_b
//                         - convertLayer.Vr*convertLayer.offset_vr_to_pectoral_b
//                         -convertLayer.offset_vr_to_pectoral_b;
//
//    double neck_roll_angle = _Convert_VrNeckRollAngle(convertLayer.Vr, convertLayer.pos_vr_to_neck_threshold,
//                                                      convertLayer.pos_vr_to_neck_k);
//    double neck_pitch_speed = convertLayer.speed_vp_to_neck * convertLayer.Vp;
//
//    double tail1_freq = freq_vf_to_tail;
//    double tail2_freq = freq_vf_to_tail * convertLayer.factor_tail1_to_tail2;
//    double tail1_amp = amp_vf_to_tail;
//    double tail2_amp = amp_vf_to_tail * convertLayer.factor_tail1_to_tail2;
//
//    double tail1_offset = convertLayer.Vy*convertLayer.offset_vy_to_tail1;
//    double tail2_offset = convertLayer.Vy*convertLayer.offset_vy_to_tail2;
//
//    remora->cpg_left_pectrol.amplitude = pec_amp;
//    remora->cpg_left_pectrol.frequency = pec_freq;
//    remora->cpg_left_pectrol.offset = left_pec_offset;
//
//    remora->cpg_right_pectrol.amplitude = pec_amp;
//    remora->cpg_right_pectrol.frequency = pec_freq;
//    remora->cpg_right_pectrol.offset = right_pec_offset;
//
//    remora->cpg_tail1.amplitude = tail1_amp;
//    remora->cpg_tail1.frequency = tail1_freq;
//    remora->cpg_tail1.offset = tail1_offset;
//
//    remora->cpg_tail2.amplitude = tail2_amp;
//    remora->cpg_tail2.frequency = tail2_freq;
//    remora->cpg_tail2.offset = tail2_offset;
//
//    remora->bumper_neck_roll.set = neck_roll_angle;
//    remora->bumper_neck_pitch.speed = neck_pitch_speed;
//
//    printf("=======================\r\n"
//           "pec_amp:%d, pec_freq:%d, loffset:%d, roffset:%d"
//           "tail_amp:%d, tail_freq:%d, tailoffset:%d");
//
//}
//
//void Convert_LayerInit(){
//    convertLayer.amp_vf_threshold1 = 0.1;
//    convertLayer.amp_vf_threshold2 = 2;
//    convertLayer.amp_vf_to_pectoral_k = 20;
//    convertLayer.amp_vf_to_pectoral_b = 0;
//    convertLayer.amp_vf_to_tail_k = 7;
//    convertLayer.amp_vf_to_tail_b = -14;
//
//    convertLayer.freq_vf_threshold1 = 0.3;
//    convertLayer.freq_vf_threshold2 = 2;
//    convertLayer.freq_vf_to_pectoral_k = 0.7;
//    convertLayer.freq_vf_to_pectoral_b = 0;
//    convertLayer.freq_vf_to_tail_k = 0.3;
//    convertLayer.freq_vf_to_tail_b = -0.6;
//
//    convertLayer.offset_vd_to_pectoral_k = 5;
//    convertLayer.offset_vd_to_pectoral_b = 0;
//    convertLayer.offset_vr_to_pectoral_k = 3;
//    convertLayer.offset_vr_to_pectoral_b = 0;
//
//    convertLayer.speed_vp_to_neck = 0.5;
//
//    convertLayer.pos_vr_to_neck_threshold = 2;
//    convertLayer.pos_vr_to_neck_k = 20;
//
//    convertLayer.offset_vy_to_tail1 = 0.3;
//    convertLayer.offset_vy_to_tail2 = 0.5;
//
//    convertLayer.amp_vf_weight = 1;
//    convertLayer.amp_vd_weight = 2;
//    convertLayer.amp_vr_weight = 2;
//
//    convertLayer.freq_vf_weight = 1;
//    convertLayer.freq_vd_weight = 0.5;
//    convertLayer.freq_vr_weight = 0.5;
//
//    convertLayer.factor_tail1_to_tail2 = 1.4;
//
//
//}
//
//void Convert_SetSpeed(fish_t *remora, double Vf, double Vr, double Vd, double Vp, double Vy){
//    convertLayer.Vf = Vf;
//    convertLayer.Vr = Vr;
//    convertLayer.Vd = Vd;
//    convertLayer.Vp = Vp;
//    convertLayer.Vy = Vy;
//    Convert_UpdateToController(remora);
//}
//
//
//
//
//
