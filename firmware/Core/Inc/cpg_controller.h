/*************************************************************
 * @file cpg_controller.h
 * @author MicroOrange
 * @brief cpg振荡器实现
 * @date 2022.1.15
 *************************************************************/


#ifndef FISHCONTROLLERTEST_CPG_CONTROLLER_H
#define FISHCONTROLLERTEST_CPG_CONTROLLER_H

//Update time gap
#define CPG_TIME_CONSTANT 0.02
#define PI 3.1415926
#define CPG_MAX_LINKS 5



/**
 * amplitude 振荡器目标幅值  degree
 * offset 振荡器目标偏置 degree
 * frequency 振荡器节点频率 Hz
 *
 * amp_dynamic_factor 幅值动态特性因子  1
 * 原理同kp 越高动态性越好（达到目标越快
 * offset_dynamic_factor 偏置动态特性因子 1
 *
 * links 链接的源节点
 * link_weights 源节点到该节点权重 1
 * link_bias 源节点到该节点目标相位差 degree
 * 该节点落后源节点bias度
 *
 * now_xxx 当前振荡器状态
 *
 * output振荡器输出 degree
 **/
typedef struct cpg_node_struct{
    double amplitude;
    double offset;
    double frequency;

    double amp_dynamic_factor;
    double offset_dynamic_factor;

    struct cpg_node_struct *links[CPG_MAX_LINKS];
    double link_weights[CPG_MAX_LINKS];
    double link_bias[CPG_MAX_LINKS];
    int link_count;

    double now_amp;
    double now_offset;
    double now_deg;
    double output;

} cpg_node_t;

enum {
    BUMPER_MODE_SPEED,
    BUMPER_MODE_POS
};


typedef struct{
    double set;
    double dynamic;
    double speed;
    int mode;

    double output_maxlimit;
    double output_minlimit;
    double output;

} bumper_isotropy_t;

typedef struct{
    double set;
    double dynamic_to;
    double dynamic_back;
    double now_speed;

    double output_maxlimit;
    double output_minlimit;
    double output;
} bumper_anisotropy_t;




void CPG_NodeInit(cpg_node_t *node, double amplitude, double offset, double frequency, double amp_dynamic_factor, double offset_dynamic_factor);
void CPG_UpdateNode(cpg_node_t *node);
void CPG_CreateLink(cpg_node_t *target_node, cpg_node_t *source_node, double weight, double bias);
int CPG_SetLinkPara(cpg_node_t *target_node, cpg_node_t *source_node, double weight, double bias);

void Bumper_IsotropyInit(bumper_isotropy_t *bumper, double dynamic, double init_value);
void Bumper_IsotropyUpdate(bumper_isotropy_t *bumper);

void Bumper_AnisotropyInit(bumper_anisotropy_t *bumper, double dynamic_to, double dynamic_back, double init_value);
void Bumper_AnisotropyUpdate(bumper_anisotropy_t *bumper);
#endif //FISHCONTROLLERTEST_CPG_CONTROLLER_H
