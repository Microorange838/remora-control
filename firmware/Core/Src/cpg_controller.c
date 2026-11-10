/*************************************************************
 * @file cpg_controller.c
 * @author MicroOrange
 * @brief cpg振荡器实现
 * @date 2022.1.15
 *************************************************************/

#include "cpg_controller.h"
#include "string.h"
#include "stdio.h"
#include "math.h"


double LimitOutput(double input,double max,double min)
    {
        if (input > max)
        {
            return max;
        }
        else if (input < min)
        {
            return min;
        }
        return input;
    }

void CPG_NodeInit(cpg_node_t *node, double amplitude, double offset, double frequency, double amp_dynamic_factor, double offset_dynamic_factor){
    node->amplitude = amplitude;
    node->offset = offset;
    node->frequency = frequency;
    node->amp_dynamic_factor = amp_dynamic_factor;
    node->offset_dynamic_factor = offset_dynamic_factor;
    node->link_count = 0;
    node->now_amp = 0;
    node->now_offset = 0;
    node->now_deg = 0;
    node->output = 0;
}


void CPG_UpdateNode(cpg_node_t *node){
    int i;
    double delta_amp = node->amp_dynamic_factor * (node->amplitude-node->now_amp);
    double delta_offset = node->offset_dynamic_factor * (node->offset-node->now_offset);
    double delta_deg = 2*PI*node->frequency;
    for (i = 0; i < node->link_count; ++i) {
        cpg_node_t *linked_node = node->links[i];
        delta_deg += node->link_weights[i] * (linked_node->now_deg - node->now_deg - node->link_bias[i]);
    }
    node->now_amp += delta_amp*CPG_TIME_CONSTANT;
    node->now_offset += delta_offset*CPG_TIME_CONSTANT;
    node->now_deg += delta_deg*CPG_TIME_CONSTANT;
    node->output = node->now_offset + node->now_amp*cos(node->now_deg);
}

void CPG_CreateLink(cpg_node_t *target_node, cpg_node_t *source_node, double weight, double bias){
    target_node->links[target_node->link_count] = source_node;
    target_node->link_bias[target_node->link_count] = bias;
    target_node->link_weights[target_node->link_count] = weight;
    target_node->link_count++;
}

int CPG_SetLinkPara(cpg_node_t *target_node, cpg_node_t *source_node, double weight, double bias){
    int i;
    for (i = 0; i < target_node->link_count; ++i) {
        if (target_node->links[i] == source_node){
            target_node->link_weights[i] = weight;
            target_node->link_bias[i] = bias;
            return 0;
        }
    }
    return 1;
}

void Bumper_IsotropyInit(bumper_isotropy_t *bumper, double dynamic, double init_value){
    bumper->dynamic = dynamic;
    bumper->output = init_value;
    bumper->set = init_value;
    bumper->mode = BUMPER_MODE_POS;
    bumper->output_minlimit = -30;
    bumper->output_maxlimit = 30;
    bumper->speed = 0;
}

void Bumper_IsotropyUpdate(bumper_isotropy_t *bumper){
    if (bumper->mode == BUMPER_MODE_POS){
        double delta_pos;
        delta_pos = bumper->dynamic * (bumper->set - bumper->output);
        bumper->output += CPG_TIME_CONSTANT * delta_pos;
    } else{
        bumper->output += CPG_TIME_CONSTANT * bumper->speed;
    }


    bumper->output = LimitOutput(bumper->output,bumper->output_maxlimit ,bumper->output_minlimit);
}

void Bumper_AnisotropyInit(bumper_anisotropy_t *bumper, double dynamic_to, double dynamic_back, double init_value){
    bumper->dynamic_back = dynamic_back;
    bumper->dynamic_to = dynamic_to;
    bumper->output = init_value;
    bumper->set = init_value;
    bumper->now_speed = 0;
}

void Bumper_AnisotropyUpdate(bumper_anisotropy_t *bumper){
    double speed;
    if ((bumper->now_speed>=0 && bumper->output>=0) || (bumper->now_speed<0 && bumper->output<0))
        bumper->now_speed = bumper->dynamic_to * (bumper->set - bumper->output);
    else
        bumper->now_speed = bumper->dynamic_back * (bumper->set - bumper->output);

    bumper->output += bumper->now_speed * CPG_TIME_CONSTANT;
    bumper->output = LimitOutput(bumper->output,bumper->output_maxlimit ,bumper->output_minlimit);
}

