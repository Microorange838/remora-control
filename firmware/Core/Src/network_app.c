/*************************************************************
 * @file network.c
 * @author MicroOrange
 * @brief 
 * @date 2023/11/9
 *************************************************************/
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "remora_nn_6t1.h"
#include "remora_nn_6t1_data_params.h"
#include "log.h"
#include "fish_controller.h"
#include "network_app.h"

uint8_t network_recv_buf[256];
uint32_t network_recv_len;
float network_value;
uint32_t recv_flag=0;
uint32_t timeout_tick;

//local networks
AI_ALIGNED(32)
static ai_u8 remora_nn_activations[AI_REMORA_NN_6T1_DATA_ACTIVATIONS_SIZE];
AI_ALIGNED(32)
static ai_float remora_nn_in_data[AI_REMORA_NN_6T1_IN_1_SIZE];
AI_ALIGNED(32)
static ai_float remora_nn_out_data[AI_REMORA_NN_6T1_OUT_1_SIZE];
ai_buffer *remora_nn_input;
ai_buffer *remora_nn_output;
ai_handle remora_nn_network = AI_HANDLE_NULL;
ai_error remora_nn_err;

/* Standardisation constants for the network inputs and output, taken from
 * the scaler exported with the model. Input order: DISC, HEIGHT,
 * DISC_VELOCITY, DISC_ACCELERATION, HEIGHT_VELOCITY, HEIGHT_ACCELERATION. */
static const float remora_nn_in_mean[AI_REMORA_NN_6T1_IN_1_SIZE] = {
     3.371943f,  3.876132f, -23.060791f,  -4.677599f, -4.756309f, -0.886888f
};
static const float remora_nn_in_scale[AI_REMORA_NN_6T1_IN_1_SIZE] = {
    22.254569f,  3.362933f,  75.021626f, 1488.675683f,  5.661166f, 50.879340f
};
static const float remora_nn_out_mean = 15.936313f;
static const float remora_nn_out_scale = 12.119304f;

void Network_Send(float angle, float height){
    uint8_t send_str[64];
    memset(send_str, 0, 64);
    sprintf(send_str, "%f,%f#", angle, height);
    recv_flag = 0;
    HAL_UART_Transmit(&huart4, send_str, strlen(send_str), 2000);
    timeout_tick = HAL_GetTick()+3000;
    while (recv_flag == 0){
        if (HAL_GetTick() > timeout_tick)
            break;
    }
}

void Network_Read(float *angle){
    *angle = network_value;
}

void Network_Recv_Callback(){
    if (network_recv_buf[network_recv_len-1] == '#'){
        network_recv_buf[network_recv_len-1] = 0;
        sscanf(network_recv_buf, "%f", &network_value);
        network_recv_len = 0;
        recv_flag = 1;
    }
    HAL_UART_Receive_IT(&huart4, &network_recv_buf[network_recv_len++], 1);
}

void Network_Init(){
    HAL_UART_Receive_IT(&huart4, &network_recv_buf[network_recv_len++], 1);
}


void RemoraNN_6t1_Init(){
    const ai_handle acts[] = {remora_nn_activations};
    remora_nn_err = ai_remora_nn_6t1_create_and_init(&remora_nn_network, acts, NULL);
    if (remora_nn_err.type != AI_ERROR_NONE)
    {
        LOGLN_INFO("ai", "RemoraNN6t1 init FAILED");
    }
    else
    {
        LOGLN_INFO("ai", "RemoraNN6t1 init success");
    }
}

ai_float RemoraNN_6t1_run(ai_float disc_deg, ai_float distance1_mm,
                          ai_float disc_vel, ai_float disc_acc,
                          ai_float height_vel, ai_float height_acc){
    /* Convert the ranging reading to centimetres with the calibrated factor
     * before standardising. */
    const float height_cm = distance1_mm * NN_DIST_SCALE;
    float x[AI_REMORA_NN_6T1_IN_1_SIZE];

    x[0] = disc_deg;
    x[1] = height_cm;
    x[2] = disc_vel;
    x[3] = disc_acc;
    x[4] = height_vel;
    x[5] = height_acc;

    for (int i = 0; i < AI_REMORA_NN_6T1_IN_1_SIZE; ++i)
        remora_nn_in_data[i] = (x[i] - remora_nn_in_mean[i]) / remora_nn_in_scale[i];

    remora_nn_input = ai_remora_nn_6t1_inputs_get(remora_nn_network, NULL);
    remora_nn_output = ai_remora_nn_6t1_outputs_get(remora_nn_network, NULL);
    remora_nn_input->data = AI_HANDLE_PTR(remora_nn_in_data);
    remora_nn_output->data = AI_HANDLE_PTR(remora_nn_out_data);
    ai_remora_nn_6t1_run(remora_nn_network, remora_nn_input, remora_nn_output);

    /* De-standardise to the neck angle in degrees. */
    return remora_nn_out_data[0] * remora_nn_out_scale + remora_nn_out_mean;
}