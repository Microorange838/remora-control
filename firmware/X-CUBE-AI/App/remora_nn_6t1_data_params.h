/**
  ******************************************************************************
  * @file    remora_nn_6t1_data_params.h
  * @author  AST Embedded Analytics Research Platform
  * @date    Tue Nov 10 13:20:13 2026
  * @brief   AI Tool Automatic Code Generator for Embedded NN computing
  ******************************************************************************
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */

#ifndef REMORA_NN_6T1_DATA_PARAMS_H
#define REMORA_NN_6T1_DATA_PARAMS_H

#include "ai_platform.h"

/*
#define AI_REMORA_NN_6T1_DATA_WEIGHTS_PARAMS \
  (AI_HANDLE_PTR(&ai_remora_nn_6t1_data_weights_params[1]))
*/

#define AI_REMORA_NN_6T1_DATA_CONFIG               (NULL)


#define AI_REMORA_NN_6T1_DATA_ACTIVATIONS_SIZES \
  { 1536, }
#define AI_REMORA_NN_6T1_DATA_ACTIVATIONS_SIZE     (1536)
#define AI_REMORA_NN_6T1_DATA_ACTIVATIONS_COUNT    (1)
#define AI_REMORA_NN_6T1_DATA_ACTIVATION_1_SIZE    (1536)



#define AI_REMORA_NN_6T1_DATA_WEIGHTS_SIZES \
  { 139268, }
#define AI_REMORA_NN_6T1_DATA_WEIGHTS_SIZE         (139268)
#define AI_REMORA_NN_6T1_DATA_WEIGHTS_COUNT        (1)
#define AI_REMORA_NN_6T1_DATA_WEIGHT_1_SIZE        (139268)



#define AI_REMORA_NN_6T1_DATA_ACTIVATIONS_TABLE_GET() \
  (&g_remora_nn_6t1_activations_table[1])

extern ai_handle g_remora_nn_6t1_activations_table[1 + 2];



#define AI_REMORA_NN_6T1_DATA_WEIGHTS_TABLE_GET() \
  (&g_remora_nn_6t1_weights_table[1])

extern ai_handle g_remora_nn_6t1_weights_table[1 + 2];


#endif    /* REMORA_NN_6T1_DATA_PARAMS_H */
