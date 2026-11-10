//
// Created by Gmail on 2023/12/16.
//

#ifndef REMORARL_P8864_PLANE_SOLVER_H
#define REMORARL_P8864_PLANE_SOLVER_H

#define P8864_RANSAC_POINTS_SAMPLE 15
#define P8864_RANSAC_MAX_ITER 100
//unit mm
#define P8864_RANSAC_INLINE_DISTANCE_THRESHOLD 8
#define P8864_FOV 32.0
#define P8864_PiX_WH 8
#define P8864_WATER_REFLEX_INDEX 1.33194

#define P8864_DISTANCE_ERROR_LOW_AMP 16001000
#define P8864_DISTANCE_ERROR_OVERFLOW 16002000
#define P8864_DISTANCE_ERROR_FULL 16003000
#define P8864_DISTANCE_ERROR_NULL 16004000
#define P8864_DISTANCE_ERROR_LOW_FLOW 16005000
#define P8864_DISTANCE_ERROR_HIGH_AMP 16006000

#define P8864_AMP_LOW_THRESHOLD 100
#define P8864_AMP_HIGH_THRESHOLD 1900

#define P8864_DISTANCE_VALID_THRESHOLD 1000
#define P8864_POINTS_VALID_THRESHOLD 5

#define P8864_VALID_THRESHOLD 8

void p8864_solver_init();
void p8864_unit_test();
void p8864_plane_slove(p8864_data_t *p8864_data, float *pitch, float *roll, float *distance);
void p8864_point_preprocess(const double *points_i, uint32_t points_cnt,double *points_c);
void p8864_SVD_plane_fit(const double *points,uint32_t points_cnt, double *a, double *b, double *c, double *d);
void p8864_ransac_plane_fit(const double *points, uint8_t *result_mask, uint32_t points_cnt, double *a, double *b, double *c, double *d);



#endif //REMORARL_P8864_PLANE_SOLVER_H
