#include "p8864_sensor.h"
#include "easyMatrix.h"
#include "string.h"
#include "math.h"
#include "declareFunctions.h"
#include "p8864_plane_solver.h"
#include "stm32h7xx.h"
#include "math.h"
#include "fish_controller.h"
#include "log.h"
#include "FreeRTOS.h"
#include "karman_filter_1dim.h"
struct easyMatrix pre_x_mat,y_mat,res_mat;
float pre_x_data[] = {-0.010416666977107525,-0.010416666977107525,-0.010416666977107525,-0.010416666977107525,-0.010416666977107525,-0.010416666977107525,-0.010416666977107525,-0.010416666977107525,-0.007440476678311825,-0.007440476678311825,-0.007440476678311825,-0.007440476678311825,-0.007440476678311825,-0.007440476678311825,-0.007440476678311825,-0.007440476678311825,-0.004464285913854837,-0.004464285913854837,-0.004464285913854837,-0.004464285913854837,-0.004464285913854837,-0.004464285913854837,-0.004464285913854837,-0.004464285913854837,-0.00148809514939785,-0.00148809514939785,-0.00148809514939785,-0.00148809514939785,-0.00148809514939785,-0.00148809514939785,-0.00148809514939785,-0.00148809514939785,0.00148809514939785,0.00148809514939785,0.00148809514939785,0.00148809514939785,0.00148809514939785,0.00148809514939785,0.00148809514939785,0.00148809514939785,0.00446428544819355,0.00446428544819355,0.00446428544819355,0.00446428544819355,0.00446428544819355,0.00446428544819355,0.00446428544819355,0.00446428544819355,0.007440476678311825,0.007440476678311825,0.007440476678311825,0.007440476678311825,0.007440476678311825,0.007440476678311825,0.007440476678311825,0.007440476678311825,0.010416666977107525,0.010416666977107525,0.010416666977107525,0.010416666977107525,0.010416666977107525,0.010416666977107525,0.010416666977107525,0.010416666977107525,-0.010416666977107525,-0.007440476678311825,-0.004464285913854837,-0.00148809514939785,0.00148809514939785,0.00446428544819355,0.007440476678311825,0.010416666977107525,-0.010416666977107525,-0.007440476678311825,-0.004464285913854837,-0.00148809514939785,0.00148809514939785,0.00446428544819355,0.007440476678311825,0.010416666977107525,-0.010416666977107525,-0.007440476678311825,-0.004464285913854837,-0.00148809514939785,0.00148809514939785,0.00446428544819355,0.007440476678311825,0.010416666977107525,-0.010416666977107525,-0.007440476678311825,-0.004464285913854837,-0.00148809514939785,0.00148809514939785,0.00446428544819355,0.007440476678311825,0.010416666977107525,-0.010416666977107525,-0.007440476678311825,-0.004464285913854837,-0.00148809514939785,0.00148809514939785,0.00446428544819355,0.007440476678311825,0.010416666977107525,-0.010416666977107525,-0.007440476678311825,-0.004464285913854837,-0.00148809514939785,0.00148809514939785,0.00446428544819355,0.007440476678311825,0.010416666977107525,-0.010416666977107525,-0.007440476678311825,-0.004464285913854837,-0.00148809514939785,0.00148809514939785,0.00446428544819355,0.007440476678311825,0.010416666977107525,-0.010416666977107525,-0.007440476678311825,-0.004464285913854837,-0.00148809514939785,0.00148809514939785,0.00446428544819355,0.007440476678311825,0.010416666977107525,0.0885416641831398,0.078125,0.0677083283662796,0.0572916641831398,0.0468749962747097,0.0364583283662796,0.0260416641831398,0.015624992549419403,0.078125,0.0677083283662796,0.0572916641831398,0.0468749962747097,0.0364583283662796,0.0260416641831398,0.015624992549419403,0.005208328366279602,0.0677083283662796,0.0572916641831398,0.0468749962747097,0.0364583283662796,0.0260416641831398,0.015624992549419403,0.005208328366279602,-0.005208335816860199,0.0572916641831398,0.0468749962747097,0.0364583283662796,0.0260416641831398,0.015624992549419403,0.005208328366279602,-0.005208335816860199,-0.015625007450580597,0.0468749962747097,0.0364583283662796,0.0260416641831398,0.015624992549419403,0.005208328366279602,-0.005208335816860199,-0.015625007450580597,-0.026041671633720398,0.0364583283662796,0.0260416641831398,0.015624992549419403,0.005208328366279602,-0.005208335816860199,-0.015625007450580597,-0.026041671633720398,-0.0364583358168602,0.0260416641831398,0.015625,0.005208328366279602,-0.005208335816860199,-0.015625007450580597,-0.026041671633720398,-0.0364583358168602,-0.0468750074505806,0.015624992549419403,0.005208328366279602,-0.005208343267440796,-0.015625007450580597,-0.026041679084300995,-0.0364583358168602,-0.0468750074505806,-0.057291679084300995};
float y_data[64];
float res_data[3];
double unit_test_points[64*3];
double p8864_intrinsics_inv[9];
KFP_1D p8864_roll_KF,p8864_pitch_KF,p8864_distance_KF;

double math_radians(double deg){
    return (deg/180.0)*PI;
}


void p8864_solver_init(){
    pre_x_mat.rows = 3;
    pre_x_mat.cols = 64;
    pre_x_mat.element = pre_x_data;
    y_mat.rows = 64;
    y_mat.cols = 1;
    y_mat.element = y_data;
    res_mat.rows = 3;
    res_mat.cols = 1;
    res_mat.element = res_data;

    p8864_intrinsics_inv[8] = 1;
    p8864_intrinsics_inv[0] = (P8864_PiX_WH / 2.0) / tan(math_radians(P8864_FOV / 2.0));
    p8864_intrinsics_inv[2] = P8864_PiX_WH / 2.0;
    p8864_intrinsics_inv[4] = (8 / 2.0) / tan(math_radians(P8864_FOV / 2.0));
    p8864_intrinsics_inv[5] = P8864_PiX_WH / 2.0;
    inv(p8864_intrinsics_inv, 3);
    kalmanFilter_1D_init(&p8864_roll_KF, 90.0f, 20.0f, 0.5f);
    kalmanFilter_1D_init(&p8864_pitch_KF, 90.0f, 20.0f, 0.5f);
    kalmanFilter_1D_init(&p8864_distance_KF, 1000.0f, 100.0f, 2.0f);
}

void p8864_plane_slove(p8864_data_t *p8864_data, float *pitch, float *roll, float *distance){
    double *points_i = pvPortMalloc(sizeof(double)*64*3);
    uint32_t points_cnt = 0;
    volatile uint32_t second_points_cnt = 0;
    uint32_t point_distance_raw;
    uint32_t point_amp;
    double first_plane_a,first_plane_b,first_plane_c,first_plane_d;
    double second_plane_a,second_plane_b,second_plane_c,second_plane_d;
    double first_plane_ang_x,first_plane_ang_y,first_plane_ang_z;
    float dis;
    for (int i = 0; i < P8864_PiX_WH; ++i) {
        for (int j = 0; j < P8864_PiX_WH; ++j) {
            point_distance_raw = p8864_data->distance[i*8+j];
            point_amp = p8864_data->amp[i*8+j];
//            LOGLN_INFO("Lidar","Point%d:%lu,amp:%lu", i*8+j, point_distance_raw, point_amp);
            if (point_distance_raw==P8864_DISTANCE_ERROR_FULL |
                point_distance_raw==P8864_DISTANCE_ERROR_HIGH_AMP |
                point_distance_raw==P8864_DISTANCE_ERROR_LOW_AMP |
                point_distance_raw==P8864_DISTANCE_ERROR_LOW_FLOW |
                point_distance_raw==P8864_DISTANCE_ERROR_NULL |
                point_distance_raw==P8864_DISTANCE_ERROR_OVERFLOW)
                continue;
            if (point_amp<P8864_AMP_LOW_THRESHOLD | point_amp>P8864_AMP_HIGH_THRESHOLD)
                continue;

            points_i[points_cnt*3] = i;
            points_i[points_cnt*3+1] = j;
            points_i[points_cnt*3+2] = point_distance_raw*0.1;
            points_cnt++;
        }
    }
    LOGLN_INFO("Lidar", "Point:%d", points_cnt);

    if (points_cnt < P8864_POINTS_VALID_THRESHOLD)
        ;
    else {
        double *points_c = pvPortMalloc(sizeof(double)*points_cnt*3);
        double *second_points_c = pvPortMalloc(sizeof(double)*points_cnt*3);
        uint8_t *first_plane_mask = pvPortMalloc(sizeof(uint8_t)*points_cnt);
        uint8_t *second_plane_mask = pvPortMalloc(sizeof(uint8_t)*points_cnt);

        p8864_point_preprocess(points_i, points_cnt, points_c);
        p8864_ransac_plane_fit(points_c, first_plane_mask, points_cnt, &first_plane_a, &first_plane_b, &first_plane_c,
                               &first_plane_d);
        //    orange_log_mat(points_c, points_cnt, 3, 4, LogLevelInfo, "Lidar");
//        for (int i = 0; i < points_cnt; ++i) {
//            if (first_plane_mask[i])
//                continue;
//            second_points_c[second_points_cnt * 3] = points_c[i * 3];
//            second_points_c[second_points_cnt * 3 + 1] = points_c[i * 3 + 1];
//            second_points_c[second_points_cnt * 3 + 2] = points_c[i * 3 + 2];
//            second_points_cnt++;
//        }
        //Assume plane vector points to the ground
        if (first_plane_c < 0){
            first_plane_a = -first_plane_a;
            first_plane_b = -first_plane_b;
            first_plane_c = -first_plane_c;
        }


        //Angle (with axis)
        first_plane_ang_x = acos(first_plane_a);
        first_plane_ang_y = acos(first_plane_b);
        first_plane_ang_z = acos(first_plane_c);

        dis = (float)(fabs(first_plane_d));
        //p/r/d filter
        if (dis < P8864_DISTANCE_VALID_THRESHOLD){
            *pitch = kalmanFilter_1D(&p8864_pitch_KF, -((float)(90.0-first_plane_ang_x/PI*180) - 25));
            *roll = kalmanFilter_1D(&p8864_roll_KF, (float)fabs(90.0-first_plane_ang_y/PI*180));
            *distance = kalmanFilter_1D(&p8864_distance_KF, dis);
//            *pitch = -((float)(90.0-first_plane_ang_x/PI*180) - 25);
//            //TODO: Tempary lean
//            *roll = (float)(90.0-first_plane_ang_y/PI*180);
//            *distance = dis;
        }


        LOGLN_INFO("Lidar", "Angle: %f,  %f,  Distance:%f:%d ",*pitch, *roll, *distance, points_cnt);
        vPortFree(points_c);
        vPortFree(first_plane_mask);
        vPortFree(second_plane_mask);
        vPortFree(second_points_c);
    }
    vPortFree(points_i);
}

void p8864_point_preprocess(const double *points_i, uint32_t points_cnt,double *points_c){
    double *points_temp_i = pvPortMalloc(sizeof(double)*3);
    double distance;
    for (int i = 0; i <points_cnt; ++i) {
        distance = points_i[3*i+2] * P8864_WATER_REFLEX_INDEX;
        points_temp_i[0] = points_i[3*i] * distance;
        points_temp_i[1] = points_i[3*i+1] * distance;
        points_temp_i[2] = 1 * distance;
        mul(p8864_intrinsics_inv, points_temp_i, false, &points_c[3*i], 3, 3, 1);
    }
    vPortFree(points_temp_i);
}

void p8864_SVD_plane_fit(const double *points, uint32_t points_cnt, double *a, double *b, double *c, double *d){
    double *points_mean = pvPortMalloc(sizeof(double)*3);
    double *points_norm = pvPortMalloc(sizeof(double)*3*points_cnt);
    double *SVD_U = pvPortMalloc(sizeof(double)*points_cnt*points_cnt);
    double *SVD_S = pvPortMalloc(sizeof(double)*points_cnt*3);
    double *SVD_Vt = pvPortMalloc(sizeof(double)*9);
    memcpy(points_norm, points, sizeof(double)*3*points_cnt);

    points_mean[0] = 0;
    points_mean[1] = 0;
    points_mean[2] = 0;
    for (int i = 0; i < points_cnt; ++i) {
        points_mean[0] += points_norm[3*i];
        points_mean[1] += points_norm[3*i+1];
        points_mean[2] += points_norm[3*i+2];
    }
    points_mean[0] /= points_cnt;
    points_mean[1] /= points_cnt;
    points_mean[2] /= points_cnt;
    for (int i = 0; i < points_cnt; ++i) {
        points_norm[3*i] -= points_mean[0];
        points_norm[3*i+1] -= points_mean[1];
        points_norm[3*i+2] -= points_mean[2];
    }
    svd(points_norm, SVD_U, SVD_S, SVD_Vt, (int)points_cnt, 3);
    *a = SVD_Vt[2];
    *b = SVD_Vt[5];
    *c = SVD_Vt[8];
    *d = -(*a)*(points_mean[0]) - (*b)*(points_mean[1]) - (*c)*(points_mean[2]);
    vPortFree(points_mean);
    vPortFree(SVD_U);
    vPortFree(SVD_S);
    vPortFree(SVD_Vt);
    vPortFree(points_norm);

}

void p8864_SVD_plane_fit_masked(const double *points,const uint8_t *mask, uint32_t points_cnt, double *a, double *b, double *c, double *d){
    double *points_mean = pvPortMalloc(sizeof(double)*3);
    double *points_norm = pvPortMalloc(sizeof(double)*3*points_cnt);
    double *SVD_U = pvPortMalloc(sizeof(double)*points_cnt*points_cnt);
    double *SVD_S = pvPortMalloc(sizeof(double)*points_cnt*3);
    double *SVD_Vt = pvPortMalloc(sizeof(double)*9);
    uint32_t masked_point_cnt = 0;
    for (int i = 0; i < points_cnt; ++i) {
        if (mask[i] == 1){
            points_norm[3*masked_point_cnt] = points[3*masked_point_cnt];
            points_norm[3*masked_point_cnt+1] = points[3*masked_point_cnt+1];
            points_norm[3*masked_point_cnt+2] = points[3*masked_point_cnt+2];
            masked_point_cnt++;
        }
    }
//    orange_log_mat(points_norm, points_cnt, 3, 2, LogLevelInfo, "Lidar_masks");
    points_mean[0] = 0;
    points_mean[1] = 0;
    points_mean[2] = 0;
    for (int i = 0; i < masked_point_cnt; ++i) {
        points_mean[0] += points_norm[3*i];
        points_mean[1] += points_norm[3*i+1];
        points_mean[2] += points_norm[3*i+2];
    }
    points_mean[0] /= masked_point_cnt;
    points_mean[1] /= masked_point_cnt;
    points_mean[2] /= masked_point_cnt;
    for (int i = 0; i < masked_point_cnt; ++i) {
        points_norm[3*i] -= points_mean[0];
        points_norm[3*i+1] -= points_mean[1];
        points_norm[3*i+2] -= points_mean[2];
    }
    svd(points_norm, SVD_U, SVD_S, SVD_Vt, (int)masked_point_cnt, 3);
    *a = SVD_Vt[2];
    *b = SVD_Vt[5];
    *c = SVD_Vt[8];
    *d = -(*a)*(points_mean[0]) - (*b)*(points_mean[1]) - (*c)*(points_mean[2]);
    vPortFree(points_mean);
    vPortFree(SVD_U);
    vPortFree(SVD_S);
    vPortFree(SVD_Vt);
    vPortFree(points_norm);

}

void p8864_ransac_plane_fit(const double *points, uint8_t *result_mask, uint32_t points_cnt, double *a, double *b, double *c, double *d){
    double best_a,best_b,best_c,best_d;
    double temp_a,temp_b,temp_c,temp_d;
    uint32_t iteration=0;
    double *sample_points = pvPortMalloc(sizeof(double) * P8864_RANSAC_POINTS_SAMPLE *3);
    uint32_t *sample_index = pvPortMalloc(sizeof(uint32_t) * P8864_RANSAC_POINTS_SAMPLE);
    uint8_t  *sample_mask = pvPortMalloc(sizeof(uint8_t)*points_cnt);
    uint8_t  *best_mask = pvPortMalloc(sizeof(uint8_t)*points_cnt);
    double temp_distance;
    uint32_t temp_points_count;
    uint32_t best_points_cnt=0;
    double temp_x;
    double temp_y;
    double temp_z;
    srand(HAL_GetTick());
    while(iteration < P8864_RANSAC_MAX_ITER){
        memset(sample_mask, 0, sizeof(uint8_t)*points_cnt);
        temp_distance = 0;
        temp_points_count = P8864_RANSAC_POINTS_SAMPLE;
        for (int i = 0; i < P8864_RANSAC_POINTS_SAMPLE; ++i) {
            uint32_t rand_index = (rand()%points_cnt);
            sample_points[i*3] = points[rand_index*3];
            sample_points[i*3+1] = points[rand_index*3+1];
            sample_points[i*3+2] = points[rand_index*3+2];
            sample_index[i] = rand_index;
            sample_mask[rand_index] = 1;
        }
        p8864_SVD_plane_fit(sample_points, P8864_RANSAC_POINTS_SAMPLE, &temp_a, &temp_b, &temp_c, &temp_d);
//        LOGLN_INFO("Lidar_distance", "%f   %f   %f   %f", temp_a, temp_b, temp_c, temp_d);
        for (int i = 0; i < points_cnt; ++i) {
            if (sample_mask[i] == 0){
                temp_x = points[i*3];
                temp_y = points[i*3+1];
                temp_z = points[i*3+2];
                temp_distance = fabs(temp_a*temp_x + temp_b*temp_y + temp_c*temp_z + temp_d) / sqrt(temp_a*temp_a + temp_b*temp_b + temp_c*temp_c);
//                LOG_INFO("Lidar_distance", "%6.2f", temp_distance);
                if(temp_distance < P8864_RANSAC_INLINE_DISTANCE_THRESHOLD){
                    sample_mask[i] = 1;
                    temp_points_count++;
                }
            }
        }
        if (best_points_cnt < temp_points_count){
            best_points_cnt = temp_points_count;
            best_a = temp_a;
            best_b = temp_b;
            best_c = temp_c;
            best_d = temp_d;
            memcpy(best_mask, sample_mask, sizeof(uint8_t)*points_cnt);
        }
        iteration++;
    }
    p8864_SVD_plane_fit_masked(points, best_mask, points_cnt, a, b, c, d);
    memcpy(result_mask, best_mask, sizeof(uint8_t)*points_cnt);
    vPortFree(sample_points);
    vPortFree(sample_index);
    vPortFree(sample_mask);
    vPortFree(best_mask);
}



void p8864_unit_test(){
    double raw_a=1, raw_b=1, raw_c=2, raw_d = 1;
    for (int i = 0; i < 8; ++i) {
        for (int j = 0; j < 8; ++j) {
            unit_test_points[(i*8+j)*3] = i;
            unit_test_points[(i*8+j)*3+1] = j;
            unit_test_points[(i*8+j)*3+2] = -raw_a/raw_c*i -raw_b/raw_c*j -raw_d/raw_c;
        }
    }

//    unit_test_mean[0] = 0;
//    unit_test_mean[1] = 0;
//    unit_test_mean[2] = 0;
//    for (int i = 0; i < 64; ++i) {
//        unit_test_mean[0] += unit_test_points[3*i];
//        unit_test_mean[1] += unit_test_points[3*i+1];
//        unit_test_mean[2] += unit_test_points[3*i+2];
//    }
//    unit_test_mean[0] /= 64;
//    unit_test_mean[1] /= 64;
//    unit_test_mean[2] /= 64;
//    for (int i = 0; i < 64; ++i) {
//        unit_test_points[3*i] -= unit_test_mean[0];
//        unit_test_points[3*i+1] -= unit_test_mean[1];
//        unit_test_points[3*i+2] -= unit_test_mean[2];
//    }
//    svd(unit_test_points, unit_test_U, unit_test_S, unit_test_V, 64, 3);
//    unit_test_Plane_Vector[0] = unit_test_V[2];
//    unit_test_Plane_Vector[1] = unit_test_V[5];
//    unit_test_Plane_Vector[2] = unit_test_V[8];
    ;

}



