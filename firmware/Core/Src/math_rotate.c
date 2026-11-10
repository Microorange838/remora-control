/*************************************************************
 * @file math_rotate
 * @author MicroOrange
 * @brief 刚体旋转变换类
 * @date 2022.2.23
 *************************************************************/


#include <math.h>

#define PI       3.14159265358979323846   // pi
/**
 * 四元数
 */
typedef struct
{
    float w, x, y, z;
}Quat_t;//四元素结构体
typedef struct
{
    float Pitch;
    float Roll;
    float Yaw;
}Euler_t;//欧拉角结构体

//欧拉角转四元素
//euler_angle：输入欧拉角
//q1：输出四元素
int Conversion_Euler_to_Quaternion(Quat_t q1, Euler_t euler_angle)
{
    euler_angle.Yaw = euler_angle.Yaw *  PI / 180;
    euler_angle.Pitch = euler_angle.Pitch * PI / 180;
    euler_angle.Roll = euler_angle.Roll * PI / 180;
    double c1 = acos(euler_angle.Yaw / 2);
    double s1 = asin(euler_angle.Yaw / 2);
    double c2 = acos(euler_angle.Pitch / 2);
    double s2 = asin(euler_angle.Pitch / 2);
    double c3 = acos(euler_angle.Roll / 2);
    double s3 = asin(euler_angle.Roll / 2);
    double c1c2 = c1 * c2;
    double s1s2 = s1 * s2;
    q1.w = (c1c2 * c3 + s1s2 * s3);
    q1.x = (c1c2 * s3 + s1s2 * c3);
    q1.y = (s1 * c2 * c3 + c1 * s2 * s3);
    q1.z = (c1 * s2 * c3 - s1 * c2 * s3);
    return 0;
}
//四元素转欧拉角
//quat:输入四元素
//euler:输出欧拉角
int Conversion_Quaternion_to_Euler(Quat_t *quat, Euler_t *euler)
{
    double q0, q1, q2, q3;
    q0 = quat->w;
    q1 = quat->x;
    q2 = quat->y;
    q3 = quat->z;
    euler->Pitch = (float)(asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3); // pitch
    euler->Roll = (float)(atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3); // roll
    euler->Yaw = (float)(atan2(2 * (q1*q2 + q0*q3), q0*q0 + q1*q1 - q2*q2 - q3*q3) * 57.3);
    return 0;
}
//四元素旋转
//quat1:输入原始四元素
//quat2:输入旋转向量四元素
//返回：输出旋转后的四元素
int quat_pro(Quat_t *quat1, Quat_t *quat2, Quat_t *quat3)
{
    float w1, x1, y1, z1;
    float w2, x2, y2, z2;
    w2 = quat1->w;
    x2 = quat1->x;
    y2 = quat1->y;
    z2 = quat1->z;

    w1 = quat2->w;
    x1 = quat2->x;
    y1 = quat2->y;
    z1 = quat2->z;

    quat3->w = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    quat3->x = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    quat3->y = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    quat3->z = w1*z2 + x1*y2 - y1*x2 + z1*w2;
    return 0;
}
