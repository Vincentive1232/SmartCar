#ifndef __IMU_H__
#define __IMU_H__
#include "headfile.h"
#define delta_T      0.005f  //5ms计算一次
#define M_PI   3.1415926


extern float Gyro_x,Gyro_y,Gyro_z;
extern float Acc_x,Acc_y,Acc_z;
extern float Last_Angle;
extern float x1,x2,y1;

extern float angle_acc_yaw;



typedef struct {
  float q0;
  float q1;
  float q2;
  float q3;
}quaterInfo_t;

typedef struct {
  float pitch;
  float yaw;
  float roll;
}eulerianAngles_t;

extern float I_ex, I_ey, I_ez;  // 误差积分
extern quaterInfo_t Q_info;  // 全局四元数
extern eulerianAngles_t eulerAngle; //欧拉角

extern float param_Kp;   // 加速度计(磁力计)的收敛速率比例增益50 
extern float param_Ki;   //陀螺仪收敛速率的积分增益 0.2
extern float param_Kd;



extern float ggx;
extern float ggy;
extern float ggz;

extern float K1;



void Get_gyro_data(void);
void Get_acc_data(void);
float invSqrt(float x);
void IMU_AHRSupdate_noMagnetic(float gx, float gy, float gz, float ax, float ay, float az);
void IMU_quaterToEulerianAngles();
void IMUread();
void IMU_zs(float gx, float gy, float gz, float ax, float ay, float az);
float angle_calc_pitch(float angle_m, float gyro_m);
float angle_calc_yaw(float angle_m, float gyro_m);
//滤波器函数，此处提供1阶和2阶两种互补滤波
void Complementary_Filter_1();
void Complementary_Filter_2();
#endif
