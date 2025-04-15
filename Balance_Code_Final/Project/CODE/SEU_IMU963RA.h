#include "headfile.h"
#define delta_T      0.005f  //5ms计算一次
#define M_PI   3.1415926


extern float param_Kp;   // 加速度计(磁力计)的收敛速率比例增益50 
extern float param_Ki;   //陀螺仪收敛速率的积分增益 0.2
extern float param_Kd;

extern float angle_acc_pitch_imu;
extern float angle_acc_yaw_imu;//z轴的偏航角
extern float angle_acc_roll_imu;

extern float ggx;
extern float ggy;
extern float ggz;

extern float K1;



void Get_gyro_data_imu(void);
void Get_acc_data_imu(void);
void IMUread_IMU963RA();
float imu_angle_calc_pitch(float angle_m, float gyro_m);
float imu_angle_calc_roll(float angle_m, float gyro_m);
void IMU_read(float gx, float gy, float gz, float ax, float ay, float az);
