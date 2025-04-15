#include "headfile.h"

#define Angle_Set 44.5       //直立车机械零点；//17.5，21.5
#define Angle_Zero 0.33

#define Gyro_Set 0  //陀螺仪零点漂移；
//#define Target_Speed 0  //直立串级目标速度（单位为角度）


extern float a;
extern float b;
extern float c;
extern float d;

extern float a2;
extern float b2;
extern float c2;
extern float d2;

extern float a3;
extern float b3;
extern float c3;
extern float d3;


extern float e;
extern int v1;
extern int v2;

//直立内环（角速度环）参数（有备无患，作为参考）
extern float Gyro_P;
extern float Gyro_D;
extern float Gyro_I;
extern float Gyro_error;
extern float Gyro_last_error;

//直立外环（角度环）参数（有备无患，作为参考）
extern float Angle_P;
extern float Angle_D;
extern float Angle_I;
extern float Angle_error;
extern float Angle_last_error;

extern float Angle_Out;

//单环直立环PID参数
extern float Balance_P;
extern float Balance_D;
extern float Balance_I;
extern float Balance_error;
extern float Balance_last_error;

//单环速度环PID参数
extern float Speed_P;
extern float Speed_D;
extern float Speed_I;
extern float Speed_error;
extern float Speed_last_error;
extern float Target_Speed;

//单环方向环PID参数
extern float Dir_P;
extern float Dir_D;
extern float Dir_I;
extern float Dir_error;
extern float Dir_last_error;

//双环方向环内环PID参数
extern float Dir_Yaw_P;
extern float Dir_Yaw_D;
extern float Dir_Yaw_I;
extern float Dir_Yaw_error;
extern float Dir_Yaw_last_error;
extern float Dir_Yaw_Out;
//各环输出参数
extern float Balance_Out;
extern float Angle_Out;
extern float Speed_Out;
extern float Dir_Out;
extern float Left_PWM_Out;
extern float Right_PWM_Out;

extern float Anteil;
//轨迹参数
extern uint8 Threshold_1;
extern uint8 Threshold_2;
extern float T1_K1;
extern float T2_K2;
extern int compensate;
extern int MODE_LAST;
extern int Dir_upper;
extern int Dir_count;

extern int RL_P;
extern int RL_D;
extern int RL_I;
extern int Circle_R;
extern int MODE1_count_th;
extern int Dir_Yaw_Out1;

extern float K_Line_th;
extern float center_final_col_line1;

//int PID_Position(float P,float D,float I,float error,float last_error);
//int PID_Increase(float P,float D,float I,float error,float last_error);

float Balance_Control(float Cur_Angle);//此处为单环直立环控制函数

float Gyro_Control(float Angle_Out);//此处为内外环双环直立控制函数，与单环函数相互独立
float Angle_Control(float Cur_Angle);

float Speed_Control(float Left_Encoder, float Right_Encoder);
float Dir_Control();
float Dir_Yaw_Control();

void PWM_Out();
void Speed_level(int V_Expected);

void Total_Control();
void PWM_Goto_Light();
void PWM_Stop();
void PWM_return();
void PWM_Search_Light();
void PWM_Circle();
void PWM_Getdirection();
void PWM_Slow();
void PWM_EXP();