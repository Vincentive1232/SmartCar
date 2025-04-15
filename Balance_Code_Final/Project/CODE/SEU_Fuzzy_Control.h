#include "headfile.h"

extern float Fuzzy_Rule_Kp_x[7];
extern float Fuzzy_Rule_Kp_y[7];

extern float Fuzzy_Rule_Ki_x[7];
extern float Fuzzy_Rule_Ki_y[7];

extern float Fuzzy_Rule_Kd_x[7];
extern float Fuzzy_Rule_Kd_y[7];

extern float Fuzzy_Kp[7][7];
extern float Fuzzy_Ki[7][7];
extern float Fuzzy_Kd[7][7];


extern float dKp;
extern float dKi;
extern float dKd;

extern float Fuzzy_Out;

void Fuzzy_Calculate_P(float error,float derror);
void Fuzzy_Calculate_I(float error,float derror);
void Fuzzy_Calculate_D(float error,float derror);
void Fuzzy_Control(float error,float derror);