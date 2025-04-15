#include "headfile.h"

#define ENCODER_L					TIM_3
#define ENCODER_L_A					TIM_3_ENC1_B04
#define ENCODER_L_B					TIM_3_ENC2_B05
#define ENCODER_R					TIM_4
#define ENCODER_R_A					TIM_4_ENC1_B06
#define ENCODER_R_B					TIM_4_ENC2_B07


extern float Encoder_L;
extern float Encoder_R;
extern float Actual_Speed;

void Encoder_Read();