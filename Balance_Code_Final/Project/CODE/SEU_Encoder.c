#include "SEU_Encoder.h"

float Encoder_L = 0;
float Encoder_R = 0;
float Encoder_L_last = 0;
float Encoder_R_last = 0;
float low_pass_K=0.95;

float Actual_Speed=0;
void Encoder_Read(){
  Encoder_L = tim_encoder_get_count(ENCODER_L);							// 采集对应编码器数据
  tim_encoder_rst(ENCODER_L);													// 清除对应计数
  Encoder_R = tim_encoder_get_count(ENCODER_R);							// 采集对应编码器数据
  tim_encoder_rst(ENCODER_R);	
												// 清除对应计数
  Encoder_R = Encoder_R;///靠电池接口方向
  Encoder_L = -Encoder_L;
  
//  Encoder_L=low_pass_K*Encoder_L+(1-low_pass_K)*Encoder_L_last;
//  Encoder_R=low_pass_K*Encoder_R+(1-low_pass_K)*Encoder_R_last;
//  
//  Encoder_L_last=Encoder_L;
//  Encoder_R_last=Encoder_R;
  
  Actual_Speed=(Encoder_R+Encoder_L)/2;
  
}