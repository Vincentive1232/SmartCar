#include "SEU_Encoder.h"

float Encoder_L = 0;
float Encoder_R = 0;
float Encoder_L_last = 0;
float Encoder_R_last = 0;
float low_pass_K=0.95;

float Actual_Speed=0;
void Encoder_Read(){
  Encoder_L = tim_encoder_get_count(ENCODER_L);							// �ɼ���Ӧ����������
  tim_encoder_rst(ENCODER_L);													// �����Ӧ����
  Encoder_R = tim_encoder_get_count(ENCODER_R);							// �ɼ���Ӧ����������
  tim_encoder_rst(ENCODER_R);	
												// �����Ӧ����
  Encoder_R = Encoder_R;///����ؽӿڷ���
  Encoder_L = -Encoder_L;
  
//  Encoder_L=low_pass_K*Encoder_L+(1-low_pass_K)*Encoder_L_last;
//  Encoder_R=low_pass_K*Encoder_R+(1-low_pass_K)*Encoder_R_last;
//  
//  Encoder_L_last=Encoder_L;
//  Encoder_R_last=Encoder_R;
  
  Actual_Speed=(Encoder_R+Encoder_L)/2;
  
}