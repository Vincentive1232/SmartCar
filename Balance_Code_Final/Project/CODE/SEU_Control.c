#include "SEU_Control.h"

//����ֱ����PID����                               ��ʹ��
float Balance_P = 2500;
float Balance_D = 13000;
float Balance_I = 0;
float Balance_error = 0;       //��ǰ���
float Balance_last_error = 0;  //��һ�����

//ֱ���ڻ������ٶȻ����������б��޻�����Ϊ�ο���
float Gyro_P = 150;//300;��������ʺϴ�������������
float Gyro_D = 75;//100;
float Gyro_I = 0;//0
float Gyro_error = 0;
float Gyro_last_error = 0;
float Sum_Gyro_error=0;

//ֱ���⻷���ǶȻ����������б��޻�����Ϊ�ο���
float Angle_P = -15;//-40;//-12
float Angle_D = -16;//-60;//-1.5
float Angle_I = 0;
float Angle_error = 0;
float Angle_last_error = 0;

//�����ٶȻ�PID����
float Speed_P = 0.01;//0.01;//-0.002;
float Speed_D = 0;
float Speed_I = 0.05;//0.07;// -0.0004;
float Speed_error = 0;            //��ǰ���
float Speed_last_error = 0;       //��һ�����
float Speed_last_last_error = 0;  //���ϴ����
float Target_Speed = 2500;           //Ŀ���ٶ�
float Speed_Filter = 0.1;         //��ͨ�˲�������

//˫�������⻷PID����
float Dir_P = -85;
float Dir_D = -120;
float Dir_I = 0;
float Dir_error = 0;
float Dir_last_error = 0;

//˫�������ڻ�PID����
float Dir_Yaw_P = 15;
float Dir_Yaw_D = 0;
float Dir_Yaw_I = 0;
float Dir_Yaw_error = 0;
float Dir_Yaw_last_error = 0;
float Dir_Yaw_error_sum=0;
float testtemp = 0;
float Dir_Yaw_Out = 0;


//�����������
float Balance_Out = 0;
float Speed_Out = 0;
float Dir_Out = 0;
float Left_PWM_Out = 0;                //�����;
float Right_PWM_Out = 0;               //�����;
float Angle_Out = 0;                   //�ǶȻ������
float Speed_error_sum = 0;


int Turning_Slow = 0;        //�����ھ���̫Զ�Ҳ�����ʱ����תȦ�ٶ�
int Left_dodge = 0;               //���ڵ�ǰ���
int Right_dodge = 0;               //���ڵ�ǰ���
float Anteil = 0;//-0.01;
int dodge = 0;
int MODE_LAST = 0;
int compensate = 0;
int Dir_upper = 38000;
int Dir_count = 0;




//-------------------------------------------------------------------------------------------------------------------
// @brief		ֱ�����������ƺ���(���ٶȻ�������
// @return		float Balance_OUT(ֱ���������
// Sample usage:        Balance_Control(Pitch);
// @note		�˺���Ϊλ��ʽPD����
//-------------------------------------------------------------------------------------------------------------------
float Balance_Control(float Cur_Angle){//////////////////////////������ʹ��
  float Angle = Cur_Angle;
  Balance_error = Angle - Angle_Set - Speed_Out;
  Balance_Out = Balance_P * Balance_error + Balance_D * (Balance_error - Balance_last_error);
  Balance_Out=-Balance_Out;
  Balance_last_error = Balance_error;
  return Balance_Out;
}

int v1 = 14;
int v2 = 13;


//-------------------------------------------------------------------------------------------------------------------
// @brief		�ٶȻ����ƺ���(����ǶȻ��У�
// @return		float Speed_OUT(�ٶȻ������
// Sample usage:        Speed_Control(Left_Encoder, Right_Encoder);
// @note		�˺���Ϊ����ʽPI���ƣ�������޷�
//---------------------------------------------------------------------------------------------------------------
float Speed_Control(float Left_Encoder, float Right_Encoder){
//  float Speed_P_OUT = 0;
//  float Speed_I_OUT = 0;
//  float Speed_D_OUT = 0;
  int upper_limit = 12;
  Speed_error = (Left_Encoder + Right_Encoder) / 2.0 - Target_Speed;
  
  //Speed_error = (1 - Speed_Filter) * Speed_error + Speed_Filter * Speed_last_error; //ʹ�ò��θ���ƽ�����˳���Ƶ���ţ���ֹ�ٶ�ͻ��
  
//  //����ʽPID����
//  Speed_P_OUT = Speed_P * (Speed_error - Speed_last_error);
//  Speed_I_OUT = Speed_I * Speed_error;
//  Speed_D_OUT = Speed_D * (Speed_error - 2 * Speed_last_error + Speed_last_last_error);
//  //Speed_Out += Speed_P_OUT + Speed_I_OUT + Speed_D_OUT;
//  Speed_Out += Speed_P_OUT + Speed_I_OUT + Speed_D_OUT;
  
  
  //λ��ʽPID������ʽ���ٶȻ����ȶ�
  if(Speed_error < 1000 && Speed_error > -1000){
    Speed_error_sum += Speed_error;
  }

  
  if(Speed_error < -1000)
  {      upper_limit = v1;

  }
  else
  {
      upper_limit = v2 ;
  }  
  
  Speed_Out = Speed_P*Speed_error + Speed_I*Speed_error_sum*0.0001;
  
  Speed_last_error = Speed_error;
  Speed_last_last_error = Speed_last_error;
  
  //�ٶ��޷�
  if(Speed_Out < -upper_limit-1){/////��ǰ����
    Speed_Out = -upper_limit-1;
  }
  else if(Speed_Out > upper_limit){
    Speed_Out = upper_limit;
  }
  
  return Speed_Out;
}





//-------------------------------------------------------------------------------------------------------------------
// @brief		ת���⻷���ƺ���
// @return		float Dir_OUT(ת���⻷�����Ϊ�������ٶȣ�
// Sample usage:        Dir_Control();
// @note		�˺���Ϊλ��ʽPD����
//-------------------------------------------------------------------------------------------------------------------
//��Բ�߲���
//5000�ٶȲ���
uint8 Threshold_1 = 80;//30;
uint8 Threshold_2 = 40;//20;
float T1_K1 = 0.56;//0.23;
float T2_K2 = 0.5;//0.45;//0.45;



float cc1=0;
float cc2=0;
float cc3=0;
float temp=0;
float y=0;
float center_final_col_line_last=0;
float D_part=0;
float K_Line_th = 130;
float center_final_col_line1 = 0;

float Dir_Control(){
  float Speed = (Encoder_L+Encoder_R)/2.0;
  float K_Line = 0;
  if(MODE==1||MODE==4){ 
//    if(center_final_col>94-40 && center_final_col<94+40){
       center_final_col_line=center_final_col-T1_K1*MIN(Threshold_1,center_final_row)+T2_K2*(MAX(MIN(100,center_final_row),Threshold_2)-Threshold_2);
       center_final_col_line=MIN(188-40,MAX(center_final_col_line,40));
//    }
  }
  else
  {
    center_final_col_line=center_final_col;
  }
  if(center_final_row < 120 - K_Line_th && MODE == 1){
    K_Line = (120 - center_final_row)/((center_final_col_line - 94)*1.0);
    center_final_col_line1 = 94 + K_Line_th/K_Line;
  }
  else{
    center_final_col_line1 = center_final_col_line;
  }
  
  
  
  
  Dir_error = car_center - (center_final_col_line1);
  //΢������   ����Dir_D=Dir_P*Td
//  temp=y*(float)Dir_D+(float)Dir_D;
//  cc3=(float)Dir_D/temp;
//  cc2=((float)Dir_P+(float)Dir_D)/temp;
//  cc1=y*cc3;
  
//  D_part=cc1*D_part+cc2*center_final_col_line+cc3*center_final_col_line_last;
//  Dir_Out=Dir_P*Dir_error+D_part;
  
  Dir_Out = Dir_P * Dir_error + Dir_D * (Dir_error - Dir_last_error);
  Dir_last_error = Dir_error;
  center_final_col_line_last=center_final_col_line;
  return Dir_Out;
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		ת���ڻ����ƺ���
// @return		float Dir_Yaw_Out(ת���ڻ������Ϊʵ��ת�������
// Sample usage:        Dir_Yaw_Control();
// @note		�˺���Ϊλ��ʽPD����
//-------------------------------------------------------------------------------------------------------------------
float Dir_Yaw_Control(){
  Dir_Yaw_error = Dir_Out - Gyro_z;
  Dir_Yaw_error_sum += Dir_Yaw_error;
  Dir_Yaw_Out = Dir_Yaw_P * Dir_Yaw_error + Dir_Yaw_D * (Dir_Yaw_error - Dir_Yaw_last_error)+Dir_Yaw_I*Dir_Yaw_error_sum*0.01;
  Dir_Yaw_last_error = Dir_Yaw_error;
//  if(Dir_Yaw_Out > Dir_upper){
//    Dir_Yaw_Out = Dir_upper;
//  }
//  else if(Dir_Yaw_Out < -Dir_upper){
//    Dir_Yaw_Out = -Dir_upper;
//  }
//  else{
//    Dir_Yaw_Out = Dir_Yaw_Out;
//  }
  //Dir_Yaw_Out = low_pass_filter(Dir_Yaw_Out);
  return Dir_Yaw_Out;
}





//-------------------------------------------------------------------------------------------------------------------
// @brief		ֱ����˫�����ƺ���1�����ٶȻ�/�ڻ���
// @return		float Balance_OUT(ֱ���������
// Sample usage:        Gyro_Control(Pitch);
// @note		�˺���Ϊλ��ʽPD���ƣ�����Ľ����Ϊֱ����������
//-------------------------------------------------------------------------------------------------------------------
float Gyro_Control(float Angle_Out){
  Gyro_error = Angle_Out - (Gyro_y-Angle_Zero) + Gyro_Set;
  Sum_Gyro_error+=Gyro_error;
  Balance_Out = Gyro_P * Gyro_error + Gyro_D * (Gyro_error - Gyro_last_error)+Gyro_I*(Sum_Gyro_error); 
  Gyro_last_error = Gyro_error;
  return Balance_Out;
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		ֱ����˫�����ƺ���2���ǶȻ�/�⻷��
// @return		float Angle_OUT(�ǶȻ������
// Sample usage:        Angle_Control(Pitch);
// @note		�˺���Ϊλ��ʽPD���ƣ�����Ľ�����뵽���ٶȻ�1��
//-------------------------------------------------------------------------------------------------------------------
float Angle_Control(float Cur_Angle){
  float Angle = Cur_Angle;
  Angle_error = Angle - Angle_Set - Speed_Out;
  Angle_Out = Angle_P * Angle_error + Angle_D * (Angle_error - Angle_last_error);
  Angle_last_error = Angle_error;
  return Angle_Out;
}






//-------------------------------------------------------------------------------------------------------------------
// @brief		PWMռ�ձ��������
// @return		float PWM_Out(�����������
// Sample usage:        PWM_Out();
// @note		�˴����޷�
//-------------------------------------------------------------------------------------------------------------------
void PWM_Out(){
  if(Left_PWM_Out > 38000){
    Left_PWM_Out = 38000;
  }
  else if(Left_PWM_Out < -38000){
    Left_PWM_Out = -38000;
  }
  else{
    Left_PWM_Out = Left_PWM_Out;
  }
  if(Right_PWM_Out > 38000){
    Right_PWM_Out = 38000;
  }
  else if(Right_PWM_Out < -38000){
    Right_PWM_Out = -38000;
  }
  else{
    Right_PWM_Out = Right_PWM_Out;
  }
  
  if(Right_PWM_Out > 0){
    gpio_set (A0, 0);
    pwm_duty_updata(PWM_TIM, PWM_CH3, Right_PWM_Out);	
  }
  else{
    gpio_set (A0, 1);
    pwm_duty_updata(PWM_TIM, PWM_CH3, -Right_PWM_Out);
  }
  if(Left_PWM_Out > 0){
    gpio_set (A1, 0);
    pwm_duty_updata(PWM_TIM, PWM_CH4, Left_PWM_Out);	
  }
  else{
    gpio_set (A1, 1);
    pwm_duty_updata(PWM_TIM, PWM_CH4, -Left_PWM_Out);
  }
}





//-------------------------------------------------------------------------------------------------------------------
// @brief		�˶�ģʽ
// @return		����Left_PWM_Out��Right_PWM_Out�����޸�
// Sample usage:        ���ܿ���
// @note		��ͬ�Ŀ���ģʽҪ�ڴ˴��Ӻ���
//                      ��State�м���״̬�б�
//                      �����ܵ�Total_Control������
//-------------------------------------------------------------------------------------------------------------------
void PWM_Stop()///���������ε���״̬
{
      Target_Speed = 0;
      Left_PWM_Out = 0;
      Right_PWM_Out = 0;
}



float a = -39;
float b = -175;
float c = 6;//8
float d = 0;//-20

float a2 = -16.5;//-5
float b2 = 0;//-30
float c2 = 3;
float d2 = 0;

float a3 = -10;//-8;//-4��-8;
float b3 = 0;//-20;//10��-20;
float c3 = 3;//10;//5;
float d3 = 0;//-2;//5;

float e = 7000;

int MODE1_count=0;
int MODE1_count_th=0;
void PWM_Goto_Light()
{
  if(Actual_Speed > 3500){
    Dir_P = a;
    Dir_D = b;    
  }
  else if(Actual_Speed<1500){
      Dir_P = a/6.0;
    Dir_D = b/6.0;

}
  else
  {
    Dir_P = a/2.0;
    Dir_D = b/2.0;
  }
  Dir_error = car_center - (center_final_col_line);
//  Dir_P = a;// - c*abs(sqrt(Dir_error));//d * Actual_Speed * 0.01;
//  Dir_D = b;// - d*abs(sqrt(Dir_error));//c * Actual_Speed * 0.01;
  Dir_Yaw_P=c;
  Dir_Yaw_D=d;
  Target_Speed=e;
  //Left_PWM_Out = Balance_Out - Dir_Yaw_Out;
  //Right_PWM_Out = Balance_Out + Dir_Yaw_Out;//�˴��Ӽ�������ȷ��
}

void PWM_Search_Light()
{
    Dir_P=a2;
    Dir_D=b2;
    Dir_Yaw_P=c2;
    Dir_Yaw_D=d2;
    if(0){
      Dir_P = -5;
        Dir_D = 5;
        Dir_Yaw_P = 5;
        Dir_Yaw_D = -2;
    }
    else{
        Dir_P=a2;
        Dir_D=b2;
        Dir_Yaw_P=c2;
        Dir_Yaw_D=d2;
    }
    Target_Speed=(e);
    //Left_PWM_Out = Balance_Out - Dir_Yaw_Out;   //ԽС�ܵ�Խ��   ����ת��blance����outΪ����
    //Right_PWM_Out= Balance_Out + Dir_Yaw_Out;
}


int RL_P=0;
int RL_D=0;
int RL_I=0;
int RL=0;
int RL_error=0;
int RL_error_sum=0;
int RL_last_error=0;
int Dir_Yaw_Out1=0;
int Circle_R=700;
void PWM_Circle()
{
    RL=Encoder_R-Encoder_L;
    RL_error=RL-Circle_R;    
    
    RL_error_sum+=RL_error;
    
    Dir_Yaw_Out1=RL_P*RL_error+RL_D*(RL_error-RL_last_error)+RL_I*RL_error_sum;
    
    //Left_PWM_Out = Balance_Out - Dir_Yaw_Out - Dir_Yaw_Out1;
    //Right_PWM_Out = Balance_Out + Dir_Yaw_Out + Dir_Yaw_Out1;
    RL_last_error=RL_error;
    
}



void PWM_Getdirection()
{
    Turning_Slow = 0;
    if(sum_exp > 15){
       Dir_P = -16;
       Dir_D = 0;
       Dir_Yaw_P = 4;
       Dir_Yaw_D = 0;
    }
    else{
       Dir_P = a3;
       Dir_D = b3;
       Dir_Yaw_P = c3;
       Dir_Yaw_D = d3;
    }
    Target_Speed=(0.85*e);

    //Left_PWM_Out = Balance_Out - Dir_Yaw_Out;
    //Right_PWM_Out = Balance_Out + Dir_Yaw_Out;
    
}


void PWM_Slow()
{
    Target_Speed =e; 
    Dir_P=a;
    Dir_D=b;
    Dir_Yaw_P=c;
    Dir_Yaw_D=d;


    //Left_PWM_Out = Balance_Out - Dir_Yaw_Out;
    //Right_PWM_Out = Balance_Out + Dir_Yaw_Out;
    
}
  
int V_Step=100; 
void Speed_level(int V_Expected)
{
  if(Target_Speed<V_Expected){Target_Speed+=V_Step;}
  else if(Target_Speed>V_Expected){Target_Speed-=V_Step;}
}


void PWM_EXP()
{
    Target_Speed =0.6*e; 
    Speed_I=d;
    Speed_P=c;
    Dir_P=0;
    Dir_D=0;
    Left_PWM_Out = Balance_Out - Dir_Yaw_Out;
    Right_PWM_Out = Balance_Out + Dir_Yaw_Out;
    
}




//-------------------------------------------------------------------------------------------------------------------
// @brief		�ܿ��ƺ���
// @return		void
// Sample usage:        Total_Control()
// @note		����ڲ�ͬ��״̬���е��˶����ƣ���Ҫ����ͼ��
                        //��Ҫ���޸��˶�ʱ��ģʽ������Ӧ��PID��speed����
//-------------------------------------------------------------------------------------------------------------------


void Total_Control(){
  //if(State_Machine_flag==1){State_Machine();}  
  State_Machine();
  
  switch (MODE){
  case -1:{PWM_return();break;}
  case 1:{PWM_Goto_Light();
           RL_error_sum=0;
           break;}//�ҵ��ƣ�ȥ���Ƶ�·��
  case 2:{PWM_Search_Light();PWM_Circle();break;}//�Ҳ����ƣ�ԭ��תȦ
  case 3:{PWM_Getdirection();  MODE1_count=0;break;}//�ҵ��Ƶ�˲�䣬dir�нϴ��ƫ��
  //case 3:{PWM_Search_Light();PWM_Circle();break;}
  case 4:{
            PWM_Slow();break;
          }
  case 5:{PWM_EXP() ;break;}////���ڵ��ڲ���
  default:{PWM_Stop();break;}
  }
  if(MODE_LAST != MODE && MODE == 1){
    dodge = 1;
//    Dir_upper = 1000;
//    Dir_count++;
  }
  else if(dodge == 1){
    if(Dir_count < 20){
      Dir_upper = 1000;
      Dir_count++;
    }
    else{
      dodge = 2;
    }
  } 
  else{
    dodge = 0;
    Dir_count = 0;
    Dir_upper = 38000;
  }
}

void PWM_return(){//����һ�ξ���
    Left_PWM_Out = -15000;
    Right_PWM_Out =-15000; 
}





