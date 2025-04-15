#include "SEU_Control.h"

//单环直立环PID参数                               不使用
float Balance_P = 2500;
float Balance_D = 13000;
float Balance_I = 0;
float Balance_error = 0;       //当前误差
float Balance_last_error = 0;  //上一次误差

//直立内环（角速度环）参数（有备无患，作为参考）
float Gyro_P = 150;//300;这组参数适合大多数电量的情况
float Gyro_D = 75;//100;
float Gyro_I = 0;//0
float Gyro_error = 0;
float Gyro_last_error = 0;
float Sum_Gyro_error=0;

//直立外环（角度环）参数（有备无患，作为参考）
float Angle_P = -15;//-40;//-12
float Angle_D = -16;//-60;//-1.5
float Angle_I = 0;
float Angle_error = 0;
float Angle_last_error = 0;

//单环速度环PID参数
float Speed_P = 0.01;//0.01;//-0.002;
float Speed_D = 0;
float Speed_I = 0.05;//0.07;// -0.0004;
float Speed_error = 0;            //当前误差
float Speed_last_error = 0;       //上一次误差
float Speed_last_last_error = 0;  //上上次误差
float Target_Speed = 2500;           //目标速度
float Speed_Filter = 0.1;         //低通滤波器参数

//双环方向环外环PID参数
float Dir_P = -85;
float Dir_D = -120;
float Dir_I = 0;
float Dir_error = 0;
float Dir_last_error = 0;

//双环方向环内环PID参数
float Dir_Yaw_P = 15;
float Dir_Yaw_D = 0;
float Dir_Yaw_I = 0;
float Dir_Yaw_error = 0;
float Dir_Yaw_last_error = 0;
float Dir_Yaw_error_sum=0;
float testtemp = 0;
float Dir_Yaw_Out = 0;


//各环输出参数
float Balance_Out = 0;
float Speed_Out = 0;
float Dir_Out = 0;
float Left_PWM_Out = 0;                //总输出;
float Right_PWM_Out = 0;               //总输出;
float Angle_Out = 0;                   //角度环输出；
float Speed_error_sum = 0;


int Turning_Slow = 0;        //用于在距离太远找不到灯时放慢转圈速度
int Left_dodge = 0;               //用于灯前躲避
int Right_dodge = 0;               //用于灯前躲避
float Anteil = 0;//-0.01;
int dodge = 0;
int MODE_LAST = 0;
int compensate = 0;
int Dir_upper = 38000;
int Dir_count = 0;




//-------------------------------------------------------------------------------------------------------------------
// @brief		直立环单环控制函数(与速度环串环）
// @return		float Balance_OUT(直立环输出）
// Sample usage:        Balance_Control(Pitch);
// @note		此函数为位置式PD控制
//-------------------------------------------------------------------------------------------------------------------
float Balance_Control(float Cur_Angle){//////////////////////////单环不使用
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
// @brief		速度环控制函数(串入角度环中）
// @return		float Speed_OUT(速度环输出）
// Sample usage:        Speed_Control(Left_Encoder, Right_Encoder);
// @note		此函数为增量式PI控制，积分项被限幅
//---------------------------------------------------------------------------------------------------------------
float Speed_Control(float Left_Encoder, float Right_Encoder){
//  float Speed_P_OUT = 0;
//  float Speed_I_OUT = 0;
//  float Speed_D_OUT = 0;
  int upper_limit = 12;
  Speed_error = (Left_Encoder + Right_Encoder) / 2.0 - Target_Speed;
  
  //Speed_error = (1 - Speed_Filter) * Speed_error + Speed_Filter * Speed_last_error; //使得波形更加平滑，滤除高频干扰，防止速度突变
  
//  //增量式PID控制
//  Speed_P_OUT = Speed_P * (Speed_error - Speed_last_error);
//  Speed_I_OUT = Speed_I * Speed_error;
//  Speed_D_OUT = Speed_D * (Speed_error - 2 * Speed_last_error + Speed_last_last_error);
//  //Speed_Out += Speed_P_OUT + Speed_I_OUT + Speed_D_OUT;
//  Speed_Out += Speed_P_OUT + Speed_I_OUT + Speed_D_OUT;
  
  
  //位置式PID，增量式对速度环不稳定
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
  
  //速度限幅
  if(Speed_Out < -upper_limit-1){/////往前加速
    Speed_Out = -upper_limit-1;
  }
  else if(Speed_Out > upper_limit){
    Speed_Out = upper_limit;
  }
  
  return Speed_Out;
}





//-------------------------------------------------------------------------------------------------------------------
// @brief		转向环外环控制函数
// @return		float Dir_OUT(转向环外环输出，为期望角速度）
// Sample usage:        Dir_Control();
// @note		此函数为位置式PD控制
//-------------------------------------------------------------------------------------------------------------------
//椭圆线参数
//5000速度参数
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
  //微分先行   其中Dir_D=Dir_P*Td
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
// @brief		转向环内环控制函数
// @return		float Dir_Yaw_Out(转向环内环输出，为实际转向环输出）
// Sample usage:        Dir_Yaw_Control();
// @note		此函数为位置式PD控制
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
// @brief		直立环双环控制函数1（角速度环/内环）
// @return		float Balance_OUT(直立环输出）
// Sample usage:        Gyro_Control(Pitch);
// @note		此函数为位置式PD控制，输出的结果即为直立环输出结果
//-------------------------------------------------------------------------------------------------------------------
float Gyro_Control(float Angle_Out){
  Gyro_error = Angle_Out - (Gyro_y-Angle_Zero) + Gyro_Set;
  Sum_Gyro_error+=Gyro_error;
  Balance_Out = Gyro_P * Gyro_error + Gyro_D * (Gyro_error - Gyro_last_error)+Gyro_I*(Sum_Gyro_error); 
  Gyro_last_error = Gyro_error;
  return Balance_Out;
}

//-------------------------------------------------------------------------------------------------------------------
// @brief		直立环双环控制函数2（角度环/外环）
// @return		float Angle_OUT(角度环输出）
// Sample usage:        Angle_Control(Pitch);
// @note		此函数为位置式PD控制，输出的结果进入到角速度环1中
//-------------------------------------------------------------------------------------------------------------------
float Angle_Control(float Cur_Angle){
  float Angle = Cur_Angle;
  Angle_error = Angle - Angle_Set - Speed_Out;
  Angle_Out = Angle_P * Angle_error + Angle_D * (Angle_error - Angle_last_error);
  Angle_last_error = Angle_error;
  return Angle_Out;
}






//-------------------------------------------------------------------------------------------------------------------
// @brief		PWM占空比输出函数
// @return		float PWM_Out(三环总输出）
// Sample usage:        PWM_Out();
// @note		此处需限幅
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
// @brief		运动模式
// @return		对于Left_PWM_Out和Right_PWM_Out进行修改
// Sample usage:        见总控制
// @note		不同的控制模式要在此处加函数
//                      在State中加入状态判别
//                      最后汇总到Total_Control（）中
//-------------------------------------------------------------------------------------------------------------------
void PWM_Stop()///可用于依次调整状态
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

float a3 = -10;//-8;//-4，-8;
float b3 = 0;//-20;//10，-20;
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
  //Right_PWM_Out = Balance_Out + Dir_Yaw_Out;//此处加减法是正确的
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
    //Left_PWM_Out = Balance_Out - Dir_Yaw_Out;   //越小跑的越快   往左转（blance——out为正）
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
// @brief		总控制函数
// @return		void
// Sample usage:        Total_Control()
// @note		针对于不同的状态进行的运动控制（主要基于图像）
                        //主要是修改运动时的模式，和相应的PID，speed参数
//-------------------------------------------------------------------------------------------------------------------


void Total_Control(){
  //if(State_Machine_flag==1){State_Machine();}  
  State_Machine();
  
  switch (MODE){
  case -1:{PWM_return();break;}
  case 1:{PWM_Goto_Light();
           RL_error_sum=0;
           break;}//找到灯，去往灯的路上
  case 2:{PWM_Search_Light();PWM_Circle();break;}//找不到灯，原地转圈
  case 3:{PWM_Getdirection();  MODE1_count=0;break;}//找到灯的瞬间，dir有较大的偏差
  //case 3:{PWM_Search_Light();PWM_Circle();break;}
  case 4:{
            PWM_Slow();break;
          }
  case 5:{PWM_EXP() ;break;}////用于调节参数
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

void PWM_return(){//倒退一段距离
    Left_PWM_Out = -15000;
    Right_PWM_Out =-15000; 
}





