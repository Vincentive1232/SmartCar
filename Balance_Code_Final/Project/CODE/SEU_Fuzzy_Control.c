#include "SEU_Fuzzy_Control.h"

float Fuzzy_Rule_Kp_x[7] = {0,0,0,0,0,0,0};
float Fuzzy_Rule_Kp_y[7] = {0,0,0,0,0,0,0};

float Fuzzy_Rule_Ki_x[7] = {0,0,0,0,0,0,0};
float Fuzzy_Rule_Ki_y[7] = {0,0,0,0,0,0,0};

float Fuzzy_Rule_Kd_x[7] = {0,0,0,0,0,0,0};
float Fuzzy_Rule_Kd_y[7] = {0,0,0,0,0,0,0};

float Fuzzy_Kp[7][7] = {
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
};

float Fuzzy_Ki[7][7] = {
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
};

float Fuzzy_Kd[7][7] = {
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0, 0, 0},
};


float dKp = 0;
float dKi = 0;
float dKd = 0;

float Fuzzy_Out = 0;

//-------------------------------------------------------------------------------------------------------------------
// @brief		模糊控制P计算函数
// @return		void
// Sample usage:        Fuzzy_Calculate_P(error, derror)  error为当前偏差，derror为当前偏差与上一次偏差差值
// @note		无
//-------------------------------------------------------------------------------------------------------------------
void Fuzzy_Calculate_P(float error,float derror){
  float Membership_degree_left = 0,Membership_degree_right = 0;    
  float	Membership_degree_up = 0,Membership_degree_down = 0;          
  int Membership_degree_left_index = 0,Membership_degree_right_index = 0;     
  int Membership_degree_up_index = 0,Membership_degree_down_index = 0;   
	  
	if(error < Fuzzy_Rule_Kp_x[0])
	{
		Membership_degree_left = 1;
		Membership_degree_right = 0;
		Membership_degree_left_index = 0;
	}
	else if(error > Fuzzy_Rule_Kp_x[6])
	{
		Membership_degree_left = 1;
		Membership_degree_right = 0;
		Membership_degree_left_index = 6;
	}
	else
	{
	  for(int i = 0; i < 6; i++)
          {
              if(error >= Fuzzy_Rule_Kp_x[i] && error < Fuzzy_Rule_Kp_x[i+1])
              {
                  Membership_degree_left = (float)(Fuzzy_Rule_Kp_x[i+1] - error)/(Fuzzy_Rule_Kp_x[i+1] - Fuzzy_Rule_Kp_x[i]);
                  Membership_degree_right = 1 - Membership_degree_left;
                  Membership_degree_left_index = i;
                  Membership_degree_right_index = i+1;
                  break;
               }
          }
	}
        
        
        if(derror < Fuzzy_Rule_Kp_y[0])
	{
		Membership_degree_up = 1;
		Membership_degree_down = 0;
		Membership_degree_up_index = 0;
	}
	else if(derror > Fuzzy_Rule_Kp_y[6])
	{
		Membership_degree_up = 1;
		Membership_degree_down = 0;
		Membership_degree_up_index = 6;
	}
	else
	{
	  for(int i = 0; i < 6; i++)
          {
              if(derror >= Fuzzy_Rule_Kp_y[i] && derror < Fuzzy_Rule_Kp_y[i+1])
              {
                  Membership_degree_up = (float)(Fuzzy_Rule_Kp_y[i+1] - derror)/(Fuzzy_Rule_Kp_y[i+1] - Fuzzy_Rule_Kp_y[i]);
                  Membership_degree_down = 1 - Membership_degree_up;
                  Membership_degree_up_index = i;
                  Membership_degree_down_index = i+1;
                  break;
               }
          }
	}
        
        
        dKp = Fuzzy_Kp[Membership_degree_up_index][Membership_degree_left_index]*Membership_degree_left*Membership_degree_up\
              + Fuzzy_Kp[Membership_degree_up_index][Membership_degree_right_index]*Membership_degree_right*Membership_degree_up\
              + Fuzzy_Kp[Membership_degree_down_index][Membership_degree_left_index]*Membership_degree_left*Membership_degree_down\
              + Fuzzy_Kp[Membership_degree_down_index][Membership_degree_right_index]*Membership_degree_right*Membership_degree_down;
}


//-------------------------------------------------------------------------------------------------------------------
// @brief		模糊控制I计算函数
// @return		void
// Sample usage:        Fuzzy_Calculate_I(error, derror)  error为当前偏差，derror为当前偏差与上一次偏差差值
// @note		无
//-------------------------------------------------------------------------------------------------------------------
void Fuzzy_Calculate_I(float error,float derror){
  float Membership_degree_left = 0,Membership_degree_right = 0;    
  float	Membership_degree_up = 0,Membership_degree_down = 0;          
  int Membership_degree_left_index = 0,Membership_degree_right_index = 0;     
  int Membership_degree_up_index = 0,Membership_degree_down_index = 0;   
	  
	if(error < Fuzzy_Rule_Ki_x[0])
	{
		Membership_degree_left = 1;
		Membership_degree_right = 0;
		Membership_degree_left_index = 0;
	}
	else if(error > Fuzzy_Rule_Ki_x[6])
	{
		Membership_degree_left = 1;
		Membership_degree_right = 0;
		Membership_degree_left_index = 6;
	}
	else
	{
	  for(int i = 0; i < 6; i++)
          {
              if(error >= Fuzzy_Rule_Ki_x[i] && error < Fuzzy_Rule_Ki_x[i+1])
              {
                  Membership_degree_left = (float)(Fuzzy_Rule_Ki_x[i+1] - derror)/(Fuzzy_Rule_Ki_x[i+1] - Fuzzy_Rule_Ki_x[i]);
                  Membership_degree_right = 1 - Membership_degree_left;
                  Membership_degree_left_index = i;
                  Membership_degree_right_index = i+1;
                  break;
               }
          }
	}
        
        
        if(derror < Fuzzy_Rule_Ki_y[0])
	{
		Membership_degree_up = 1;
		Membership_degree_down = 0;
		Membership_degree_up_index = 0;
	}
	else if(derror > Fuzzy_Rule_Ki_y[6])
	{
		Membership_degree_up = 1;
		Membership_degree_down = 0;
		Membership_degree_up_index = 6;
	}
	else
	{
	  for(int i = 0; i < 6; i++)
          {
              if(derror >= Fuzzy_Rule_Ki_y[i] && derror < Fuzzy_Rule_Ki_y[i+1])
              {
                  Membership_degree_up = (float)(Fuzzy_Rule_Ki_y[i+1] - derror)/(Fuzzy_Rule_Ki_y[i+1] - Fuzzy_Rule_Ki_y[i]);
                  Membership_degree_down = 1 - Membership_degree_up;
                  Membership_degree_up_index = i;
                  Membership_degree_down_index = i+1;
                  break;
               }
          }
	}
        
        
        dKi = Fuzzy_Ki[Membership_degree_up_index][Membership_degree_left_index]*Membership_degree_left*Membership_degree_up\
              + Fuzzy_Ki[Membership_degree_up_index][Membership_degree_right_index]*Membership_degree_right*Membership_degree_up\
              + Fuzzy_Ki[Membership_degree_down_index][Membership_degree_left_index]*Membership_degree_left*Membership_degree_down\
              + Fuzzy_Ki[Membership_degree_down_index][Membership_degree_right_index]*Membership_degree_right*Membership_degree_down;
}



//-------------------------------------------------------------------------------------------------------------------
// @brief		模糊控制D计算函数
// @return		void
// Sample usage:        Fuzzy_Calculate_D(error, derror)  error为当前偏差，derror为当前偏差与上一次偏差差值
// @note		无
//-------------------------------------------------------------------------------------------------------------------
void Fuzzy_Calculate_D(float error,float derror){
  float Membership_degree_left = 0,Membership_degree_right = 0;    
  float	Membership_degree_up = 0,Membership_degree_down = 0;          
  int Membership_degree_left_index = 0,Membership_degree_right_index = 0;     
  int Membership_degree_up_index = 0,Membership_degree_down_index = 0;   
	  
	if(error < Fuzzy_Rule_Kd_x[0])
	{
		Membership_degree_left = 1;
		Membership_degree_right = 0;
		Membership_degree_left_index = 0;
	}
	else if(error > Fuzzy_Rule_Kd_x[6])
	{
		Membership_degree_left = 1;
		Membership_degree_right = 0;
		Membership_degree_left_index = 6;
	}
	else
	{
	  for(int i = 0; i < 6; i++)
          {
              if(error >= Fuzzy_Rule_Kd_x[i] && error < Fuzzy_Rule_Kd_x[i+1])
              {
                  Membership_degree_left = (float)(Fuzzy_Rule_Kd_x[i+1] - derror)/(Fuzzy_Rule_Kd_x[i+1] - Fuzzy_Rule_Kd_x[i]);
                  Membership_degree_right = 1 - Membership_degree_left;
                  Membership_degree_left_index = i;
                  Membership_degree_right_index = i+1;
                  break;
               }
          }
	}
        
        
        if(derror < Fuzzy_Rule_Kd_y[0])
	{
		Membership_degree_up = 1;
		Membership_degree_down = 0;
		Membership_degree_up_index = 0;
	}
	else if(derror > Fuzzy_Rule_Kd_y[6])
	{
		Membership_degree_up = 1;
		Membership_degree_down = 0;
		Membership_degree_up_index = 6;
	}
	else
	{
	  for(int i = 0; i < 6; i++)
          {
              if(derror >= Fuzzy_Rule_Kd_y[i] && derror < Fuzzy_Rule_Kd_y[i+1])
              {
                  Membership_degree_up = (float)(Fuzzy_Rule_Kd_y[i+1] - derror)/(Fuzzy_Rule_Kd_y[i+1] - Fuzzy_Rule_Kd_y[i]);
                  Membership_degree_down = 1 - Membership_degree_up;
                  Membership_degree_up_index = i;
                  Membership_degree_down_index = i+1;
                  break;
               }
          }
	}
        
        
        dKd = Fuzzy_Kd[Membership_degree_up_index][Membership_degree_left_index]*Membership_degree_left*Membership_degree_up\
              + Fuzzy_Kd[Membership_degree_up_index][Membership_degree_right_index]*Membership_degree_right*Membership_degree_up\
              + Fuzzy_Kd[Membership_degree_down_index][Membership_degree_left_index]*Membership_degree_left*Membership_degree_down\
              + Fuzzy_Kd[Membership_degree_down_index][Membership_degree_right_index]*Membership_degree_right*Membership_degree_down;
}


//-------------------------------------------------------------------------------------------------------------------
// @brief		模糊控制PID输出函数（目前只采用了位置式输出）
// @return		void
// Sample usage:        Fuzzy_Control(error, derror)  error为当前偏差，derror为当前偏差与上一次偏差差值
// @note		无
//-------------------------------------------------------------------------------------------------------------------
void Fuzzy_Control(float error,float derror){
  Fuzzy_Calculate_P(error, derror);
  Fuzzy_Calculate_I(error, derror);
  Fuzzy_Calculate_D(error, derror);
  Fuzzy_Out = dKp * error + dKd * derror;
}