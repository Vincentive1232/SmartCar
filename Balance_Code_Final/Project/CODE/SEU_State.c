#include "SEU_Control.h"


int MODE=1;//初始化默认有灯状态1，如果没找到自动进入状态2
uint8 State_Machine_flag = 1;
int car_col_last = 0;
//int car_rotation_flag = 0;
int sum=0;
int Back_Distance=0;

extern float K = 3;
float Threshold_1_4=40;
float ttt=60;
//-------------------------------------------------------------------------------------------------------------------
// @brief		状态机
// @return		void
// Sample usage:        用于融合摄像机和陀螺仪判断状态
// @note		//状态1：已经找到灯，在去灯的路上；
//                      //状态2：没有找到灯，center_col==0；
//                      //状态3：转到刚好找到灯，center_col很小的时候
//
//
//-------------------------------------------------------------------------------------------------------------------
void State_Machine(){
  MODE_LAST = MODE;
  switch(MODE){
  case 0 :{Back_Distance=0;break;}
    case 1://正常找灯
      {
        if(center_final_col == 187){// && car_col_last == 0){//状态1--->状态2
            MODE=2;
            break;
        }
//        if( textlabel == -1 || brake_flag == 1){ 
//          if((Encoder_L+Encoder_R)>3000){//4000
//              MODE=4;
//              break;
//          }
//        }
        break;
      }
    case 2://完全看不见灯
      {
        if(center_final_col<(94+80) && center_final_col>(94-80)){
            MODE=1;
            break;
        }
        if((center_final_col != 187 && car_col_last == 187) ){//状态2--->状态1
          MODE=2;
          break;
        }
        break;
      }
    case 3://///刚刚好看见灯
      {
        brake_flag = 0;
        Link_on_or_off = 1;
        if(center_final_col<(94+80) && center_final_col>(94-80)){
            MODE=1;
            break;
        }
        if(center_final_col == 187 && car_col_last == 187){//状态1--->状态2
            MODE=2;
            break;
        }
        break;
      }
  case 4:////离灯近减速
    {
      if(center_final_col == 187 && car_col_last == 187){//状态4--->状态2
        sum++;
        if(sum>K){
          sum=0;
          MODE=2;
        }
        break;
        
      }
      
//      if(sum_exp < 100 && center_final_col != 0 && car_col_last != 0){
//        brake_flag = 0;
//        Link_on_or_off = 1;
//        MODE = 1;
//        break;
//      }
      break;
    }
    case -1:
      {
          Back_Distance+=((Encoder_L+Encoder_R)/2);//倒退计算路程
          if(Back_Distance<-10000){
             MODE=0;
            break;
          }
          break;
      
      }
  }

  car_col_last = center_final_col;
}



