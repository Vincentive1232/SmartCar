#include "SEU_Control.h"


int MODE=1;//��ʼ��Ĭ���е�״̬1�����û�ҵ��Զ�����״̬2
uint8 State_Machine_flag = 1;
int car_col_last = 0;
//int car_rotation_flag = 0;
int sum=0;
int Back_Distance=0;

extern float K = 3;
float Threshold_1_4=40;
float ttt=60;
//-------------------------------------------------------------------------------------------------------------------
// @brief		״̬��
// @return		void
// Sample usage:        �����ں���������������ж�״̬
// @note		//״̬1���Ѿ��ҵ��ƣ���ȥ�Ƶ�·�ϣ�
//                      //״̬2��û���ҵ��ƣ�center_col==0��
//                      //״̬3��ת���պ��ҵ��ƣ�center_col��С��ʱ��
//
//
//-------------------------------------------------------------------------------------------------------------------
void State_Machine(){
  MODE_LAST = MODE;
  switch(MODE){
  case 0 :{Back_Distance=0;break;}
    case 1://�����ҵ�
      {
        if(center_final_col == 187){// && car_col_last == 0){//״̬1--->״̬2
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
    case 2://��ȫ��������
      {
        if(center_final_col<(94+80) && center_final_col>(94-80)){
            MODE=1;
            break;
        }
        if((center_final_col != 187 && car_col_last == 187) ){//״̬2--->״̬1
          MODE=2;
          break;
        }
        break;
      }
    case 3://///�ոպÿ�����
      {
        brake_flag = 0;
        Link_on_or_off = 1;
        if(center_final_col<(94+80) && center_final_col>(94-80)){
            MODE=1;
            break;
        }
        if(center_final_col == 187 && car_col_last == 187){//״̬1--->״̬2
            MODE=2;
            break;
        }
        break;
      }
  case 4:////��ƽ�����
    {
      if(center_final_col == 187 && car_col_last == 187){//״̬4--->״̬2
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
          Back_Distance+=((Encoder_L+Encoder_R)/2);//���˼���·��
          if(Back_Distance<-10000){
             MODE=0;
            break;
          }
          break;
      
      }
  }

  car_col_last = center_final_col;
}



