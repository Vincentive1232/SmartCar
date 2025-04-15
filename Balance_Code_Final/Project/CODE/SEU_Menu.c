#include "SEU_Menu.h"

uint8 switch1_state = 0;															// 按键动作状态
uint8 switch2_state = 0;															// 按键动作状态
uint8 last_switch1_state = 0;                                                                                                                   // 上一次按键动作状态
uint8 last_switch2_state = 0;                                                                                                                   // 上一次按键动作状态


void Menu(){
  switch1_state = gpio_get(SWITCH_1);
  switch2_state = gpio_get(SWITCH_2);
  
  if(switch1_state != last_switch1_state || switch2_state != last_switch2_state){
    ips114_clear(BLACK);
  }
  
  if(switch1_state == 0 && switch2_state == 0){
    ips114_displayimage032_zoom(Bin_Image[0], 188, 120, 188, 120);
  }
  
  else if(switch1_state == 0 && switch2_state == 1){
    //ips114_showstr(70,0,"Speed Page");
    ips114_showstr(0,1,"yaw");
    ips114_showfloat(96,1,eulerAngle.yaw,3,3);
    ips114_showstr(0,2,"center_row");
    ips114_showuint16(96,2,center_final_row);
    ips114_showstr(0,3,"center_col");
    ips114_showuint16(96,3,center_final_col);
    //ips114_showstr(0,4,"Target S");
    //ips114_showfloat(96,4,Target_Speed,5,1);
  }
  
  else if(switch1_state == 1 && switch2_state == 0){
    ips114_showstr(80,0,"Dir Page");
    ips114_showstr(0,1,"Dir P");
    ips114_showfloat(96,1,Dir_P,5,1);
    ips114_showstr(0,2,"Dir I");
    ips114_showfloat(96,2,Dir_I,5,1);
    ips114_showstr(0,3,"Dir D");
    ips114_showfloat(96,3,Dir_D,5,1);
  }
  else{  
    //ips114_displayimage032_zoom(Show_Image[0], 188, 120, 188, 120);
    //ips114_displayimage032_zoom(Bin_Image[0], 188, 120, 188, 120);
    ips114_displayimage032_zoom(mt9v03x_image[0], 188, 120, 188, 120); 
  }
  
  last_switch1_state = switch1_state;
  last_switch2_state = switch2_state;
}