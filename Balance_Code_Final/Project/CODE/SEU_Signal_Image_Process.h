#ifndef __SEU_SIGNAL_IMAGE_PROCESS_H
#define __SEU_SIGNAL_IMAGE_PROCESS_H

#include "headfile.h"
#include "reg_common.h"
#define Image_width 120
#define Image_height 188
extern short Threshold;


extern int center_col;
extern int center_row;
extern int center_final_col;
extern int center_final_row;
extern int center_final_col_line;


extern int sum1;
extern int sum_exp;
extern int textlabel;
extern int brake_flag;
extern int Link_on_or_off;

extern uint8 Bin_Image[Image_width][Image_height];
//extern uint8 Show_Image[Image_width][Image_height];
//extern uint8 Bin_Image_place[3000][4];


short GetOSTU (uint8 tmImage[Image_width][Image_height]);
void Search_Point(void);
void draw_line();

//此处是连通域函数结构体定义
typedef struct { 
  int right;
  int left;
  int up;
  int down;
  int right_row;
  int left_row;
  int up_col;
  int down_col;
  int center_c;
  int center_r;
  int center_grew;
}Link; 

extern Link label_area[200];


typedef struct QNode{ 
  int data; 
  struct QNode *next; 
}QNode; 

typedef struct Queue{ 
  struct QNode* first; 
  struct QNode* last; 
}Queue; 




#endif