#ifndef __DEBUG_H__
#define __DEBUG_H__
#include "headfile.h"
//取一个数据的各个位



extern float Variable[16];
extern float Parameter[14];

extern char send_data,SendPara;
extern uint8 Send_MODE;


void Testdata_generate(float);
void Send_Begin();
void Send_Parameter();
void Send_Variable();
void UART3_RX_IRQHandler(uint8 bytereceive);
void send_picture();

void Para_Control_1111(uint8 flag);
void Para_Control_2222(uint8 flag);
void Para_Control_3333(uint8 flag);


void Para_Send_1111(void);
void Para_Send_2222(void);
void Para_Send_3333(void);
#endif 