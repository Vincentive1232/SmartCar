#include "SEU_UART.h"
/*********************************************************
  @seusmartcar
  名优科创上位机破解协议
  @author：SEU_SmartCar
  @2019.10.11
  @for 直立组
*********************************************************/
#define BYTE0(Temp)       (*(char *)(&Temp))     
#define BYTE1(Temp)       (*((char *)(&Temp) + 1))
#define BYTE2(Temp)       (*((char *)(&Temp) + 2))
#define BYTE3(Temp)       (*((char *)(&Temp) + 3))
float Variable[16]={0};
float Parameter[14]={0};
char SendPara=0,send_data=0;
uint8 Para_Control_MODE=1;
uint8 Para_Send_MODE=1;



void my_putchar(char temp)
{
  uart_putchar(UART_5, temp);
}

/*用来通知上位机新的一组数据开始，要保存数据必须发送它*/
void Send_Begin()
{
  my_putchar(0x55);
  my_putchar(0xaa);
  my_putchar(0x11);
}



//设置需要发送的参数  可以设16个
void Testdata_generate(float k)
{
  static int data;
  static long  i;
  data=data+1;
  if(data>1000) data=0;
  
  switch(Para_Send_MODE){
  case 1:{
      Para_Send_1111();
      break;
        
    }
  case 2:{
      Para_Send_2222();
      break;
    }
  }
}



//发送实时变量
void Send_Variable()
{
  uint8 i=0;
  char ch=0;
  float temp=0;
  uint8 Variable_num=16;
  Testdata_generate(16);
  uart_putchar(UART_5, 0x55);
  uart_putchar(UART_5, 0xaa);
  uart_putchar(UART_5, 0xff);
  uart_putchar(UART_5, 0x01);
  my_putchar(Variable_num);//发送出的变量个数
  for(i=0;i<Variable_num;i++)
  {
    temp=Variable[i];
    ch=BYTE0(temp);
      uart_putchar(UART_5, ch);
    ch=BYTE1(temp);
      uart_putchar(UART_5, ch);
    ch=BYTE2(temp);
      uart_putchar(UART_5, ch);
    ch=BYTE3(temp);
      uart_putchar(UART_5, ch);
  }
 uart_putchar(UART_5, 0x01);
}





//获取到上位机参数后，修改代码内的参数
void Modify_Parameter(u8 *buff)
{
   u8 i=0,addr=0;
   float temp;
   u8 Parameter_num=14; //14个可改参数
  /*修改参数数组*/
   for(i=0;i<Parameter_num;i++)
  {
       BYTE0(temp)=*(u8 *)(buff+addr);
       addr++;
       BYTE1(temp)=*(u8 *)(buff+addr);
       addr++;
       BYTE2(temp)=*(u8 *)(buff+addr);
       addr++;
       BYTE3(temp)=*(u8 *)(buff+addr);
       addr++;
       Parameter[i]=temp;
   }
   
 /*从参数数组中更新参数值  示例*/
   switch(Para_Control_MODE){
    case 1:{
      Para_Control_1111(2);
      break;
    }
      break;
    case 2:{
       Para_Control_2222(2);
       break;
    }
    case 3:{
       Para_Control_3333(2);
       break;
    }
  }
}

//发送14个调参量  参数回传
void Send_Parameter()
{
  u8 i=0,ch=0;
  float temp=0;
  u8 Parameter_num=14;  //14个可改参数
  
  /*将参数值更新到参数数组中  示例*/
  switch(Para_Control_MODE){
    case 1:{
      Para_Control_1111(1);
      break;
    }
      break;
    case 2:{
       Para_Control_2222(1);
       break;
    }
    case 3:{
       Para_Control_3333(1);
       break;
    }
  }
/*                           */
  //Testdata_generate(16);
  uart_putchar(UART_5, 0x55);
  uart_putchar(UART_5, 0xaa);
  uart_putchar(UART_5, 0xff);
  uart_putchar(UART_5, 0x02);
  my_putchar(Parameter_num);
  for(i=0;i<Parameter_num;i++)
  { 
    //PEout(25)=1;
    temp=Parameter[i];
    ch=BYTE0(temp);
    my_putchar(ch);
    ch=BYTE1(temp);
    my_putchar(ch);
    ch=BYTE2(temp);
    my_putchar(ch);
    ch=BYTE3(temp);
    my_putchar(ch);
  }
  uart_putchar(UART_5, 0x02);////协议尾
}











//蓝牙中断接收上位机14个调参数据
void UART3_RX_IRQHandler(uint8 bytereceive)//蓝牙串口接收数据的中断函数，bytereceive为接收到的数据
{
  uint8 recv=bytereceive;
  static u8 data_cnt=0;
  static u8 predata[10];
  static u8 Recv_Buff[100];
  static u8 Data_Receiving=false;
  if(Data_Receiving)//代表正在接收来自上位机的参数数据
  {
    if(data_cnt<56)//4个字节*14个数据
    {
      Recv_Buff[data_cnt]= recv;//一个数据4字节
      data_cnt++;
    }
    else
    {
      data_cnt=0;    //达到帧长
      Data_Receiving=false;
      if(recv==2)  //帧尾
      {
         Modify_Parameter(Recv_Buff);//获取到上位机参数后，修改代码内的参数
         SendPara=1;      //参数回传给上位机，确认参数修改完成
      }
    }
  }
  if( predata[1]==0x55&&predata[0]==0xAA)//协议
  {
    switch(recv)         //根据协议判断功能字
    { 
    case 1:           //读取参数 
      if(SendPara==0) SendPara=1;
      Send_Parameter();
      break;
    case 2:           //修改参数
      Data_Receiving=true;
      data_cnt=0;
      break;
    case 3://保存参数
      break;        
    case 4:{//开关1    切换调参的页数 
      Para_Control_MODE+=1;
      if(Para_Control_MODE==4)Para_Control_MODE=1;
      break;
    }
    case 5:{//开关2     切换发送上位机的数据
      Para_Send_MODE+=1;
      if(Para_Send_MODE==3)Para_Send_MODE=1;
      break;   
    }  
    case 6:{//功能开关3
//      if(State_Machine_flag==1)State_Machine_flag=0;
//      else if(State_Machine_flag==0)State_Machine_flag=1;
      
      if(MODE==0)MODE=-1;
      
      break; 
    }
    case 7:{//功能开关4
      if(MODE==0)MODE=1;
      else MODE=0;
      break;  
    }
    default:           //
      break;
    }
  }
  predata[3]=predata[2];
  predata[2]=predata[1];
  predata[1]=predata[0];
  predata[0]=recv;
}
//**********************************************修改单片机参数*************************
void Para_Control_1111(uint8 flag){
  if(flag==1){///读取按钮
     Parameter[0] = MODE;
     Parameter[1] = e;
     Parameter[2] = a;
     Parameter[3] = b;
     Parameter[4] = c;
     Parameter[5] = d;
     Parameter[6] = a2;
     Parameter[7] = b2;
     Parameter[8] = c2;
     Parameter[9] = d2;
     Parameter[10] = a3;
     Parameter[11] = b3;
     Parameter[12] = c3;
     Parameter[13] = d3;  
  }
  else if(flag==2){//修改按钮
      MODE = Parameter[0];
       e = Parameter[1];
       a = Parameter[2];
       b = Parameter[3];
       c = Parameter[4];
       d = Parameter[5];
       a2 = Parameter[6];
       b2 = Parameter[7];
       c2 = Parameter[8];
       d2 = Parameter[9];
       a3 = Parameter[10];
       b3 = Parameter[11];
       c3 = Parameter[12];
       d3 =Parameter[13];
  }
}
void Para_Control_2222(uint8 flag){
  if(flag==1){///读取按钮
     Parameter[0] = MODE;
     Parameter[1] = e;
     Parameter[2] = Gyro_P;
     Parameter[3] = Gyro_D;
     Parameter[4] = Angle_P;
     Parameter[5] = Angle_D;
     Parameter[6] = Speed_P;
     Parameter[7] = Speed_I;
     Parameter[8] = RL_P;
     Parameter[9] = Threshold_1;
     Parameter[10] = Threshold_2;
     Parameter[11] = T1_K1;
     Parameter[12] = T2_K2;
     Parameter[13] = Circle_R;  
  }
  else if(flag==2){//修改按钮
       MODE = Parameter[0];
       e = Parameter[1];
       Gyro_P = Parameter[2];
       Gyro_D = Parameter[3];
       Angle_P = Parameter[4];
       Angle_D = Parameter[5];
       Speed_P = Parameter[6];
       Speed_I = Parameter[7];
       RL_P = Parameter[8];
       Threshold_1= Parameter[9];
       Threshold_2 = Parameter[10];
       T1_K1 = Parameter[11];
       T2_K2 = Parameter[12];
       Circle_R = Parameter[13];
  }
}
void Para_Control_3333(uint8 flag){
  if(flag==1){///读取按钮
     Parameter[0] = MODE;
     Parameter[1] = Circle_R;
     Parameter[2] = RL_P;
     Parameter[3] = RL_D;
     Parameter[4] = RL_I;
     Parameter[5] = MODE1_count_th;
     Parameter[6] = e;
     Parameter[7] = K_Line_th;
     Parameter[8] = a;
     Parameter[9] = b;
     Parameter[10] = Gyro_I;
     Parameter[11] = v1;
     Parameter[12] = v2;
     Parameter[13] = Threshold;  
  }
  else if(flag==2){//修改按钮
      MODE = Parameter[0];
       Circle_R = Parameter[1];
       RL_P = Parameter[2];
       RL_D = Parameter[3];
       RL_I = Parameter[4];
       MODE1_count_th = Parameter[5];
       e = Parameter[6];
       K_Line_th = Parameter[7];
       a = Parameter[8];
       b = Parameter[9];
       Gyro_I = Parameter[10];
       v1 = Parameter[11];
       v2 = Parameter[12];
       Threshold =Parameter[13];
  }
}
//*********************************************发送给上位机*******************************
void Para_Send_1111(void){
    Variable[0] = MODE;
    Variable[1] = Para_Control_MODE;
    Variable[2] = Para_Send_MODE;
    Variable[3] = eulerAngle.pitch;
    Variable[4] = Dir_Out;
    Variable[5] = Encoder_R-Encoder_L;
    Variable[6] = Encoder_L;
    Variable[7] = Encoder_R;
    Variable[8] = Speed_Out;
    Variable[9] = Balance_Out;
    Variable[10] = Dir_Yaw_Out;
    Variable[11] = Left_PWM_Out;
    Variable[12] = Right_PWM_Out;
    Variable[13] = center_final_col_line;
    Variable[14] = center_final_row;
    Variable[15] = center_final_col;
}
void Para_Send_2222(void){
      Variable[0] = MODE;
      Variable[1] = Para_Control_MODE;
      Variable[2] = Para_Send_MODE;
      Variable[3] = eulerAngle.pitch;
      Variable[4] = sum_exp;
      Variable[5] = brake_flag;
      Variable[6] = Encoder_L;
      Variable[7] = Encoder_R;
      Variable[8] = Speed_Out;
      Variable[9] = Balance_Out;
      Variable[10] = Right_PWM_Out;
      Variable[11] = Left_PWM_Out;
      Variable[12] = Dir_P;
      Variable[13] = Dir_D;
      Variable[14] = Dir_Yaw_P;
      Variable[15] = Dir_Yaw_D;


}



void send_picture()//摄像头上位机，只支持80*60的分辨率    不要使用   等我之后有空魔改一下这个软件
{
   uart_putchar(UART_5, 0x01);
  uart_putchar(UART_5, 0xfe);

  uart_putbuff(UART_5, mt9v03x_image[0],Image_width*Image_height);

     uart_putchar(UART_5, 0xfe);
  uart_putchar(UART_5, 0x01);
}
