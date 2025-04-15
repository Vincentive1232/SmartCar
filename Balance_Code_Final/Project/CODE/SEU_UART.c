#include "SEU_UART.h"
/*********************************************************
  @seusmartcar
  ���ſƴ���λ���ƽ�Э��
  @author��SEU_SmartCar
  @2019.10.11
  @for ֱ����
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

/*����֪ͨ��λ���µ�һ�����ݿ�ʼ��Ҫ�������ݱ��뷢����*/
void Send_Begin()
{
  my_putchar(0x55);
  my_putchar(0xaa);
  my_putchar(0x11);
}



//������Ҫ���͵Ĳ���  ������16��
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



//����ʵʱ����
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
  my_putchar(Variable_num);//���ͳ��ı�������
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





//��ȡ����λ���������޸Ĵ����ڵĲ���
void Modify_Parameter(u8 *buff)
{
   u8 i=0,addr=0;
   float temp;
   u8 Parameter_num=14; //14���ɸĲ���
  /*�޸Ĳ�������*/
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
   
 /*�Ӳ��������и��²���ֵ  ʾ��*/
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

//����14��������  �����ش�
void Send_Parameter()
{
  u8 i=0,ch=0;
  float temp=0;
  u8 Parameter_num=14;  //14���ɸĲ���
  
  /*������ֵ���µ�����������  ʾ��*/
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
  uart_putchar(UART_5, 0x02);////Э��β
}











//�����жϽ�����λ��14����������
void UART3_RX_IRQHandler(uint8 bytereceive)//�������ڽ������ݵ��жϺ�����bytereceiveΪ���յ�������
{
  uint8 recv=bytereceive;
  static u8 data_cnt=0;
  static u8 predata[10];
  static u8 Recv_Buff[100];
  static u8 Data_Receiving=false;
  if(Data_Receiving)//�������ڽ���������λ���Ĳ�������
  {
    if(data_cnt<56)//4���ֽ�*14������
    {
      Recv_Buff[data_cnt]= recv;//һ������4�ֽ�
      data_cnt++;
    }
    else
    {
      data_cnt=0;    //�ﵽ֡��
      Data_Receiving=false;
      if(recv==2)  //֡β
      {
         Modify_Parameter(Recv_Buff);//��ȡ����λ���������޸Ĵ����ڵĲ���
         SendPara=1;      //�����ش�����λ����ȷ�ϲ����޸����
      }
    }
  }
  if( predata[1]==0x55&&predata[0]==0xAA)//Э��
  {
    switch(recv)         //����Э���жϹ�����
    { 
    case 1:           //��ȡ���� 
      if(SendPara==0) SendPara=1;
      Send_Parameter();
      break;
    case 2:           //�޸Ĳ���
      Data_Receiving=true;
      data_cnt=0;
      break;
    case 3://�������
      break;        
    case 4:{//����1    �л����ε�ҳ�� 
      Para_Control_MODE+=1;
      if(Para_Control_MODE==4)Para_Control_MODE=1;
      break;
    }
    case 5:{//����2     �л�������λ��������
      Para_Send_MODE+=1;
      if(Para_Send_MODE==3)Para_Send_MODE=1;
      break;   
    }  
    case 6:{//���ܿ���3
//      if(State_Machine_flag==1)State_Machine_flag=0;
//      else if(State_Machine_flag==0)State_Machine_flag=1;
      
      if(MODE==0)MODE=-1;
      
      break; 
    }
    case 7:{//���ܿ���4
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
//**********************************************�޸ĵ�Ƭ������*************************
void Para_Control_1111(uint8 flag){
  if(flag==1){///��ȡ��ť
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
  else if(flag==2){//�޸İ�ť
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
  if(flag==1){///��ȡ��ť
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
  else if(flag==2){//�޸İ�ť
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
  if(flag==1){///��ȡ��ť
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
  else if(flag==2){//�޸İ�ť
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
//*********************************************���͸���λ��*******************************
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



void send_picture()//����ͷ��λ����ֻ֧��80*60�ķֱ���    ��Ҫʹ��   ����֮���п�ħ��һ��������
{
   uart_putchar(UART_5, 0x01);
  uart_putchar(UART_5, 0xfe);

  uart_putbuff(UART_5, mt9v03x_image[0],Image_width*Image_height);

     uart_putchar(UART_5, 0xfe);
  uart_putchar(UART_5, 0x01);
}
