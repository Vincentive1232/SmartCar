#include "SEU_Init.h"

void allInit(void)
{
  uart_init(UART_5, 115200, UART5_TX_C12, UART5_RX_D02);//���ڳ�ʼ��
  //uart_tx_irq(UART_5, ENABLE);
  uart_rx_irq(UART_5, ENABLE);
  gpio_init(SWITCH_1, GPI, GPIO_HIGH, GPI_PULL_UP);								 // ��ʼ��ΪGPIO�������� Ĭ�������ߵ�ƽ
  gpio_init(SWITCH_2, GPI, GPIO_HIGH, GPI_PULL_UP);								 // ��ʼ��ΪGPIO�������� Ĭ�������ߵ�ƽ
  ips114_init();//1.14����Ļ��ʼ����
  ips114_clear(BLACK);
  
  //icm20602_init_spi();//icm20602�����ǳ�ʼ����
  imu963ra_init();
  mt9v03x_init();//���������ͷ��ʼ����
  tim_encoder_init(ENCODER_L, ENCODER_L_A, ENCODER_L_B);							// ��ʼ�������������ɼ�
  tim_encoder_init(ENCODER_R, ENCODER_R_A, ENCODER_R_B);							// ��ʼ�������������ɼ�
  
  gpio_init(A0, GPO, GPIO_LOW, GPO_PUSH_PULL);									// �趨A0Ϊ����ڣ�������Ƶ��ת�����򣬳�ʼ��Ϊ�͵�ƽ���������
  gpio_init(A1, GPO, GPIO_LOW, GPO_PUSH_PULL);                                                                  // �趨A1Ϊ����ڣ�������Ƶ��ת�����򣬳�ʼ��Ϊ�͵�ƽ���������
  pwm_init(PWM_TIM, PWM_CH3, 10000, 0);										// ��ʼ��TIM5 Ƶ��10KHz ��ʼռ�ձ�Ϊ 0/PWM_DUTY_MAX*100%	
  pwm_init(PWM_TIM, PWM_CH4, 10000, 0);										// ��ʼ��TIM5 Ƶ��10KHz ��ʼռ�ձ�Ϊ 0/PWM_DUTY_MAX*100%	
  pwm_enable(TIM_5);//PWM������ʹ�ܣ�wichtig����������
  
  tim_interrupt_init_ms(TIM_1, 5, 0x01);											// ��ʼ�� 5ms����    TIM1 PIT�ж�  ���ȼ����
  tim_interrupt_init_ms(TIM_2, 10, 0x02);											// ��ʼ�� 10ms����   TIM2 PIT�ж�  ���ȼ��ε�
  //tim_interrupt_init_ms(TIM_8, 20, 0x03);	
  	
  // ��ʼ�� 20ms����   TIM8 PIT�ж�  ���ȼ����
 // tim_interrupt_init_ms(TIM_5, 40, 0x00);											// ��ʼ�� 20ms����   TIM3 PIT�ж�  ���ȼ����
  
  systick_delay_ms(50);
  
}