#include "SEU_Init.h"

void allInit(void)
{
  uart_init(UART_5, 115200, UART5_TX_C12, UART5_RX_D02);//串口初始化
  //uart_tx_irq(UART_5, ENABLE);
  uart_rx_irq(UART_5, ENABLE);
  gpio_init(SWITCH_1, GPI, GPIO_HIGH, GPI_PULL_UP);								 // 初始化为GPIO浮空输入 默认上拉高电平
  gpio_init(SWITCH_2, GPI, GPIO_HIGH, GPI_PULL_UP);								 // 初始化为GPIO浮空输入 默认上拉高电平
  ips114_init();//1.14寸屏幕初始化；
  ips114_clear(BLACK);
  
  //icm20602_init_spi();//icm20602陀螺仪初始化；
  imu963ra_init();
  mt9v03x_init();//总钻风摄像头初始化；
  tim_encoder_init(ENCODER_L, ENCODER_L_A, ENCODER_L_B);							// 初始化正交编码器采集
  tim_encoder_init(ENCODER_R, ENCODER_R_A, ENCODER_R_B);							// 初始化正交编码器采集
  
  gpio_init(A0, GPO, GPIO_LOW, GPO_PUSH_PULL);									// 设定A0为输出口，负责控制电机转动方向，初始化为低电平，推挽输出
  gpio_init(A1, GPO, GPIO_LOW, GPO_PUSH_PULL);                                                                  // 设定A1为输出口，负责控制电机转动方向，初始化为低电平，推挽输出
  pwm_init(PWM_TIM, PWM_CH3, 10000, 0);										// 初始化TIM5 频率10KHz 初始占空比为 0/PWM_DUTY_MAX*100%	
  pwm_init(PWM_TIM, PWM_CH4, 10000, 0);										// 初始化TIM5 频率10KHz 初始占空比为 0/PWM_DUTY_MAX*100%	
  pwm_enable(TIM_5);//PWM电机输出使能（wichtig！！！！）
  
  tim_interrupt_init_ms(TIM_1, 5, 0x01);											// 初始化 5ms周期    TIM1 PIT中断  优先级最高
  tim_interrupt_init_ms(TIM_2, 10, 0x02);											// 初始化 10ms周期   TIM2 PIT中断  优先级次低
  //tim_interrupt_init_ms(TIM_8, 20, 0x03);	
  	
  // 初始化 20ms周期   TIM8 PIT中断  优先级最低
 // tim_interrupt_init_ms(TIM_5, 40, 0x00);											// 初始化 20ms周期   TIM3 PIT中断  优先级最低
  
  systick_delay_ms(50);
  
}