#define row_num     50          //摄像头采集行数
#define col_num     160	        //摄像头采集列数

#define P_WIDTH       8         //lp1，和lp2指针的之间宽度
#define BW_DELTA      50

//#define whiteRoad     110      //whiteRoad为阈值，小于这个阈值便视为黑色
#define LINE_EDGE 	  2 

#define BLOCK_LEN     20

#define flashInfoNum 2        //利用flash存取值的个数
#define MotorPwmLimit 5000

/******电机通道命名******/
#define PWM_TIM				TIM_5
#define PWM_CH1				TIM_5_CH1_A00
#define PWM_CH2				TIM_5_CH2_A01
#define PWM_CH3				TIM_5_CH3_A02
#define PWM_CH4				TIM_5_CH4_A03

/******拨码开关管脚定义******/
#define SWITCH_1			B8														// 定义主板上拨码开关对应引脚
#define SWITCH_2			B9														// 定义主板上拨码开关对应引脚