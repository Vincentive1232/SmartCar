#ifndef IMG_PROCESSING_H_
#define IMG_PROCESSING_H_

#include "headfile.h"




void searchline_MT9V03X(void);
void dispimage(void);
void dispimage1(void);
void JudgeRoadType(void);
void ProcessImgByType(void);
void crossImgProcess(void);
void rdbPreImgProcess(void);
//unsigned int Steer_Control_M1(void);
//int Motor_Control(void);
//void SteerAndMotorControl(void);
//void setSpeed(int16_t left,int16_t right);
void GrayToBin(void);
    //void FixCurve(void);

typedef enum
{
    rdb_flag1 = 1,
    rdb_flag2 = 2,
    rdb_flag3 = 3,
    rdb_flag4 = 4,
    rdb_flag5 = 5,
    rdb_flag6 = 6,
    rdb_flag7 = 7,
    normal = 8,
    cross = 9,
    portIn = 10,
    rdb_pre = 11,
    rdb_pre2 = 12,
    portIn2 = 13
} roadTypeEnum;

extern roadTypeEnum roadType;
extern uint8 Lx[row_num];
extern uint8 Rx[row_num];
extern uint8 Midx[row_num];
extern bool Llost[row_num];
extern bool Rlost[row_num];
extern int8 dLx[row_num];
extern int8 dRx[row_num];

extern uint8 binImg[row_num][col_num];
extern bool cam_flag;

extern uint8 whiteRoad;
extern uint8 car_center;
extern float KP;
extern float KD;
extern int LastError;
//extern int motorPwm;
//extern unsigned int steerPwm;

extern int startRow;
extern int endRow;
extern int joinRow;

extern int lLostNum;
extern int rLostNum;

extern bool specialDeal;
extern bool isRdb;//是否进过环岛

extern int far_error;
extern int speed_upper;

#endif
