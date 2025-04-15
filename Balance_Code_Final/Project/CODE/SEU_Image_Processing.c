#include "SEU_Image_Processing.h"

#define grayImg mt9v03x_image
#define THR 7
#define THR_JMP 3
#define rdb_type (roadType<8||roadType==11||roadType==12)
//判断道路类型时用的一个阈值


//左中右线
uint8 Lx[row_num];
uint8 Rx[row_num];
uint8 Midx[row_num];

//左右线变化率
int8 dLx[row_num];
int8 dRx[row_num];


//对应行是否丢线
bool Llost[row_num];
bool Rlost[row_num];

bool cam_flag=false;//摄像头是否发生了更新，处理后置false

//丢线计数
int lLostNum = 0, rLostNum = 0;

uint8 whiteRoad = 150; //全局阈值
uint8 car_center = 94; //车模中心位置


// steer error计算的初始和末尾行
int startRow = 23;
int endRow = 28;

// motor far_error计算的初始和末尾行
int startRow_m = 21;
int endRow_m =  23;

// 合点 即所能准确看到的最远处， 越小越远。
int joinRow = 0;

//路类型
roadTypeEnum roadType = normal; //之后要改的

bool isRdb = false;//是否进过环岛
int recommendedPwm = 0;

//延时
uint32_t endts;
//是否特殊处理
bool specialDeal=false;

uint8 binImg[row_num][col_num];

int far_error;
int speed_upper=550;

void GrayToBin(void)
{
	int cnt, cnt2;
	for (cnt = 0; cnt < row_num; cnt++)
	{
		for (cnt2 = 0; cnt2 < col_num; cnt2++)
		{
			if (*grayImg[cnt * col_num + cnt2] < whiteRoad)
				binImg[cnt][cnt2] = 0;
			else
				binImg[cnt][cnt2] = 0xff;
		}
	}
}

void searchline_MT9V03X(void)
{

	int CurL = 0, Start = 0;	 /* CurL  当前行   Start 开始扫线的行  第一行从0开始 */
	int Cur_Offset = car_center; /* 初始扫线中心为80，此变量为当前行中心 */
	int i = joinRow;
	int CurPoint = Cur_Offset; /* CurPoint为当前正在扫描的点 */
							   /*
 * 注意：左上角为（0,0）
 * ====================按行扫描左右线===============================
 */
	for (CurL = row_num - 1; CurL >= Start; --CurL)
	{
		//找左线并判断是否丢线
		Llost[CurL] = false;
		CurPoint = Cur_Offset;
		while (CurPoint - 2 >= 0)
		{
			if (binImg[CurL][CurPoint] && !binImg[CurL][CurPoint - 1] && !binImg[CurL][CurPoint - 2])
			{
				Lx[CurL] = CurPoint - 1;
				break;
			}
			else
			{ /* 没找到，向右移动一个像素点 */
				--CurPoint;
			}
		}
		if (CurPoint < 2) // 为0或者丢线时
		{
			Lx[CurL] = 0;
			if (!(binImg[CurL][1] == 0xff && binImg[CurL][0] == 0)) //丢线
				Llost[CurL] = true;
		}

		//找右线并判断是否丢线
		Rlost[CurL] = false;
		CurPoint = Cur_Offset;
		while (CurPoint + 2 <= col_num - 1)
		{
			if (binImg[CurL][CurPoint] && !binImg[CurL][CurPoint + 1] && !binImg[CurL][CurPoint + 2]) /* 找到左边界  并且进行去噪 */
			{
				Rx[CurL] = CurPoint + 1;
				break;
			}
			else
			{ /* 没找到，向左移动一个像素点 */
				++CurPoint;
			}
		}
		if (CurPoint > col_num - 3) //为col_num-2或者丢线时
		{
			Rx[CurL] = col_num - 1;
			if (!(binImg[CurL][col_num - 2] == 0xff && binImg[CurL][col_num - 1] == 0)) //丢线
				Rlost[CurL] = true;
		}

		//生成中线

		Midx[CurL] = (Lx[CurL] + Rx[CurL]) >> 1;
		if (Llost[CurL] && Rlost[CurL])
		{
			Midx[CurL] = car_center;
		}
		//求变化率
		if (CurL < col_num - 1)
		{
			dRx[CurL] = Rx[CurL] - Rx[CurL + 1];
			dLx[CurL] = Lx[CurL] - Lx[CurL + 1];
		}
		else
		{
			dRx[CurL] = 0;
			dLx[CurL] = 0;
		}

		//找合点 有可能会有bug 再说
		if ((Rx[CurL] - Lx[CurL] <= 3) || (CurL != row_num - 1 && !binImg[CurL - 1][Midx[CurL]])) //如果到达合点，则退出巡线
		{
			joinRow = CurL + 1 > startRow ? startRow : CurL + 1;
			break;
		}
		Cur_Offset = Midx[CurL];
	}

	lLostNum = 0;
	rLostNum = 0;
	while (i <= row_num - 1)// 是row不是col
	{
		if (Rlost[i])
		{
			rLostNum++;
		}
		if (Llost[i])
		{
			lLostNum++;
		}
		i++;
	}
}

void JudgeRoadType(void)
{
	
	bool pdAreaNoLost = true;
	//预判断
	for (int i = row_num - 1; i >= startRow; i--)
	{
		if (Llost[i] || Rlost[i])
		{
			pdAreaNoLost = false;
			break;
		}
	}
	if (roadType == portIn || roadType == portIn2)
	{
		roadType=portIn2;
		goto end;
	}
	if (pdAreaNoLost && !rdb_type) //pd区正常则无需判断 是normal
	{
		roadType = normal;
	}
	else 
	{
		if (lLostNum >= THR && rLostNum >= THR && !rdb_type)
		{
			roadType = cross;
			goto end;
		}
		int flag1 = 0;//用于环岛判断，值为右侧线 有线→丢线→有线 丢线到有线的行 为0时代表右边没丢
		int flag2 = 0;//flag2=0代表没有再出现往右的白线
		
		//情况：图rdb4之前
		if (!rdb_type && !isRdb && !Rlost[row_num - 1] && lLostNum <= 1) //非环岛附近，右下角不丢线
		{
			for (int i = row_num - 3; i >= joinRow; i--) //然后出现连续丢线
			{
				if (Rlost[i - 1] && Rlost[i - 2] && Rlost[i - 3])
				{
					flag1 = i - 3;
				}
				else
				{
					if (flag1)
						break;
				}
			}
			if (flag1)
			{
				for (int i = flag1 - 1; i >= joinRow; i--) //右边线 左拐/左突跃
				{
					if (dRx[i] <= -2 && dRx[i - 1] <= -2 && dRx[i - 2] <= -2)
					{
						flag2 = i - 2;
					}
					else
					{
						if (flag2)
							break;
					}
				}
			}
			else
			{
				roadType = normal;
			}

			if (flag2)
			{
				for (int i = flag2 - 1; i >= joinRow; i--)
				{
					if (dRx[i] >= 2) //环的另一边
					{
						roadType = rdb_pre;
						break;
					}
					// 	if (i == joinRow)
					// 	{
					// 		roadType = rdb_flag1;
					// 	}
				}
			}
		}

		// 情况：图rdb4
		if (roadType == rdb_pre)
			roadType = (Rlost[row_num - 1] && Rlost[row_num - 2] && Rlost[row_num - 3]) ? rdb_pre2 : rdb_pre;

		else if (roadType == rdb_pre2)
			roadType = (!Rlost[row_num - 1] && !Rlost[row_num - 2] && !Rlost[row_num - 3]) ? rdb_flag1 : rdb_pre2;

		//情况：图rbd5
		else if (roadType == rdb_flag1)
			roadType = (Rlost[row_num - 1] && Rlost[row_num - 2] && Rlost[row_num - 3]) ? rdb_flag2 : rdb_flag1;
		
		//情况：图rdb6
		else if (roadType == rdb_flag2)
			roadType = ((Llost[row_num - 2] && Llost[row_num - 3])) ? rdb_flag3 : rdb_flag2;

		//完全进环  如同弯道
		else if (roadType == rdb_flag3)
		{
			if (!Llost[row_num - 1] && !Llost[row_num - 2])
			{
				roadType = rdb_flag4;
				isRdb = true;
			}
			else
			{
				roadType = rdb_flag3;
			}
		}

		//出环
		else if (roadType == rdb_flag4)
			roadType = (lLostNum >= 5) ? rdb_flag5 : rdb_flag4;

		//出环后碰到了入环处
		else if (roadType == rdb_flag5)
			roadType = (lLostNum <= 2) ? rdb_flag6 : rdb_flag5;

		//正常行驶
		else if (roadType == rdb_flag6)
		{
			roadType = (rLostNum <= 5) ? normal : rdb_flag6;
			isRdb = false;
		}

		if (!isRdb && !rdb_type) //检测斑马线
		{
			int tar[4] = {32, 37, 27 ,22};
			int cnt[4] = {0};
			int ll = 20;
			uint8_t thisFlag = 0;
			for (int j = 0; j < 4; j++)
			{
				thisFlag = 0;
				for (int i = ll; i < 151 - ll; i++)
				{
					if (binImg[tar[j]][i] != thisFlag)
					{
						cnt[j]++;
						thisFlag = ~thisFlag;
					}
				}
			}
			if (cnt[0] + cnt[1] + cnt[2] + cnt[3] >= 14)
			{
				roadType = portIn;
			}
		}
		
	}
end:
	roadType=normal;
}

// normal cross不接管  portIn必接管
void ProcessImgByType(void)
{
	switch (roadType)
	{
		case rdb_flag1:
			specialDeal = false;
			break;
		case rdb_flag2:
			specialDeal = true;
			break;
		case rdb_flag3:
			specialDeal = true;
			break;
		case rdb_flag4:
			specialDeal = false;
			break;
		case rdb_flag5:
			specialDeal = true;
			break;
		case rdb_flag6:
			specialDeal = false;
			break;
		case cross:
			specialDeal = false;
			crossImgProcess();
            break;
		case portIn:
			specialDeal = true;
			break;
		case rdb_pre:
			specialDeal = false;
			//rdbPreImgProcess();
			break;
		case portIn2:
			specialDeal = true;
			break;
		default:
		specialDeal = false;
			break;
		}
}

void crossImgProcess(void)
{
	int i;
	float leftAppear_Lost,rightAppear_Lost;

	int leftLost = row_num - 1, rightLost = row_num - 1;
	int leftAppear = joinRow, rightAppear = joinRow;
	//找左边消失和出现点
	for(i=row_num-1;i>=joinRow;i--)
	{
		if(dLx[i]<-THR_JMP)
		{
			leftLost = i + 1;
			break;
		}
	}
	for (i = joinRow; i <= row_num - 1; i++)
	{
		if (dLx[i] > THR_JMP && !dLx[i + 1] > THR_JMP)
		{
			leftAppear = i;
			break;
		}
	}

	//找右边消失和出现点
	for(i=row_num-1;i>=joinRow;i--)
	{
		if(dRx[i]>THR_JMP)
		{
			rightLost = i + 1;
			break;
		}
	}
	for (i = joinRow; i <= row_num - 1; i++)
	{
		if (dRx[i] < -THR_JMP&&!dRx[i+1] < -THR_JMP)
		{
			rightAppear = i;
			break;
		}
	}

	//找到后补线即可。

	leftAppear_Lost=Lx[leftAppear]-Lx[leftLost];
	rightAppear_Lost=Rx[rightAppear]-Rx[rightLost];
	if (leftAppear != joinRow)
	{
		for (i = leftLost; i >= leftAppear; i--)
		{
			Lx[i] = Lx[leftLost] + (int)(leftAppear_Lost / (leftLost - leftAppear) * (leftLost - i));
		}
	}

	if (rightAppear != joinRow)
	{
		for (i = rightLost; i >= rightAppear; i--)
		{
			Rx[i] = Rx[leftLost] + (int)(rightAppear_Lost / (rightLost - rightAppear) * (rightLost - i));
		}
	}
//其他情况不补线也没关系。
	//最后再求中线
	for (i = row_num - 1; i >= joinRow; i--)
	{
		Midx[i] = (Lx[i] + Rx[i]) >> 1;
		if (Lx[i] <= 1 && Rx[i] >= row_num - 2)
		{
			Midx[i] = car_center;
		}
	}
}

void rdbPreImgProcess(void)
{
	//找到右出现点（从下往上看，右边线突然往左）
	int rightAppear = joinRow;
	for (int i = joinRow; i <= row_num - 1; i++)
	{
		if (dRx[i] < -THR_JMP)
		{
			rightAppear = i;
			break;
		}
	}

	int rightStart = row_num - 1;

	//找到右下角补线开始点
	for (int i = row_num - 1; i >= rightAppear; i--)
	{
		if (dRx[i] > 0)
		{
			rightStart = i;
			break;
		}
	}

	if(rightStart==rightAppear){}
	else
	{
		int rightAppearStart = Rx[rightAppear]-Rx[rightStart];
		for (int i = rightStart; i >= rightAppear; i--)
		{
			Rx[i] = Rx[rightStart] + (int)((rightStart - i) * rightAppearStart / (rightStart - rightAppear));
		}
		//重新求中线
		for (int i = row_num - 1; i >= joinRow; i--)
			Midx[i] = (Lx[i] + Rx[i]) >> 1;
	}
	
}

//返回舵机应当的占空比
/*unsigned int Steer_Control_M1(void)
{
	int i;
	int ErrorSum = 0, Error, ChangeDuty;
	int TmpDuty;
	int tmpStartRow = startRow < joinRow ? joinRow : startRow;
	uint32_t Duty;
	for (i = tmpStartRow; i <= endRow; i++)
	{
		ErrorSum += (car_center - Midx[i]);
	}
	Error = ErrorSum / (endRow - startRow + 1);
	ChangeDuty = (int)(KP * Error + KD * (Error - LastError));
	LastError = Error;
	TmpDuty = ChangeDuty;
	Duty = TmpDuty < 0 ? 0 : TmpDuty;
	return Duty;
}

//返回速度值
int Motor_Control(void)
{
	int errorSum=0, motorSpeed;
	far_error = 0;
	for (int i = startRow; i <= endRow; i++)
	{
		errorSum += (car_center - Midx[i]);
	}
	far_error = errorSum / (endRow_m - startRow_m - 1);
	far_error = far_error < 0 ? -far_error : far_error;
	motorSpeed = 1000 - 0.8*far_error;
	return motorSpeed;
}

void setSpeed(int16_t left,int16_t right)
{
  
}

void SteerAndMotorControl(void)
{
	int16_t _motor_speed_left, _motor_speed_right;
	int16_t Motor_speed = Motor_Control();
	int16_t Steer_pwm = Steer_Control_M1();
	double rad;
	double ratio;
	rad = ((double)(Steer_pwm - STEER_MID))/128; //弧度
	rad = rad > 0 ? rad : -rad;
	ratio = 1 + 1.5*tan(rad);
	if (Steer_pwm > STEER_MID)
	{		
		_motor_speed_left=Motor_speed;
		_motor_speed_right = Motor_speed * ratio;
	}
	else
	{
		_motor_speed_left = Motor_speed * ratio;
		_motor_speed_right = Motor_speed;
	}
	//入库出库控制
	if(specialDeal)
	{
		if (roadType == portIn)//入库
		{
			is_pid_motor = false;
#ifdef inDebug
			ChangeSteer(STEER_LEFT - 20);
#else
			ChangeSteer(STEER_RIGHT + 10);
#endif
			ChangeMotor(0);
		}
		else //环岛拐弯的时候
		{
			ChangeSteer(STEER_RIGHT);
			setSpeed(300 * ratio, 300);
		}
	}
	else
	{

		ChangeSteer(Steer_pwm);
		if (roadType == rdb_pre) //进环前减速
		{
			setSpeed(320, 320);
		}
		else if (rdb_type)
		{
			setSpeed(350 * ratio, 350);
		}
		else
			setSpeed(_motor_speed_left , _motor_speed_right);
	}
}
*/


/*************将处理后的图像显示出来***************/
unsigned char display_col[158] = {0, 0, 1, 2, 3, 4, 4, 5, 6, 7, 8, 8, 9, 10, 11, 12, 12, 13, 14,
								  15, 16, 17, 17, 18, 19, 20, 21, 21, 22, 23, 24, 25, 25, 26, 27,
								  28, 29, 29, 30, 31, 32, 33, 34, 34, 35, 36, 37, 38, 38, 39, 40,
								  41, 42, 42, 43, 44, 45, 46, 46, 47, 48, 49, 50, 51, 51, 52, 53,
								  54, 55, 55, 56, 57, 58, 59, 59, 60, 61, 62, 63, 64, 64, 65, 66,
								  67, 68, 68, 69, 70, 71, 72, 72, 73, 74, 75, 76, 76, 77, 78, 79,
								  80, 81, 81, 82, 83, 84, 85, 85, 86, 87, 88, 89, 89, 90, 91, 92,
								  93, 93, 94, 95, 96, 97, 98, 98, 99, 100, 101, 102, 102, 103, 104,
								  105, 106, 106, 107, 108, 109, 110, 110, 111, 112, 113, 114, 115,
								  115, 116, 117, 118, 119, 119, 120, 121, 122, 123, 123, 124, 125, 126, 127};

void dispimage(void)
{
	uint16_t i = 0, j = 0;
	for (i = 0; i < row_num; i++)
	{
		for (j = 0; j < col_num; j++)
		{
			if (binImg[i][j])
			{
				ips114_drawpoint(display_col[j], i + 14, 1);
			}
			else
			{
				ips114_drawpoint(display_col[j], i + 14, 0);
			}
			if (Midx[i] != 0 && Midx[i] < col_num && i >= joinRow)
				ips114_drawpoint(display_col[Midx[i]], i + 14, 0);
		}
	}
	ips114_showstr(5,0,"                                           ");
	ips114_showuint8(5, 0, lLostNum);
	ips114_showuint8(25, 0, rLostNum);
	ips114_showuint8(45, 0, joinRow);
	ips114_showuint8(65, 0, roadType);
	ips114_clear(BLACK);
}

/* 显示找到的边线 */
void dispimage1(void)
{
	uint16_t i = 0;
	ips114_clear(BLACK);
	for (i = joinRow; i < row_num; i++)
	{
		/*
		for ( j = 0; j < col_num; j++ )
		{
			if ( imgadd[i * col_num + j] > whiteRoad )
			{
				OLED_DrawPoint( display_col[j], i + 14, 0 );
			}else {
				OLED_DrawPoint( display_col[j], i + 14, 0 );
			}
		}
*/

		/* 画出找到的边界线 */
		if (Lx[i] != col_num && Lx[i] > 0)
			ips114_drawpoint(display_col[Lx[i]], i + 14, 1);
		if (Rx[i] != 0 && Rx[i] < col_num)
			ips114_drawpoint(display_col[Rx[i]], i + 14, 1);
		if (Midx[i] != 0 && Midx[i] < col_num)
			ips114_drawpoint(display_col[Midx[i]], i + 14, 1);
	}
	ips114_clear(BLACK);
}