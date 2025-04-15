#include "SEU_Image_Processing.h"

#define grayImg mt9v03x_image
#define THR 7
#define THR_JMP 3
#define rdb_type (roadType<8||roadType==11||roadType==12)
//�жϵ�·����ʱ�õ�һ����ֵ


//��������
uint8 Lx[row_num];
uint8 Rx[row_num];
uint8 Midx[row_num];

//�����߱仯��
int8 dLx[row_num];
int8 dRx[row_num];


//��Ӧ���Ƿ���
bool Llost[row_num];
bool Rlost[row_num];

bool cam_flag=false;//����ͷ�Ƿ����˸��£��������false

//���߼���
int lLostNum = 0, rLostNum = 0;

uint8 whiteRoad = 150; //ȫ����ֵ
uint8 car_center = 94; //��ģ����λ��


// steer error����ĳ�ʼ��ĩβ��
int startRow = 23;
int endRow = 28;

// motor far_error����ĳ�ʼ��ĩβ��
int startRow_m = 21;
int endRow_m =  23;

// �ϵ� ������׼ȷ��������Զ���� ԽСԽԶ��
int joinRow = 0;

//·����
roadTypeEnum roadType = normal; //֮��Ҫ�ĵ�

bool isRdb = false;//�Ƿ��������
int recommendedPwm = 0;

//��ʱ
uint32_t endts;
//�Ƿ����⴦��
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

	int CurL = 0, Start = 0;	 /* CurL  ��ǰ��   Start ��ʼɨ�ߵ���  ��һ�д�0��ʼ */
	int Cur_Offset = car_center; /* ��ʼɨ������Ϊ80���˱���Ϊ��ǰ������ */
	int i = joinRow;
	int CurPoint = Cur_Offset; /* CurPointΪ��ǰ����ɨ��ĵ� */
							   /*
 * ע�⣺���Ͻ�Ϊ��0,0��
 * ====================����ɨ��������===============================
 */
	for (CurL = row_num - 1; CurL >= Start; --CurL)
	{
		//�����߲��ж��Ƿ���
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
			{ /* û�ҵ��������ƶ�һ�����ص� */
				--CurPoint;
			}
		}
		if (CurPoint < 2) // Ϊ0���߶���ʱ
		{
			Lx[CurL] = 0;
			if (!(binImg[CurL][1] == 0xff && binImg[CurL][0] == 0)) //����
				Llost[CurL] = true;
		}

		//�����߲��ж��Ƿ���
		Rlost[CurL] = false;
		CurPoint = Cur_Offset;
		while (CurPoint + 2 <= col_num - 1)
		{
			if (binImg[CurL][CurPoint] && !binImg[CurL][CurPoint + 1] && !binImg[CurL][CurPoint + 2]) /* �ҵ���߽�  ���ҽ���ȥ�� */
			{
				Rx[CurL] = CurPoint + 1;
				break;
			}
			else
			{ /* û�ҵ��������ƶ�һ�����ص� */
				++CurPoint;
			}
		}
		if (CurPoint > col_num - 3) //Ϊcol_num-2���߶���ʱ
		{
			Rx[CurL] = col_num - 1;
			if (!(binImg[CurL][col_num - 2] == 0xff && binImg[CurL][col_num - 1] == 0)) //����
				Rlost[CurL] = true;
		}

		//��������

		Midx[CurL] = (Lx[CurL] + Rx[CurL]) >> 1;
		if (Llost[CurL] && Rlost[CurL])
		{
			Midx[CurL] = car_center;
		}
		//��仯��
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

		//�Һϵ� �п��ܻ���bug ��˵
		if ((Rx[CurL] - Lx[CurL] <= 3) || (CurL != row_num - 1 && !binImg[CurL - 1][Midx[CurL]])) //�������ϵ㣬���˳�Ѳ��
		{
			joinRow = CurL + 1 > startRow ? startRow : CurL + 1;
			break;
		}
		Cur_Offset = Midx[CurL];
	}

	lLostNum = 0;
	rLostNum = 0;
	while (i <= row_num - 1)// ��row����col
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
	//Ԥ�ж�
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
	if (pdAreaNoLost && !rdb_type) //pd�������������ж� ��normal
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
		int flag1 = 0;//���ڻ����жϣ�ֵΪ�Ҳ��� ���ߡ����ߡ����� ���ߵ����ߵ��� Ϊ0ʱ�����ұ�û��
		int flag2 = 0;//flag2=0����û���ٳ������ҵİ���
		
		//�����ͼrdb4֮ǰ
		if (!rdb_type && !isRdb && !Rlost[row_num - 1] && lLostNum <= 1) //�ǻ������������½ǲ�����
		{
			for (int i = row_num - 3; i >= joinRow; i--) //Ȼ�������������
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
				for (int i = flag1 - 1; i >= joinRow; i--) //�ұ��� ���/��ͻԾ
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
					if (dRx[i] >= 2) //������һ��
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

		// �����ͼrdb4
		if (roadType == rdb_pre)
			roadType = (Rlost[row_num - 1] && Rlost[row_num - 2] && Rlost[row_num - 3]) ? rdb_pre2 : rdb_pre;

		else if (roadType == rdb_pre2)
			roadType = (!Rlost[row_num - 1] && !Rlost[row_num - 2] && !Rlost[row_num - 3]) ? rdb_flag1 : rdb_pre2;

		//�����ͼrbd5
		else if (roadType == rdb_flag1)
			roadType = (Rlost[row_num - 1] && Rlost[row_num - 2] && Rlost[row_num - 3]) ? rdb_flag2 : rdb_flag1;
		
		//�����ͼrdb6
		else if (roadType == rdb_flag2)
			roadType = ((Llost[row_num - 2] && Llost[row_num - 3])) ? rdb_flag3 : rdb_flag2;

		//��ȫ����  ��ͬ���
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

		//����
		else if (roadType == rdb_flag4)
			roadType = (lLostNum >= 5) ? rdb_flag5 : rdb_flag4;

		//�������������뻷��
		else if (roadType == rdb_flag5)
			roadType = (lLostNum <= 2) ? rdb_flag6 : rdb_flag5;

		//������ʻ
		else if (roadType == rdb_flag6)
		{
			roadType = (rLostNum <= 5) ? normal : rdb_flag6;
			isRdb = false;
		}

		if (!isRdb && !rdb_type) //��������
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

// normal cross���ӹ�  portIn�ؽӹ�
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
	//�������ʧ�ͳ��ֵ�
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

	//���ұ���ʧ�ͳ��ֵ�
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

	//�ҵ����߼��ɡ�

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
//�������������Ҳû��ϵ��
	//�����������
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
	//�ҵ��ҳ��ֵ㣨�������Ͽ����ұ���ͻȻ����
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

	//�ҵ����½ǲ��߿�ʼ��
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
		//����������
		for (int i = row_num - 1; i >= joinRow; i--)
			Midx[i] = (Lx[i] + Rx[i]) >> 1;
	}
	
}

//���ض��Ӧ����ռ�ձ�
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

//�����ٶ�ֵ
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
	rad = ((double)(Steer_pwm - STEER_MID))/128; //����
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
	//���������
	if(specialDeal)
	{
		if (roadType == portIn)//���
		{
			is_pid_motor = false;
#ifdef inDebug
			ChangeSteer(STEER_LEFT - 20);
#else
			ChangeSteer(STEER_RIGHT + 10);
#endif
			ChangeMotor(0);
		}
		else //���������ʱ��
		{
			ChangeSteer(STEER_RIGHT);
			setSpeed(300 * ratio, 300);
		}
	}
	else
	{

		ChangeSteer(Steer_pwm);
		if (roadType == rdb_pre) //����ǰ����
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


/*************��������ͼ����ʾ����***************/
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

/* ��ʾ�ҵ��ı��� */
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

		/* �����ҵ��ı߽��� */
		if (Lx[i] != col_num && Lx[i] > 0)
			ips114_drawpoint(display_col[Lx[i]], i + 14, 1);
		if (Rx[i] != 0 && Rx[i] < col_num)
			ips114_drawpoint(display_col[Rx[i]], i + 14, 1);
		if (Midx[i] != 0 && Midx[i] < col_num)
			ips114_drawpoint(display_col[Midx[i]], i + 14, 1);
	}
	ips114_clear(BLACK);
}