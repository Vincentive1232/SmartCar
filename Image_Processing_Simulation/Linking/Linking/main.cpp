/*******************************************************************************
* File Name: AppBwLabel.c
*    Author: Showlo
*    Created: 2018-04-18
*    Abstract:
*    Reference:
*    Version:
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>
using namespace std::chrono;

#define AFE_DATA_SIZE 809
#define CH_X_NUM             18     // 矩阵行数
#define CH_Y_NUM             32     // 矩阵列数

/* basic data type */
typedef signed char         SINT8;
typedef signed short        SINT16;
typedef signed long         SINT32;
typedef unsigned char       UINT8;
typedef unsigned short      UINT16;
typedef unsigned short      ADDR;
typedef unsigned long       UINT32;
typedef float               FP32;
typedef double              FP64;

/* 数据源矩阵Buff */
typedef struct {
	SINT16 ssDiffBuf[AFE_DATA_SIZE];
} ST_FrameData;

/*
*0 1 2
*3   4
*5 6 7
*/
typedef enum
{
	NEI_L_UP = 0,          // 左上角
	NEI_UP = 1,            // 上
	NEI_R_UP = 2,          // 右上角
	NEI_LEFT = 3,          // 左
	NEI_RIGHT = 4,         // 右
	NEI_L_DOWN = 5,         // 左下角
	NEI_DOWN = 6,          // 下
	NEI_R_DOWN = 7,        // 右下角
} ENUM_NEI_DIR;

/*******************************************************************************
* 3.Global constant and macro definitions using #define
*******************************************************************************/
/* 数据源矩阵 */
#define CH_ORDER {\
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,\
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\
}
/*******************************************************************************
* 4.Static variables
*******************************************************************************/
/*******************************************************************************
* 5.Global variable or extern global variabls/functions
*******************************************************************************/
ST_FrameData g_stFrmData = { CH_ORDER };
SINT16 ssTemp[8] = { -33,-32,-31,-1,1,31,32,33 }; // 八连通相对位置
												/*Debug Printf*/
void AppPrintfBuf(void)
{
	int i, j, index;
	printf("\nFrameData:\n");
	printf("  ");
	for (i = 0; i < 32; i++)
	{
		printf("%2d ", i);
	}
	for (i = 0; i < CH_X_NUM; i++)
	{
		printf("\n\r%2d", i);
		for (j = 0; j < CH_Y_NUM; j++)
		{
			index = i * CH_Y_NUM + j;
			printf("%2d ", g_stFrmData.ssDiffBuf[index]);
		}
	}
	printf("\n\r");
}

/*******************************************************************************
*   Name: AppMarkNeighbours
*  Brief: 标记当前通道，并按8个方向查找是否存在领域前景数据,找到有前景数据将进行递归标记
*  Input: SINT16 *ssDiff
* Output: None
* Return: None
*******************************************************************************/
void AppMarkNeighbours(UINT16 ssChanel, UINT16 scLabel)
{
	UINT8 i;
	/* 标记 */
	g_stFrmData.ssDiffBuf[ssChanel] = scLabel;
	/* 8邻域查找并递归 */
	for (i = 0; i < 8; i++)
	{
		if (g_stFrmData.ssDiffBuf[ssTemp[i] + ssChanel] == 1)
		{
			AppMarkNeighbours((ssTemp[i] + ssChanel), scLabel);
		}
	}
	return;
}

void AppBwLabel(SINT16* ssDiff)
{
	SINT8 i, j, ucTxNumVA = CH_X_NUM, ucRxNum = CH_Y_NUM;
	UINT16  usIndex = 0, ucLabel = 1; // 初始1，用于区分前景数据与背景数据

	/* 对矩阵源从头到尾遍历一次 */
	for (i = 0; i < ucTxNumVA; i++)
	{
		for (j = 0; j < ucRxNum; j++)
		{
			usIndex = i * ucRxNum + j;

			if (ssDiff[usIndex] != 0 && ssDiff[usIndex] == 1)
			{
				ucLabel++;
				AppMarkNeighbours(usIndex, ucLabel);
			}
		}
	}

	/*对连通域的起始编码调整为最小自然数*/
	for (i = 0; i < ucTxNumVA; i++)
	{
		for (j = 0; j < ucRxNum; j++)
		{
			usIndex = i * ucRxNum + j;

			if (ssDiff[usIndex] != 0)
			{
				ssDiff[usIndex] = ssDiff[usIndex] - 1;
			}
		}
	}
}

int main(void)
{
	high_resolution_clock::time_point t1 = high_resolution_clock::now(); //返回时间戳
	AppPrintfBuf();
	// 连通域标记算法入口
	AppBwLabel(g_stFrmData.ssDiffBuf);
	AppPrintfBuf();
	high_resolution_clock::time_point t2 = high_resolution_clock::now();

	duration<double, std::milli> time_span = t2 - t1;

	std::cout << time_span.count() << std::endl;
	system("pause");
	return 0;
}