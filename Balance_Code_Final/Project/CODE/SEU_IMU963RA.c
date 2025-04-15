#include "SEU_IMU963RA.h"
#define AccFilterNum 8

//郑鹏算法
float acc_ratio1 = 0.85;      //加速度计比例
float acc_ratio2 = 0.95;      //加速度计比例
float gyro_ratio1 = 0.995;     //陀螺仪比例
float dt_imu = 0.005;             //采样周期


//-------------------------------------------------------------------------------------------------------------------
// @brief		角速度获取函数
// @return		void
// Sample usage:        Get_gyro_data();
// @note		无
//-------------------------------------------------------------------------------------------------------------------
void Get_gyro_data_imu(void)
{
	//imu963ra_get_gyro1();

	Gyro_x=(float)imu963ra_gyro_z/16.384f;//转换关系:gyro/(2^16/2)*2000°/s, 16.384
	Gyro_y=(float)imu963ra_gyro_x/16.384f;
	Gyro_z=(float)imu963ra_gyro_x/16.384f;
}




//-------------------------------------------------------------------------------------------------------------------
// @brief		角加速度获取函数
// @return		void
// Sample usage:        Get_acc_data();
// @note		无
//-------------------------------------------------------------------------------------------------------------------
void Get_acc_data_imu(void)
{
	uint8 i;
	int32 acc_sum[3]={0};

	for(i=0;i < AccFilterNum;i++)
	{
		imu963ra_get_acc1();
		acc_sum[0]  += imu963ra_acc_z;
		acc_sum[1]  += imu963ra_acc_y;
		acc_sum[2]  -= imu963ra_acc_x;
		systick_delay_us(1);
	}
	
	Acc_x=(float)acc_sum[0]/3343.67347f;//转换关系:acc/(2^16/2)*8g = acc_sum/AccFilterNum/(2^16/2)*8g, g取9.8
	Acc_y=(float)acc_sum[1]/3343.67347f;
	Acc_z=(float)acc_sum[2]/3343.67347f;
}




//-------------------------------------------------------------------------------------------------------------------
// @brief		解算角度函数
// @return		void
// Sample usage:        IMU_zs(icm_gyro_x, icm_gyro_y, icm_gyro_z, icm_acc_x, icm_acc_y, icm_acc_z);
// @note		此函数使用的是原始数据，即加速度计和陀螺仪直接测出的原始数据。
//-------------------------------------------------------------------------------------------------------------------
float angle_acc_pitch_imu = 0;
float angle_acc_yaw_imu = 0;//z轴的偏航角
float angle_acc_roll_imu = 0;

void IMU_read(float gx, float gy, float gz, float ax, float ay, float az)
{
  float angle_error;
  
  angle_acc_pitch_imu = atan(ay*invSqrt(pow(ax,2)+pow(az,2)))*180.0/M_PI;
  angle_acc_roll_imu = atan(-ax*invSqrt(pow(az,2)+pow(ay,2)))*180.0/M_PI;
  //angle_acc_yaw_imu = atan(ay*invSqrt(pow(ax,2)+pow(az,2)))*180.0/M_PI;
  
  //angle_acc_yaw=atan(1/(invSqrt(pow(ay,2)+pow(ax,2))*az))*180.0/M_PI;
  ggx = gx*0.061035156;//roll
  ggy = gy*0.061035156;//pitch
  ggz = gz*0.061035156;//yaw
  
  
  eulerAngle.pitch = imu_angle_calc_pitch(angle_acc_pitch_imu,ggx);
  eulerAngle.roll = imu_angle_calc_roll(angle_acc_roll_imu,ggy);
}


//-------------------------------------------------------------------------------------------------------------------
// @brief		解算角度函数
// @return		void
// Sample usage:        angle_calc_pitch(angle_acc_pitch,ggy);
// @note		此函数使用的是原始数据，即加速度计和陀螺仪直接测出的原始数据。
//-------------------------------------------------------------------------------------------------------------------
float imu_angle_calc_pitch(float angle_m, float gyro_m)
{

    float temp_angle;
    float error_angle;
    float gyro_now;
    static float last_angle0;
    static uint8 first_angle;

    if(!first_angle)//判断是否为第一次运行本函数
    {
        //如果是第一次运行，则将上次角度值设置为与加速度值一致
        first_angle = 1;
        last_angle0 = angle_m;
    }

    gyro_now = gyro_m * gyro_ratio1;
    //根据测量到的加速度值转换为角度之后与上次的角度值求偏差
    error_angle = (angle_m - last_angle0)*acc_ratio1;
    //根据偏差与陀螺仪测量得到的角度值计算当前角度值
    temp_angle = last_angle0 + (error_angle + gyro_now)*dt_imu;
    //保存当前角度值
    last_angle0 = temp_angle;


    return temp_angle;

}


float imu_angle_calc_roll(float angle_m, float gyro_m)
{

    float temp_angle1;
    float error_angle;
    float gyro_now;
    static float last_angle1;
    static uint8 first_angle1;

    if(!first_angle1)//判断是否为第一次运行本函数
    {
        //如果是第一次运行，则将上次角度值设置为与加速度值一致
        first_angle1 = 1;
        last_angle1 = angle_m;
    }

    gyro_now = gyro_m * gyro_ratio1;
    //根据测量到的加速度值转换为角度之后与上次的角度值求偏差
    error_angle = (angle_m - last_angle1)*acc_ratio1;
    //根据偏差与陀螺仪测量得到的角度值计算当前角度值
    temp_angle1 = last_angle1 + (error_angle + gyro_now)*dt_imu;
    //保存当前角度值
    last_angle1 = temp_angle1;


    return temp_angle1;

}


static float getYaw(float accVals[3], float magVals[3]) 
{
	//float roll = (float)atan2(accVals[0],accVals[2]);
	//float pitch = -(float)atan(accVals[1]/(accVals[0]*sin(roll)+accVals[2]*cos(roll)));
	//float yaw = (float)atan2(magVals[2]*sin(eulerAngle.roll*(M_PI/180.0))*sin(eulerAngle.pitch*(M_PI/180.0))+magVals[1]*cos(eulerAngle.roll*(M_PI/180.0))*sin(eulerAngle.pitch*(M_PI/180.0))+magVals[0]*cos(eulerAngle.pitch*(M_PI/180.0)),
	//	   -magVals[2]*cos(eulerAngle.roll*(M_PI/180.0))+magVals[1]*sin(eulerAngle.roll*(M_PI/180.0)));
        float yaw = (float)atan2(magVals[2]*sin(eulerAngle.roll*(M_PI/180.0))*sin(eulerAngle.pitch*(M_PI/180.0)) + magVals[1]*cos(eulerAngle.roll*(M_PI/180.0))*sin(eulerAngle.pitch*(M_PI/180.0)) + magVals[0]*cos(eulerAngle.pitch*(M_PI/180.0)),
		   -magVals[2]*cos(eulerAngle.roll*(M_PI/180.0))+magVals[1]*sin(eulerAngle.roll*(M_PI/180.0)));
	return yaw;
}

float AC_Azimuth(s16 ax,s16 ay, s16 az, s16 mx, s16 my, s16 mz)
{
	float accVals[3], magVals[3];
	float ftemp;
	
	accVals[0] = -ax;
	accVals[1] = az;
	accVals[2] = ay;
	magVals[0] = mx;
	magVals[1] = mz;
	magVals[2] = my;
			
	ftemp = getYaw(accVals, magVals) * 180.0f / 3.141593f;
	if(ftemp > 0) ftemp = -180.0f + (ftemp - 180.0f) ;
	ftemp = 0.0f - ftemp;
	ftemp += 90.0f;
	ftemp += -5.0f;	//补偿磁偏角，不同地区会不一样,此处给出的-5是江苏地区的补偿磁偏角
	if(ftemp > 360.0f) ftemp -= 360.0f;
        //if(ftemp < 0) ftemp += 360;// -180.0f + (ftemp - 180.0f) ;
	return ftemp;	
}


//-------------------------------------------------------------------------------------------------------------------
// @brief		总体调用函数
// @return		void
// Sample usage:        IMUread();
// @note		获取Pitch Yaw Roll等只使用本函数即可（参量在上位机输出）
//-------------------------------------------------------------------------------------------------------------------
void IMUread_IMU963RA(){
  imu963ra_get_acc1();
  imu963ra_get_gyro1();
  imu963ra_get_mag1();
  Get_gyro_data_imu();
  IMU_read(imu963ra_gyro_x, imu963ra_gyro_y, imu963ra_gyro_z, imu963ra_acc_x, imu963ra_acc_y, imu963ra_acc_z);
  //eulerAngle.yaw = (180.0/M_PI)*atan(-(imu963ra_mag_y*cos(eulerAngle.roll)-imu963ra_mag_z*sin(eulerAngle.roll))/(imu963ra_mag_x*cos(eulerAngle.pitch)+imu963ra_mag_y*sin(eulerAngle.pitch)*sin(eulerAngle.roll)+imu963ra_mag_z*sin(eulerAngle.pitch)*cos(eulerAngle.roll)));
  eulerAngle.yaw = AC_Azimuth(imu963ra_acc_x, imu963ra_acc_y, imu963ra_acc_z, imu963ra_mag_x,imu963ra_mag_y, imu963ra_mag_z);
  //eulerAngle.yaw = (atan2((double)imu963ra_acc_y,(double)imu963ra_acc_x)*(180 / M_PI)+180);  
}
