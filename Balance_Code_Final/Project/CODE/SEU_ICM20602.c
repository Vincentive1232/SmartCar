#include "SEU_ICM20602.h"
#define AccFilterNum 8


float Gyro_x,Gyro_y,Gyro_z;//角速度
float Acc_x,Acc_y,Acc_z;//角加速度
float Last_Angle = 0;
float x1,x2,y1;
//zs**************************************   
float last_ex=0,last_ey=0,last_ez=0;//

float I_ex, I_ey, I_ez;  // 误差积分
quaterInfo_t Q_info = {-0.029,0.068,-0.997,0};  // 全局四元数
eulerianAngles_t eulerAngle; //欧拉角(包含了Pitch，Yaw。roll）
float param_Kp = 350;//50.0;   // 加速度计(磁力计)的收敛速率比例增益50 
float param_Ki = 3;//0.20;   //陀螺仪收敛速率的积分增益 0.2
float param_Kd = 0;


float K1 =0.027; // 对加速度计取值的权重
float K2 =0.2; // 对加速度计取值的权重


//郑鹏算法
float ggx=0,ggy=0,ggz=0;
float acc_ratio=0.85;      //加速度计比例
float gyro_ratio=0.995;     //陀螺仪比例
float dt=0.005;             //采样周期


//-------------------------------------------------------------------------------------------------------------------
// @brief		角速度获取函数
// @return		void
// Sample usage:        Get_gyro_data();
// @note		无
//-------------------------------------------------------------------------------------------------------------------
void Get_gyro_data(void)
{
	get_icm20602_gyro_spi();

	Gyro_x=(float)icm_gyro_z/16.384f;//转换关系:gyro/(2^16/2)*2000°/s, 16.384
	Gyro_y=(float)icm_gyro_y/16.384f;
	Gyro_z=-(float)icm_gyro_x/16.384f;
}




//-------------------------------------------------------------------------------------------------------------------
// @brief		角加速度获取函数
// @return		void
// Sample usage:        Get_acc_data();
// @note		无
//-------------------------------------------------------------------------------------------------------------------
void Get_acc_data(void)
{
	uint8 i;
	int32 acc_sum[3]={0};

	for(i=0;i<AccFilterNum;i++)
	{
		get_icm20602_accdata_spi();
		acc_sum[0]  += icm_acc_z;
		acc_sum[1]  += icm_acc_y;
		acc_sum[2]  -= icm_acc_x;
		systick_delay_us(1);
	}
	
	Acc_x=(float)acc_sum[0]/3343.67347f;//转换关系:acc/(2^16/2)*8g = acc_sum/AccFilterNum/(2^16/2)*8g, g取9.8
	Acc_y=(float)acc_sum[1]/3343.67347f;
	Acc_z=(float)acc_sum[2]/3343.67347f;
}



//-------------------------------------------------------------------------------------------------------------------
// @brief		一种快速计算1/sqrt(x)的函数
// @return		float
// Sample usage:        invSqrt(x);
// @note		无
//-------------------------------------------------------------------------------------------------------------------
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}




//-------------------------------------------------------------------------------------------------------------------
// @brief		四元数姿态解算
// @return		void
// Sample usage:        IMU_AHRSupdate_noMagnetic(Gyro_x, Gyro_y, Gyro_z, Acc_x, Acc_y, Acc_z);
// @note		具体原理参见   https://blog.csdn.net/zhangyufeikk/article/details/94391858
//-------------------------------------------------------------------------------------------------------------------
void IMU_AHRSupdate_noMagnetic(float gx, float gy, float gz, float ax, float ay, float az)
{
    float halfT = 0.5 * delta_T;
    float vx, vy, vz;    //当前的机体坐标系上的重力单位向量
    float ex, ey, ez;    //四元数计算值与加速度计测量值的误差
    float exdev,eydev,ezdev;
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    float delta_2 = 0;
    
     //对加速度数据进行归一化 得到单位加速度
    float norm = invSqrt(ax*ax + ay*ay + az*az);       
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;
    
    vx = 2*(q1q3 - q0q2);
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;
    
    //用叉乘误差来做PI修正陀螺零偏，
    //通过调节 param_Kp，param_Ki 两个参数，
    //可以控制加速度计修正陀螺仪积分姿态的速度。
    I_ex += delta_T * (ex+last_ex)/2;   // integral error scaled by Ki
    I_ey += delta_T * (ey+last_ey)/2;
    I_ez += delta_T * (ez+last_ez)/2;
    
    
    
    
    exdev=ex-last_ex;
    eydev=ey-last_ey;
    ezdev=ez-last_ez;
    // adjusted gyroscope measuremen
    
    gx = gx+ param_Kp*ex + param_Ki*I_ex+ param_Kd*exdev;
    gy = gy+ param_Kp*ey + param_Ki*I_ey+ param_Kd*eydev;
    gz = gz+ param_Kp*ez + param_Ki*I_ez+ param_Kd*ezdev;
    
    ggx=gx;ggy=gy;ggz=gz;
    
    
    
    last_ex=ex;
    last_ey=ey;
    last_ez=ez;
    
    //四元数微分方程，其中halfT为测量周期的1/2，gx gy gz为陀螺仪角速度，以下都是已知量，这里使用了一阶龙哥库塔求解四元数微分方程
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + ( q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + ( q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + ( q0*gz + q1*gy - q2*gx)*halfT;
 
    
    
    
    
    
    
//    delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);
    // 整合四元数率    四元数微分方程  四元数更新算法，二阶毕卡法
//    q0 = (1-delta_2/8)*q0 + (-q1*gx - q2*gy - q3*gz)*halfT;			
//    q1 = (1-delta_2/8)*q1 + (q0*gx + q2*gz - q3*gy)*halfT;
//    q2 = (1-delta_2/8)*q2 + (q0*gy - q1*gz + q3*gx)*halfT;
//    q3 = (1-delta_2/8)*q3 + (q0*gz + q1*gy - q2*gx)*halfT;	
 
    // normalise quaternion
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    Q_info.q0 = q0 * norm;
    Q_info.q1 = q1 * norm;
    Q_info.q2 = q2 * norm;
    Q_info.q3 = q3 * norm;
}





//-------------------------------------------------------------------------------------------------------------------
// @brief		将得到的四元数转换为欧拉角
// @return		void
// Sample usage:        IMU_quaterToEulerianAngles();
// @note		具体原理参见   https://blog.csdn.net/zhangyufeikk/article/details/94391858
//-------------------------------------------------------------------------------------------------------------------
void IMU_quaterToEulerianAngles()

{
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
    eulerAngle.pitch = asin(-2*q1*q3 + 2*q0*q2) * 180.0/M_PI; // pitch
    eulerAngle.roll = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1) * 180.0/M_PI; // roll
    eulerAngle.yaw = atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1) * 180.0/M_PI; // yaw
 
/*   可以不用作姿态限度的限制
    if(eulerAngle.roll>90 || eulerAngle.roll<-90)
    {
        if(eulerAngle.pitch > 0)
        {
            eulerAngle.pitch = 180-eulerAngle.pitch;
        }
        if(eulerAngle.pitch < 0)
        {
            eulerAngle.pitch = -(180+eulerAngle.pitch);
        }
    }
    if(eulerAngle.yaw > 180)
    {
        eulerAngle.yaw -=360;
    }
    else if(eulerAngle.yaw <-180)
    {
        eulerAngle.yaw +=360;
    } 
    */
}


//-------------------------------------------------------------------------------------------------------------------
// @brief		解算角度函数
// @return		void
// Sample usage:        IMU_zs(icm_gyro_x, icm_gyro_y, icm_gyro_z, icm_acc_x, icm_acc_y, icm_acc_z);
// @note		此函数使用的是原始数据，即加速度计和陀螺仪直接测出的原始数据。
//-------------------------------------------------------------------------------------------------------------------
float angle_acc_pitch=0;
float angle_acc_yaw  =0;//z轴的偏航角
float angle_acc_roll=0;

void IMU_zs(float gx, float gy, float gz, float ax, float ay, float az)
{
  float angle_error;
  
  angle_acc_pitch=atan(ax*invSqrt(pow(ay,2)+pow(az,2)))*180.0/M_PI;
  //angle_acc_yaw=atan(az*invSqrt(pow(az,2)+pow(ax,2)))*180.0/M_PI;
  angle_acc_yaw=atan(ay*invSqrt(pow(ax,2)+pow(az,2)))*180.0/M_PI;
  
  //angle_acc_yaw=atan(1/(invSqrt(pow(ay,2)+pow(ax,2))*az))*180.0/M_PI;
  //ggx=gx*0.061035156;//roll
  ggy=gy*0.061035156;//pitch
  ggz=gz*0.061035156;//yaw
  
  
  
  eulerAngle.pitch=angle_calc_pitch(angle_acc_pitch,ggy);
  //eulerAngle.roll=angle_calc_pitch(angle_acc_roll,ggx);
  eulerAngle.yaw=angle_calc_yaw(angle_acc_yaw,ggz);
}


//-------------------------------------------------------------------------------------------------------------------
// @brief		解算角度函数
// @return		void
// Sample usage:        angle_calc_pitch(angle_acc_pitch,ggy);
// @note		此函数使用的是原始数据，即加速度计和陀螺仪直接测出的原始数据。
//-------------------------------------------------------------------------------------------------------------------
float angle_calc_pitch(float angle_m, float gyro_m)
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

    gyro_now = gyro_m * gyro_ratio;
    //根据测量到的加速度值转换为角度之后与上次的角度值求偏差
    error_angle = (angle_m - last_angle0)*acc_ratio;
    //根据偏差与陀螺仪测量得到的角度值计算当前角度值
    temp_angle = last_angle0 + (error_angle + gyro_now)*dt;
    //保存当前角度值
    last_angle0 = temp_angle;


    return temp_angle;

}


float angle_calc_yaw(float angle_m, float gyro_m)
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

    gyro_now = gyro_m * gyro_ratio;
    //根据测量到的加速度值转换为角度之后与上次的角度值求偏差
    error_angle = (angle_m - last_angle0)*acc_ratio;
    //根据偏差与陀螺仪测量得到的角度值计算当前角度值
    temp_angle = last_angle0 + (error_angle + gyro_now)*dt;
    //保存当前角度值
    last_angle0 = temp_angle;


    return temp_angle;

}
//-------------------------------------------------------------------------------------------------------------------
// @brief		一阶互补滤波函数
// @return		void
// Sample usage:        Complementary_Filter_1();
// @note		暂时无用
//-------------------------------------------------------------------------------------------------------------------
void Complementary_Filter_1(){
  eulerAngle.pitch = K1 * eulerAngle.pitch + (1-K1) * (Last_Angle + Gyro_x * delta_T);
  Last_Angle = eulerAngle.pitch;
}



//-------------------------------------------------------------------------------------------------------------------
// @brief		二阶互补滤波函数
// @return		void
// Sample usage:        Complementary_Filter_2();
// @note		太慢了，暂时无用
//-------------------------------------------------------------------------------------------------------------------
void Complementary_Filter_2(){
  x1 = (eulerAngle.pitch - Last_Angle) * (1-K2) * (1-K2);
  y1 = y1 + x1 * delta_T;
  x2 = y1 + 2*(1 - K2) * (eulerAngle.pitch - Last_Angle) + Gyro_x;
  Last_Angle = Last_Angle + x2 * delta_T;
  eulerAngle.pitch = Last_Angle;
}


//-------------------------------------------------------------------------------------------------------------------
// @brief		总体调用函数
// @return		void
// Sample usage:        IMUread();
// @note		获取Pitch Yaw Roll等只使用本函数即可（参量在上位机输出）
//-------------------------------------------------------------------------------------------------------------------
void IMUread(){
  Get_gyro_data();
  Get_acc_data();
  //IMU_AHRSupdate_noMagnetic(Gyro_x, Gyro_y, Gyro_z, Acc_x, Acc_y, Acc_z);
  //IMU_quaterToEulerianAngles();
  //Complementary_Filter_1();
  IMU_zs(icm_gyro_x, icm_gyro_y, icm_gyro_z, icm_acc_x, icm_acc_y, icm_acc_z);
}
