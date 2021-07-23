#include "./attitude_solution/attitude_solution.h"
#include "./adis/bsp_spi_adis.h" 
#include <math.h>

/* 该文件利用四元数进行姿态解算*/
//注：x轴朝向为航行器正前方，绕x轴旋转为横滚角(roll)，y轴为俯仰角（pitch），z轴为偏航角（yaw）
//注：
//全局变量
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; 
extern struct angle euler;
extern struct origin_data gyro_data;
extern struct origin_data accl_data;
//累计误差
float integral_e[3]={0};

/**
 * @brief   该函数用来对一个数进行（1/开方）运算
 * @param   要开方的数
 * @retval  开方后的倒数
 */
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/*将IMU的信息直接更新
* 陀螺仪和加速度计的原始数据
*/
void IMU_update(void) 
{
	float Acel[3] = {0,0,0};
	float Gyro[3] = {0,0,0};
	ADIS16465ReadAcc(Acel);
	ADIS16465ReadGyro(Gyro);
	
	gyro_data.x = Gyro[0];
	gyro_data.y = Gyro[1];
	gyro_data.z = Gyro[2];
	
	accl_data.x = Acel[0];
	accl_data.y = Acel[1];
	accl_data.z = Acel[2];
}


/**
 * @brief   该函数用来姿态解算
 * @param   void
 * @retval  void
 */
void AHRS_update(void) 
{
	//定义四元数,属于中间变量
	float q0_temp, q1_temp, q2_temp, q3_temp;  
	//定义加速度和角速度数组
	float Acel[3] = {0,0,0};
	float Gyro[3] = {0,0,0};
	//定义开方并取倒数后得变量
	float recipNorm;
	//物体坐标系下重力分量
	float Vb[3];
	//姿态误差
	float e[3];
	//定义姿态角中间变量
	float g[5];
	
	//读取加速度和陀螺仪的值
	/*此处的加速度与角速度均经过了校准*/
	ADIS16465ReadAcc(Acel);
	ADIS16465ReadGyro(Gyro);
	/**/
	euler.yaw = euler.yaw + ( Gyro[2] - 0.03050 ) * dtt;
	if(euler.yaw>180.0f || euler.yaw<-180.0f)
	{
		euler.yaw = -euler.yaw;
	}
	
	//加速度在-0.005到0.005之间算0
	if(!((-0.0005f < Acel[0] < 0.0005f) && (-0.0005f < Acel[1] < 0.0005f) && (-0.0005f < Acel[2] < 0.0005f)))
	{
			//将角度转化为弧度
			Gyro[0] = Gyro[0] * 0.0174f;
			Gyro[1] = Gyro[1] * 0.0174f;
			Gyro[2] = Gyro[2] * 0.0174f;
					
			//对加速度进行归一化
			recipNorm = invSqrt(Acel[0]*Acel[0] + Acel[1]*Acel[1] + Acel[2]*Acel[2]);
			
			Acel[0] = Acel[0] * recipNorm;
			Acel[1] = Acel[1] * recipNorm;
			Acel[2] = Acel[2] * recipNorm;
			
			//提取物体坐标系下重力分量
			Vb[0] = 2 * (q1 * q3 - q0 * q2);
			Vb[1] = 2 * (q0 * q1 + q2 * q3);
			Vb[2] = 2 * (q0 * q0 + q3 * q3 - 0.5f);
			
			//求取姿态误差(陀螺仪)
			e[0] = (Acel[1] * Vb[2] - Acel[2] * Vb[1]);
			e[1] = (Acel[2] * Vb[0] - Acel[0] * Vb[2]);
			e[2] = (Acel[0] * Vb[1] - Acel[1] * Vb[0]);
			
			
			if(Ki > 0.0f)
			{
				integral_e[0] += Ki * e[0] * dtt;
				integral_e[1] += Ki * e[1] * dtt;
				integral_e[2] += Ki * e[2] * dtt;
				
				Gyro[0] += integral_e[0];
				Gyro[1] += integral_e[1];
				Gyro[2] += integral_e[2];
			}
			else
			{
				integral_e[0] = 0.0f;
				integral_e[1] = 0.0f;
				integral_e[2] = 0.0f;
			}
			
			//对误差进行比例运算，比例的结果累加到陀螺仪的数据中，用于修正陀螺仪数据，比例系数是Kp。
			Gyro[0] += Kp * e[0];
			Gyro[1] += Kp * e[1];
			Gyro[2] += Kp * e[2];	
	}
	
	//解四元数微分方程
	Gyro[0] *= 0.5f * dtt;
	Gyro[1] *= 0.5f * dtt;
	Gyro[2] *= 0.5f * dtt;
		
	q0_temp = q0;
	q1_temp = q1;
	q2_temp = q2;
	q3_temp = q3;
		
	q0 = q0 + (-q1_temp * Gyro[0] - q2_temp * Gyro[1] - q3_temp * Gyro[2]);
	q1 = q1 + ( q0_temp * Gyro[0] + q2_temp * Gyro[2] - q3_temp * Gyro[1]);
	q2 = q2 + ( q0_temp * Gyro[1] - q1_temp * Gyro[2] + q3_temp * Gyro[0]);
	q3 = q3 + ( q0_temp * Gyro[2] + q1_temp * Gyro[1] - q2_temp * Gyro[0]);
	
	//四元数归一化
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	
	//计算姿态角
	g[0] = 2.0f * ( q1 * q3 - q0 * q2 );
	g[1] = 2.0f * ( q1 * q0 + q3 * q2 );
	g[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
//	g[3] = 2.0f * ( q1 * q2 + q0 * q3 );
//	g[4] = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
	
	euler.pitch = asinf(g[0]) * 57.3f;    //1弧度 = 57.3°
	euler.roll  = atan2f(g[1] , g[2]) * 57.3f;
	//euler.yaw   = atan2f(g[3] , g[4]) * 57.3f;
		
}

/*使用PI控制器进行滤波，准确地说事消除漂移误差，只要存在误差控制器便会持续作用，
	直至误差为0。控制的效果取决于P和I参数，分别对应比例控制和积分控制的参数。 	
	e~(w) = kp * e(w) + ki * Σ(e(w) * dt)      !!!很重要，这个是互补滤波
*/
/*飞行器角度补偿参数K决定了这两个惯性环节的时间常数，K越小，姿态角度⑥跟踪
	输出越慢，虽能有效抑制加速度计（或电子罗盘）上的噪声，但陀螺仪的输出误差
	随之增大。K越大，姿态角度⑥跟踪8输出越快，虽能抑制陀螺仪的累积误差，但加速
	度计（或电子罗盘）上的噪声也随之增大。可见，合理选择补偿参数K的值是互补滤波
	算法中的关键。在实际应用中，通过观察互补滤波器处理后的姿态角的变化趋势，按照
	上述规律在线调整K值大小，直到滤波处理后的姿态角能准确地跟踪上四旋翼飞行器倾角的变化。
*/
/*比例增益kp控制低频段测量和高频段测量之间的截止频率，积分增益ki，控
	制滤波器消除误差的时间。截止频率较低时，滤波结果主要依赖于陀螺的测量；
	截止频率较高时，滤波结果主要依赖于加速度计的测量。通常，kp和k的选择与截
	止频率w和阻尼系数了有关：
*/

//!!!Kp控制的是相信加速度计还是相信陀螺仪
//!!!Ki控制的是滤波器消除误差的时间
//过阻尼：Ki=Kp^2/4
//欠阻尼：Ki=Kp^2/2

//即使使用了两者，也只可以用于测得飞机的俯仰和横滚角度。对于偏航角度，由于偏航
//角和重力方向正交，无法用加速度计测量得到，因此还需要采用其他设备来校准测量偏航
//角度的陀螺仪的漂移值。校准的设备可以使用磁罗盘计（电子磁罗盘，对磁场变化和惯性力敏感）或者GPS。

