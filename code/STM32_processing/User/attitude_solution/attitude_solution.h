#ifndef __ATTITUDE_SOLUTION_H
#define   __ATTITUDE_SOLUTION_H

#include "stm32f4xx.h"

#define Ki 0.0f
#define Kp 35.0f
#define dtt 0.005f   //频率倒数，即积分周期

//定义角度结构体
struct angle 
{
	double pitch,roll,yaw;
};
struct origin_data
{
	double x,y,z;
};
void AHRS_update(void);
void IMU_update(void);










#endif   /*__ATTITUDE_SOLUTION_H*/
