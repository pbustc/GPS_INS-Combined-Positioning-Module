#include "./kalman/kalman.h"


/**
  ******************************************************************************
  * @file    void Kalman_Filter_Init(KalmanCountData * Kalman_Struct)
  * @author  �ô�����Դ������
  * @version V0.1
  * @date    Feb-2020
  * @brief   �������˲������м�����ʼ��
  *	
  *
  ******************************************************************************
  * @attention
  *
  * 
  * 
  *
  ******************************************************************************
  */

void Kalman_Filter_Init(KalmanCountData * Kalman_Struct)
{
	Kalman_Struct -> Angle_err		 = 0;
	Kalman_Struct -> Q_bias			 = 0;
	Kalman_Struct -> PCt_0			 = 0;
	Kalman_Struct -> PCt_1			 = 0;
	Kalman_Struct -> E				 = 0;
	Kalman_Struct -> K_0			 = 0;
	Kalman_Struct -> K_1			 = 0;
	Kalman_Struct -> t_0			 = 0;
	Kalman_Struct -> t_1			 = 0;
	Kalman_Struct -> Pdot[0]		 = 0;
	Kalman_Struct -> Pdot[1]		 = 0;
	Kalman_Struct -> Pdot[2]		 = 0;
	Kalman_Struct -> Pdot[3]		 = 0;	      //Э�������΢�־���
	Kalman_Struct -> PP[0][0]		 = 1;
	Kalman_Struct -> PP[0][1]		 = 0;
	Kalman_Struct -> PP[1][0]		 = 0;
	Kalman_Struct -> PP[1][1]		 = 1;	      //Э��������ʼ��ΪPP[2][2] = { { 1, 0 },{ 0, 1 } };ע�ⲻ��Ϊ0
	Kalman_Struct -> Angle_Final	 = 0;
	Kalman_Struct -> Gyro_Final		 = 0;

}


/**
  ******************************************************************************
  * @file    void Kalman_Filter(float Accel,	float Gyro ,KalmanCountData * Kalman_Struct)
  *	
  *
  ******************************************************************************
  * @attention
  *		Accel:���ٶȼ����ݴ��������ĽǶ�ֵ
  *		Gyro :���������ݴ��������Ľ��ٶ�ֵ
  *		Kalman_Struct:������������Ҫ���м���������û�����Ϊȫ�ֽṹ�����
  *		Kalman_Struct -> Angle_Final  Ϊ�˲���Ƕ�����ֵ
  *		Kalman_Struct -> Gyro_Final   Ϊ������ٶ�ֵ
  ******************************************************************************
  */

void Kalman_Filter(float Accel,	float Gyro ,KalmanCountData * Kalman_Struct)
{
		//�����ǻ��ֽǶȣ�������ƣ�
		Kalman_Struct -> Angle_Final += (Gyro - Kalman_Struct -> Q_bias) * dt; 
		
		//����������Э�����΢��
		Kalman_Struct -> Pdot[0] = Q_angle - Kalman_Struct -> PP[0][1] - Kalman_Struct -> PP[1][0]; 
		Kalman_Struct -> Pdot[1] = - Kalman_Struct -> PP[1][1];
		Kalman_Struct -> Pdot[2] = - Kalman_Struct -> PP[1][1];
		Kalman_Struct -> Pdot[3] = Q_gyro;
		
		//����������Э����Ļ���
		Kalman_Struct -> PP[0][0] += Kalman_Struct -> Pdot[0] * dt;   
		Kalman_Struct -> PP[0][1] += Kalman_Struct -> Pdot[1] * dt;   
		Kalman_Struct -> PP[1][0] += Kalman_Struct -> Pdot[2] * dt;
		Kalman_Struct -> PP[1][1] += Kalman_Struct -> Pdot[3] * dt;
			
		//�������������   
		Kalman_Struct -> PCt_0 = C_0 * Kalman_Struct -> PP[0][0];
		Kalman_Struct -> PCt_1 = C_0 * Kalman_Struct -> PP[1][0];
		
		Kalman_Struct -> E = R_angle + C_0 * Kalman_Struct -> PCt_0;
		
		Kalman_Struct -> K_0 = Kalman_Struct -> PCt_0 / Kalman_Struct -> E;
		Kalman_Struct -> K_1 = Kalman_Struct -> PCt_1 / Kalman_Struct -> E;
		
		//�����п�������������������������ֵ
		//����Ƕ�ƫ��    
		Kalman_Struct -> Angle_err = Accel - Kalman_Struct -> Angle_Final;		
		Kalman_Struct -> Angle_Final += Kalman_Struct -> K_0 * Kalman_Struct -> Angle_err;	 //����������ŽǶ�ֵ
		Kalman_Struct -> Q_bias	+= Kalman_Struct -> K_1 * Kalman_Struct -> Angle_err;		 //�������Ź���ֵ��ƫ��
		Kalman_Struct -> Gyro_Final   = Gyro - Kalman_Struct -> Q_bias;						 //�������Ž��ٶ�ֵ
		
		//����������Э�������
		Kalman_Struct -> t_0 = Kalman_Struct -> PCt_0;
		Kalman_Struct -> t_1 = C_0 * Kalman_Struct -> PP[0][1];

		Kalman_Struct -> PP[0][0] -= Kalman_Struct -> K_0 * Kalman_Struct -> t_0;		 
		Kalman_Struct -> PP[0][1] -= Kalman_Struct -> K_0 * Kalman_Struct -> t_1;
		Kalman_Struct -> PP[1][0] -= Kalman_Struct -> K_1 * Kalman_Struct -> t_0;
		Kalman_Struct -> PP[1][1] -= Kalman_Struct -> K_1 * Kalman_Struct -> t_1;

}


