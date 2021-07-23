#include "./attitude_solution/attitude_solution.h"
#include "./adis/bsp_spi_adis.h" 
#include <math.h>

/* ���ļ�������Ԫ��������̬����*/
//ע��x�ᳯ��Ϊ��������ǰ������x����תΪ�����(roll)��y��Ϊ�����ǣ�pitch����z��Ϊƫ���ǣ�yaw��
//ע��
//ȫ�ֱ���
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; 
extern struct angle euler;
extern struct origin_data gyro_data;
extern struct origin_data accl_data;
//�ۼ����
float integral_e[3]={0};

/**
 * @brief   �ú���������һ�������У�1/����������
 * @param   Ҫ��������
 * @retval  ������ĵ���
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

/*��IMU����Ϣֱ�Ӹ���
* �����Ǻͼ��ٶȼƵ�ԭʼ����
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
 * @brief   �ú���������̬����
 * @param   void
 * @retval  void
 */
void AHRS_update(void) 
{
	//������Ԫ��,�����м����
	float q0_temp, q1_temp, q2_temp, q3_temp;  
	//������ٶȺͽ��ٶ�����
	float Acel[3] = {0,0,0};
	float Gyro[3] = {0,0,0};
	//���忪����ȡ������ñ���
	float recipNorm;
	//��������ϵ����������
	float Vb[3];
	//��̬���
	float e[3];
	//������̬���м����
	float g[5];
	
	//��ȡ���ٶȺ������ǵ�ֵ
	/*�˴��ļ��ٶ�����ٶȾ�������У׼*/
	ADIS16465ReadAcc(Acel);
	ADIS16465ReadGyro(Gyro);
	/**/
	euler.yaw = euler.yaw + ( Gyro[2] - 0.03050 ) * dtt;
	if(euler.yaw>180.0f || euler.yaw<-180.0f)
	{
		euler.yaw = -euler.yaw;
	}
	
	//���ٶ���-0.005��0.005֮����0
	if(!((-0.0005f < Acel[0] < 0.0005f) && (-0.0005f < Acel[1] < 0.0005f) && (-0.0005f < Acel[2] < 0.0005f)))
	{
			//���Ƕ�ת��Ϊ����
			Gyro[0] = Gyro[0] * 0.0174f;
			Gyro[1] = Gyro[1] * 0.0174f;
			Gyro[2] = Gyro[2] * 0.0174f;
					
			//�Լ��ٶȽ��й�һ��
			recipNorm = invSqrt(Acel[0]*Acel[0] + Acel[1]*Acel[1] + Acel[2]*Acel[2]);
			
			Acel[0] = Acel[0] * recipNorm;
			Acel[1] = Acel[1] * recipNorm;
			Acel[2] = Acel[2] * recipNorm;
			
			//��ȡ��������ϵ����������
			Vb[0] = 2 * (q1 * q3 - q0 * q2);
			Vb[1] = 2 * (q0 * q1 + q2 * q3);
			Vb[2] = 2 * (q0 * q0 + q3 * q3 - 0.5f);
			
			//��ȡ��̬���(������)
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
			
			//�������б������㣬�����Ľ���ۼӵ������ǵ������У������������������ݣ�����ϵ����Kp��
			Gyro[0] += Kp * e[0];
			Gyro[1] += Kp * e[1];
			Gyro[2] += Kp * e[2];	
	}
	
	//����Ԫ��΢�ַ���
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
	
	//��Ԫ����һ��
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	
	//������̬��
	g[0] = 2.0f * ( q1 * q3 - q0 * q2 );
	g[1] = 2.0f * ( q1 * q0 + q3 * q2 );
	g[2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
//	g[3] = 2.0f * ( q1 * q2 + q0 * q3 );
//	g[4] = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
	
	euler.pitch = asinf(g[0]) * 57.3f;    //1���� = 57.3��
	euler.roll  = atan2f(g[1] , g[2]) * 57.3f;
	//euler.yaw   = atan2f(g[3] , g[4]) * 57.3f;
		
}

/*ʹ��PI�����������˲���׼ȷ��˵������Ư����ֻҪ���������������������ã�
	ֱ�����Ϊ0�����Ƶ�Ч��ȡ����P��I�������ֱ��Ӧ�������ƺͻ��ֿ��ƵĲ����� 	
	e~(w) = kp * e(w) + ki * ��(e(w) * dt)      !!!����Ҫ������ǻ����˲�
*/
/*�������ǶȲ�������K���������������Ի��ڵ�ʱ�䳣����KԽС����̬�ǶȢ޸���
	���Խ����������Ч���Ƽ��ٶȼƣ���������̣��ϵ��������������ǵ�������
	��֮����KԽ����̬�ǶȢ޸���8���Խ�죬�������������ǵ��ۻ���������
	�ȼƣ���������̣��ϵ�����Ҳ��֮���󡣿ɼ�������ѡ�񲹳�����K��ֵ�ǻ����˲�
	�㷨�еĹؼ�����ʵ��Ӧ���У�ͨ���۲컥���˲�����������̬�ǵı仯���ƣ�����
	�����������ߵ���Kֵ��С��ֱ���˲���������̬����׼ȷ�ظ������������������ǵı仯��
*/
/*��������kp���Ƶ�Ƶ�β����͸�Ƶ�β���֮��Ľ�ֹƵ�ʣ���������ki����
	���˲�����������ʱ�䡣��ֹƵ�ʽϵ�ʱ���˲������Ҫ���������ݵĲ�����
	��ֹƵ�ʽϸ�ʱ���˲������Ҫ�����ڼ��ٶȼƵĲ�����ͨ����kp��k��ѡ�����
	ֹƵ��w������ϵ�����йأ�
*/

//!!!Kp���Ƶ������ż��ٶȼƻ�������������
//!!!Ki���Ƶ����˲�����������ʱ��
//�����᣺Ki=Kp^2/4
//Ƿ���᣺Ki=Kp^2/2

//��ʹʹ�������ߣ�Ҳֻ�������ڲ�÷ɻ��ĸ����ͺ���Ƕȡ�����ƫ���Ƕȣ�����ƫ��
//�Ǻ����������������޷��ü��ٶȼƲ����õ�����˻���Ҫ���������豸��У׼����ƫ��
//�Ƕȵ������ǵ�Ư��ֵ��У׼���豸����ʹ�ô����̼ƣ����Ӵ����̣��Դų��仯�͹��������У�����GPS��

