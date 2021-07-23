/**
  ******************************************************************************
  * @file    main.c
  * @author  fire
  * @version V1.0
  * @date    2016-xx-xx
  * @brief   ��GPSģ�鴫������ݽ��н��룬��ȡ��λ��Ϣ��
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:Ұ��  STM32 F4 ������
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :http://fire-stm32.taobao.com
  *
  ******************************************************************************
  */
  
#include "stm32f4xx.h"
#include "./led/bsp_led.h"
#include "./usart/bsp_debug_usart.h"
#include "./gps/gps_config.h"
#include "./adis/bsp_spi_adis.h" 
#include "./exit/bsp_exti.h"
#include "./systick/bsp_SysTick.h"
#include "./kalman/kalman.h"
#include "./attitude_solution/attitude_solution.h"

void Delay(__IO u32 nCount); 

void Delay_us(u32 time)
{
  u32 i=8*time;
  while(i--);
}
/*���������*/
uint32_t Task_Delay[NumOfTask]={0};

extern void nmea_decode_test(void);
//struct angle euler={0,0,0};
int new_data;
double gps_lon;
double gps_lat;
int new_gps_data;
struct angle euler={0,0,0};
struct origin_data gyro_data={0,0,0};
struct origin_data accl_data={0,0,0};
int new_data;
/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
int main(void)
{
	float Acel[3] = {0};
	float Gyro[3] = {0};
	/* LED �˿ڳ�ʼ�� */
	LED_GPIO_Config();	 
  
  LED_BLUE;
  
	/*���ڳ�ʼ��*/
	Debug_USART_Config();
  
  GPS_Config();
	
	SysTick_Init();	
	
	EXTI_Config();
	ADIS16465_Init();	
/*
  printf("\r\nҰ�� GPSģ���������\r\n"); 
  
	printf("\r\n�������GPSģ�鴮�ڴ��ص����ݽ��룬");
	printf("ʵ��ʱ������������GPSģ�� \r\n"); 
*/

  /* GPS������� */
 
  new_gps_data=0;
	
  while(1)
	{  if(new_data)
				{
			
			nmea_decode_test();
			/*printf("gps�½�����Ϣ");
			printf("\r\nγ�ȣ�%f,����%f\r\n",gps_lat,gps_lon);*/
			//AHRS_update();
			IMU_update();
		  //printf("zƫ����=%3.5f\n",euler.yaw);
		
			//printf("IMU��Ϣ��");
			//printf("\nX_GYRO=%3.8f,Y_GYRO=%3.8f,Z_GYRO=%3.8f",gyro_data.x,gyro_data.y,gyro_data.z);
			//printf("\nX_ACCL=%3.8f,Y_ACCL=%3.8f,Z_ACCL=%3.8f",accl_data.x,accl_data.y,accl_data.z);		
				
			//printf("\ny������=%3.5f, x�����=%3.5f��zƫ����=%3.5f\n",euler.pitch,euler.roll,euler.yaw);
		 
					printf("$,%3.5f,%3.5f,%3.5f,%3.5f,%3.5f,%3.5f,%3.5f,%3.5f,#", 
					gps_lat,gps_lon,gyro_data.x,gyro_data.y,gyro_data.z,accl_data.x,accl_data.y,accl_data.z
				);
				
			/*
					ADIS16465ReadAcc(Acel);		
			ADIS16465ReadGyro(Gyro);
			printf("$,%3.5f,%3.5f,%3.5f,%3.5f,%3.5f,%3.5f,%3.5f,%3.5f,#", 
					gps_lat,gps_lon,Gyro[0],Gyro[1],Gyro[2],Acel[0],Acel[1],Acel[2]
				);		*/
			new_data = 0;				
			}
	}

}

void Delay(__IO uint32_t nCount)	 //�򵥵���ʱ����
{
	for(; nCount != 0; nCount--);
}
/*********************************************END OF FILE**********************/

