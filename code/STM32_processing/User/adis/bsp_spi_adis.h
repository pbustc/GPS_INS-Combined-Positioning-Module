#ifndef __SPI_ADIS_H
#define __SPI_ADIS_H

#include "stm32f4xx.h"
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
#define  sADIS_ID                       0X4051     //ADIS16465

/* Private define ------------------------------------------------------------*/
/**************寄存器定义-开头**************/
#define DIAG_STAT	      	  		  0x0200
#define X_GYRO_LOW	      	  		0x0400
#define X_GYRO_OUT	      	  		0x0600
#define Y_GYRO_LOW	      	  		0x0800
#define Y_GYRO_OUT	      	  		0x0A00
#define Z_GYRO_LOW	      	  		0x0C00
#define Z_GYRO_OUT	      	  		0x0E00
#define X_ACCL_LOW	      	  		0x1000
#define X_ACCL_OUT	      	  		0x1200
#define Y_ACCL_LOW	      	  		0x1400
#define Y_ACCL_OUT	      	  		0x1600
#define Z_ACCL_LOW	      	 		  0x1800
#define Z_ACCL_OUT	      	  		0x1A00
#define TEMP_OUT	      	  	 	  0x1C00
#define TIME_STAMP	      	  		0x1E00
#define DATA_CNTR	      	  		  0x2200
#define X_DELTANG_LOW	      		  0x2400
#define X_DELTANG_OUT	      	    0x2600
#define Y_DELTANG_LOW	      	    0x2800
#define Y_DELTANG_OUT  	      	  0x2A00
#define Z_DELTANG_LOW	      	    0x2C00
#define Z_DELTANG_OUT	      	    0x2E00
#define X_DELTVEL_LOW	      	    0x3000
#define X_DELTVEL_OUT	      	    0x3200
#define Y_DELTVEL_LOW 	      	  0x3400
#define Y_DELTVEL_OUT	      	    0x3600
#define Z_DELTVEL_LOW	      	    0x3800
#define Z_DELTVEL_OUT	      	    0x3A00
#define XG_BIAS_LOW	      	  		0x4000
#define XG_BIAS_HIGH	      	  	0x4200
#define YG_BIAS_LOW	      	  		0x4400
#define YG_BIAS_HIGH	      	  	0x4600
#define ZG_BIAS_LOW	      	  		0x4800
#define ZG_BIAS_HIGH	      	  	0x4A00
#define XA_BIAS_LOW	      	  		0x4C00
#define XA_BIAS_HIGH	      	  	0x4E00
#define YA_BIAS_LOW	      	  		0x5000
#define YA_BIAS_HIGH	      	  	0x5200
#define ZA_BIAS_LOW	      	  		0x5400
#define ZA_BIAS_HIGH	      	    0x5600
#define FILT_CTRL	      	  		  0x5C00
#define RANG_MDL	      	  		  0x5E00
#define MSC_CTRL	  	      	    0x6000
#define UP_SCALE	      	  		  0x6200
#define DEC_RATE	      	  		  0x6400
#define NULL_CNFG	      	  		  0x6600
#define GLOB_CMD	      	  		  0x6800
#define FIRM_REV	      	  		  0x6C00
#define FIRM_DM	      	  			  0x6E00
#define FIRM_Y		  	      	    0x7000
#define PROD_ID	      				    0x7200
#define SERIAL_NUM      		  	  0x7400
#define USER_SCR_1	      		    0x7600
#define USER_SCR_2	      		    0x7800
#define USER_SCR_3	  	      	  0x7A00
#define FLSHCNT_LOW		      	    0x7C00
#define FLSHCNT_HIGH  		      	0x7E00

#define BURST_READ        			  0x6800
#define Dummy_Byte                0x0FFF


/*******Global Commands (GLOB_CMD)命令***********/
#define SOFTWARE_RESET            	 		0x0080 
#define FLASH_MEMORY_TEST            		0x0010 
#define FLASH_MEMORY_UPDATE             0x0008 
#define SENSOR_SELF_TEST            		0x0004 
#define FACTORY_CALIBRATION_RESTORE     0x0002       //工厂校准恢复
#define BIAS_CORRECTION_UPDATE          0x0001       //偏差纠正更新


/*******Scale Factor***********/
#define KG           (1 / (160 * 65536))      //Gyroscope Scale Factor 单位：°/sec，算出来的十进制乘它即可
#define AG           (0.25 / (65536 * 1000))   //Accelerometer Scale Factor  单位：g，算出来的十进制乘它即可
#define DAG          (360 / 2147483648)        // Delta Angle Scale Factor  单位：°，算出来的十进制乘它即可
#define DVG          (100 / 2147483648)        // Delta Velocity Scale Factor  单位：m/sec，算出来的十进制乘它即可

/* 如果不使用BrustRead模式，注释掉BURST_READ_FREQUENCE即可 */
//#define BURST_READ_FREQUENCE      SPI_BaudRatePrescaler_128
#define NOMAL_READ_FREQUENCE      SPI_BaudRatePrescaler_64
#define PI 3.1415

/*SPI接口定义-开头****************************/
#define ADIS_SPI                           SPI1
#define ADIS_SPI_CLK                       RCC_APB2Periph_SPI1
#define ADIS_SPI_CLK_INIT                  RCC_APB2PeriphClockCmd

#define ADIS_SPI_SCK_PIN                   GPIO_Pin_3                  
#define ADIS_SPI_SCK_GPIO_PORT             GPIOB                       
#define ADIS_SPI_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOB
#define ADIS_SPI_SCK_PINSOURCE             GPIO_PinSource3
#define ADIS_SPI_SCK_AF                    GPIO_AF_SPI1

#define ADIS_SPI_MISO_PIN                  GPIO_Pin_4                
#define ADIS_SPI_MISO_GPIO_PORT            GPIOB                   
#define ADIS_SPI_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define ADIS_SPI_MISO_PINSOURCE            GPIO_PinSource4
#define ADIS_SPI_MISO_AF                   GPIO_AF_SPI1

#define ADIS_SPI_MOSI_PIN                  GPIO_Pin_5                
#define ADIS_SPI_MOSI_GPIO_PORT            GPIOB                     
#define ADIS_SPI_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOB
#define ADIS_SPI_MOSI_PINSOURCE            GPIO_PinSource5
#define ADIS_SPI_MOSI_AF                   GPIO_AF_SPI1

#define ADIS_CS_PIN                        GPIO_Pin_6               
#define ADIS_CS_GPIO_PORT                  GPIOC                     
#define ADIS_CS_GPIO_CLK                   RCC_AHB1Periph_GPIOC

#define SPI_ADIS_CS_LOW()      {ADIS_CS_GPIO_PORT->BSRRH=ADIS_CS_PIN;}
#define SPI_ADIS_CS_HIGH()     {ADIS_CS_GPIO_PORT->BSRRL=ADIS_CS_PIN;}

//#define SPI_ADIS_CS_LOW()      GPIO_ResetBits(ADIS_CS_GPIO_PORT,ADIS_CS_PIN);
//#define SPI_ADIS_CS_HIGH()     GPIO_SetBits(ADIS_CS_GPIO_PORT,ADIS_CS_PIN);
/*SPI接口定义-结尾****************************/

/*等待超时时间*/
#define SPIT_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define SPIT_LONG_TIMEOUT         ((uint32_t)(10 * SPIT_FLAG_TIMEOUT))

/*信息输出*/
#define ADIS_ERROR(fmt,arg...)          printf("\n<<-ADIS-ERROR->> "fmt"\n",##arg)
#define ADIS_INFO(fmt,arg...)           printf("\n<<-ADIS-INFO->> "fmt"\n",##arg)

#define DEBUG_ADIS(format,...)  printf("File:"__FILE__", Line:%d: \n"format"",__LINE__,##__VA_ARGS__) 

void SPI_ADIS_Init(void);
u16 SPI_ADIS_SendHalfWord(u16 HalfWord);
uint8_t SPI_ADIS_ReadID(void);
u16 SPI_ADIS_ReadSerialNum(void);
void ADIS_WriteReg(u16 reg_add,u16 reg_dat);
u16 ADIS_ReadReg(u16 reg_add);
void BrustRead(void);
uint8_t ADIS_Error_Flag_Indicators(u16 reg_dat);
void Global_Commands(u16 reg_dat);
void ADIS16465ReadAcc(float *accData);
void ADIS16465ReadGyro(float *gyroData);
void ADIS16465ReadTemp(float *tempData);
void ADIS_ReadAccData(u16 *buf);
void ADIS_ReadGyroData(u16 *buf);
void ADIS16465ReadAngle(float *angleData);
void ADIS16465ReadVelocity(float *velocityData);
void ADIS_ReadDeltaAngleData(u16 *buf);
void ADIS_ReadDeltaVelocityData(u16 *buf);
//void TwosComplementCal(float *buf);
void ADIS16465TransData(float *Data, int var);
void ADIS_ReadAccBiasData(u16 *buf);
void ADIS_ReadGyroBiasData(u16 *buf);
void accel2angle(float *accData,float *angleData);
void ADIS16465_Init(void);

#endif /* __SPI_ADIS_H */







