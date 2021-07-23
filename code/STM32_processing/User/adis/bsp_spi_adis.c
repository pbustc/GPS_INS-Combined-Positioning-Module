/**
 ******************************************************************************
 * @file    bsp_spi_adis.c
 * @author  fire
 * @version V1.0
 * @date    2015-xx-xx
 * @brief   spi adis 底层应用函数bsp
 ******************************************************************************
 * @attention
 *
 * 实验平台:野火STM32 F407 开发板
 * 论坛    :http://www.firebbs.cn
 * 淘宝    :https://fire-stm32.taobao.com
 *
 ******************************************************************************
 */

#include "./adis/bsp_spi_adis.h"
#include "./usart/bsp_debug_usart.h"
#include "./systick/bsp_SysTick.h"
#include <math.h>


static __IO uint32_t  SPITimeout = SPIT_LONG_TIMEOUT;

static uint16_t SPI_TIMEOUT_UserCallback(uint8_t errorCode);

extern void Delay_us(u32 time);

/**
 * @brief  SPI_ADIS初始化
 * @param  无
 * @retval 无
 */
void SPI_ADIS_Init(void)
{
    SPI_InitTypeDef  SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* 使能 ADIS_SPI 及GPIO 时钟 */
    /*!< SPI_ADIS_SPI_CS_GPIO, SPI_ADIS_SPI_MOSI_GPIO,
         SPI_ADIS_SPI_MISO_GPIO,SPI_ADIS_SPI_SCK_GPIO 时钟使能 */
    RCC_AHB1PeriphClockCmd (ADIS_SPI_SCK_GPIO_CLK | ADIS_SPI_MISO_GPIO_CLK|ADIS_SPI_MOSI_GPIO_CLK|ADIS_CS_GPIO_CLK, ENABLE);

    /*!< SPI_ADIS_SPI 时钟使能 */
    ADIS_SPI_CLK_INIT(ADIS_SPI_CLK, ENABLE);

    //设置引脚复用
    GPIO_PinAFConfig(ADIS_SPI_SCK_GPIO_PORT,ADIS_SPI_SCK_PINSOURCE,ADIS_SPI_SCK_AF);
    GPIO_PinAFConfig(ADIS_SPI_MISO_GPIO_PORT,ADIS_SPI_MISO_PINSOURCE,ADIS_SPI_MISO_AF);
    GPIO_PinAFConfig(ADIS_SPI_MOSI_GPIO_PORT,ADIS_SPI_MOSI_PINSOURCE,ADIS_SPI_MOSI_AF);

    /*!< 配置 SPI_ADIS_SPI 引脚: SCK */
    GPIO_InitStructure.GPIO_Pin = ADIS_SPI_SCK_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_Init(ADIS_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

    /*!< 配置 SPI_ADIS_SPI 引脚: MISO */
    GPIO_InitStructure.GPIO_Pin = ADIS_SPI_MISO_PIN;
    GPIO_Init(ADIS_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

    /*!< 配置 SPI_ADIS_SPI 引脚: MOSI */
    GPIO_InitStructure.GPIO_Pin = ADIS_SPI_MOSI_PIN;
    GPIO_Init(ADIS_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

    /*!< 配置 SPI_ADIS_SPI 引脚: CS */
    GPIO_InitStructure.GPIO_Pin = ADIS_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(ADIS_CS_GPIO_PORT, &GPIO_InitStructure);

    /* 停止信号 ADIS: CS引脚高电平*/
    SPI_ADIS_CS_HIGH();

    /* ADIS_SPI 模式配置 */
    // ADIS芯片 支持SPI模式0及模式3，据此设置CPOL CPHA
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
		/*SPI通信一次发送16b*/
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
		
#ifdef BURST_READ_FREQUENCE
    SPI_InitStructure.SPI_BaudRatePrescaler = BURST_READ_FREQUENCE;
#else
    SPI_InitStructure.SPI_BaudRatePrescaler = NOMAL_READ_FREQUENCE;
#endif
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(ADIS_SPI, &SPI_InitStructure);

    /* 使能 ADIS_SPI  */
    SPI_Cmd(ADIS_SPI, ENABLE);

}

/*******************************************************************************
* Function Name  : SPI_ADIS_SendHalfWord
* Description    : Sends a Half Word through the SPI interface and return the
*                  Half Word received from the SPI bus.
* Input          : Half Word : Half Word to send.
* Output         : None
* Return         : The value of the received Half Word.
*******************************************************************************/
u16 SPI_ADIS_SendHalfWord(u16 HalfWord)
{

    SPITimeout = SPIT_FLAG_TIMEOUT;

    /* Loop while DR register in not emplty */
	  /*等待TXE为高电平时才能写入数据*/
    /*RESET == 0 低电平*/
		while (SPI_I2S_GetFlagStatus(ADIS_SPI, SPI_I2S_FLAG_TXE) == RESET)
    {
        if((SPITimeout--) == 0) return SPI_TIMEOUT_UserCallback(2);
    }

    /*通过SPI的接口发送16b数据 */
    SPI_I2S_SendData(ADIS_SPI, HalfWord);

    SPITimeout = SPIT_FLAG_TIMEOUT;

    /* Wait to receive a Half Word */
		/*等待RXNE变为高电平即为写入完毕*/
    while (SPI_I2S_GetFlagStatus(ADIS_SPI, SPI_I2S_FLAG_RXNE) == RESET)
    {
        if((SPITimeout--) == 0) return SPI_TIMEOUT_UserCallback(3);
    }
    /* Return the Half Word read from the SPI bus */
		/*ReceiveData为接*/
    return SPI_I2S_ReceiveData(ADIS_SPI);
}

/**
  * @brief   读取ADIS ID 作为一个STM32与spi外设的通信函数
  * @param
  * @retval  正常返回1，异常返回0
  */
uint8_t SPI_ADIS_ReadID(void)
{
    u16 Re = 0;
    Re = ADIS_ReadReg(PROD_ID);    //读器件地址
    if(Re == 0x4051)
    {
        ADIS_ERROR("ADIS dectected error!\r\n检测不到ADIS16465模块，请检查模块与开发板的接线\r\n");
        return 0;
    }
    else
    {
				ADIS_INFO("The firmware revision is 0x%X\r",ADIS_ReadReg(FIRM_REV));  
				ADIS_INFO("The serial num is 0x%X\r",ADIS_ReadReg(SERIAL_NUM));
        ADIS_INFO("ADIS ID = 0x%X\n",Re);
        return 1;
    }

}

/**
 * @brief  读取ADIS Serial Num
 * @param 	无
 * @retval ADIS Serial Num
 */
u16 SPI_ADIS_ReadSerialNum(void)
{
    u16 Temp = 0;

    Delay_us(50);
    /* Select the ADIS: Chip Select low */
    SPI_ADIS_CS_LOW();
    Delay_us(50);

    /* Send "RDID " instruction */
    SPI_ADIS_SendHalfWord(SERIAL_NUM);
    SPI_ADIS_SendHalfWord(SERIAL_NUM);
    Temp = SPI_ADIS_SendHalfWord(Dummy_Byte);

    /* Deselect the ADIS: Chip Select high */
    SPI_ADIS_CS_HIGH();
    Delay_us(50);

    return Temp;
}

/**
 * @brief   从ADIS reg 读取数据，只能单次读取单个寄存器
 * @param   reg_add:寄存器地址
 * @retval  返回读取到的值
 */
u16 ADIS_ReadReg(u16 reg_add)
{
    u16 Temp = 0;

    Delay_us(50);
    /* Select the ADIS: Chip Select low */
    SPI_ADIS_CS_LOW();
    Delay_us(50);

    /* Send "RDID " instruction */
    SPI_ADIS_SendHalfWord(reg_add);
    SPI_ADIS_SendHalfWord(reg_add);
    Temp = SPI_ADIS_SendHalfWord(Dummy_Byte);

    /* Deselect the ADIS: Chip Select high */
    SPI_ADIS_CS_HIGH();
    Delay_us(50);

    return Temp;
}

/**
 * @brief   向ADIS reg 写入数据
 * @param   reg_add:寄存器地址
* @param	 reg_data:要写入的数据
 * @retval  无
 */
void ADIS_WriteReg(u16 reg_add,u16 reg_dat)
{
    u16 Temp1,Temp2;
    Temp1 = 0x8000 | reg_add | ( reg_dat & 0x00ff );  //低位
    Temp2 = 0x8000 | ( reg_add + 0x0100 ) | (( reg_dat & 0xff00 ) >> 8 );  //高位

    Delay_us(50);
    /* Select the ADIS: Chip Select low */
    SPI_ADIS_CS_LOW();
    Delay_us(50);

    /* Send "RDID " instruction */
    SPI_ADIS_SendHalfWord(Temp1);
    SPI_ADIS_SendHalfWord(Temp2);
    /* Deselect the ADIS: Chip Select high */
    SPI_ADIS_CS_HIGH();
    Delay_us(50);
}

/**
  * @brief   检测各个寄存器是否正常
  * @param   reg_dat：Global Commands中的命令
  * @retval  正常返回1，异常返回0
  */
uint8_t ADIS_Error_Flag_Indicators(u16 reg_dat)
{
    u16 Re = 0;
    Delay_us(50);
    Re = ADIS_ReadReg(DIAG_STAT);    //读器件地址
    if(Re != 0x0000)
    {
        ADIS_ERROR("ADIS error:\r\n");
        switch(Re)
        {
        case 0x0002:
            printf("\tDatapath overrun!Please initiate a reset using the RST pin.\r");
            break;
        case 0x0004:
            printf("\tFlash memory update failure!Please ensure that VDD ≥ 3 V and repeat the update attempt.\r");
            break;
        case 0x0008:
            printf("\tSPI communication error!Please repeat the previous communication sequence.\r");
            break;
        case 0x0010:
            printf("\tStandby mode!When VDD ≥ 2.8 V for 250 ms, the ADIS16465 reinitializes and starts producing data again.\r");
            break;
        case 0x0020:
            printf("\tSensor failure!Please repeat the same test.\r");
            break;
        case 0x0040:
            printf("\tMemory failure!Please repeat the same test.\r");
            break;
        case 0x0080:
            printf("\tClock error!Please adjust the frequency of the clock signal on the SYNC pin to operate within the appropriate range.\r");
            break;
        default:
            printf("\tThere were multiple register errors!\r");
            break;
        }
        return 0;
    }
    else
    {
        switch(reg_dat)
        {
        case 0x0080:
            ADIS_INFO("Software reset succeed!\r");
            break;
        case 0x0010:
            ADIS_INFO("Flash memory test succeed!\r");
            break;
        case 0x0008:
            ADIS_INFO("Flash memory update succeed!\r");
            break;
        case 0x0004:
            ADIS_INFO("Sensor self test succceed!\r");
            break;
        case 0x0002:
            ADIS_INFO("Factory calibration restore succceed!\r");
            break;
        case 0x0001:
            ADIS_INFO("Bias correction update succceed!\r");
            break;
        default:
            ADIS_INFO("Operation is completed!\r");
            break;
        }
        return 1;
    }
}

/**
 * @brief   Global Commands
 * @param   reg_dat要执行的命令，已宏定义
 * @retval  无
 */
void Global_Commands(u16 reg_dat)
{
    Delay_us( 200 );
    if(reg_dat == SOFTWARE_RESET)
    {
        Delay_ms( 250 );
    }
		/**/
    ADIS_WriteReg( GLOB_CMD, reg_dat );
    if(reg_dat == SOFTWARE_RESET)
    {
        Delay_ms( 250 );
    }
    Delay_us( 200 );
    ADIS_Error_Flag_Indicators(reg_dat);
}

/**
 * @brief   读取加速度计各个分量数据
 * @param   buf:各个加速度分量数组
 * @retval  无
 */
void ADIS_ReadAccData(u16 *buf)
{
    buf[0] = ADIS_ReadReg(X_ACCL_LOW);
    Delay_us(50);
    buf[1] = ADIS_ReadReg(X_ACCL_OUT);
    Delay_us(50);
    buf[2] = ADIS_ReadReg(Y_ACCL_LOW);
    Delay_us(50);
    buf[3] = ADIS_ReadReg(Y_ACCL_OUT);
    Delay_us(50);
    buf[4] = ADIS_ReadReg(Z_ACCL_LOW);
    Delay_us(50);
    buf[5] = ADIS_ReadReg(Z_ACCL_OUT);
    Delay_us(50);
}

/**
 * @brief   读取陀螺仪各个分量数据
 * @param   buf:各个陀螺仪分量数组
 * @retval  无
 */
void ADIS_ReadGyroData(u16 *buf)
{
    buf[0] = ADIS_ReadReg(X_GYRO_LOW);
    Delay_us(50);
    buf[1] = ADIS_ReadReg(X_GYRO_OUT);
    Delay_us(50);
    buf[2] = ADIS_ReadReg(Y_GYRO_LOW);
    Delay_us(50);
    buf[3] = ADIS_ReadReg(Y_GYRO_OUT);
    Delay_us(50);
    buf[4] = ADIS_ReadReg(Z_GYRO_LOW);
    Delay_us(50);
    buf[5] = ADIS_ReadReg(Z_GYRO_OUT);
    Delay_us(50);
}


/**
 * @brief   读取加速度计各个bias分量数据
 * @param   buf:各个加速度分量数组
 * @retval  无
 */
void ADIS_ReadAccBiasData(u16 *buf)
{
    buf[0] = ADIS_ReadReg(XA_BIAS_LOW);
    Delay_us(50);
    buf[1] = ADIS_ReadReg(XA_BIAS_HIGH);
    Delay_us(50);
    buf[2] = ADIS_ReadReg(YA_BIAS_LOW);
    Delay_us(50);
    buf[3] = ADIS_ReadReg(YA_BIAS_HIGH);
    Delay_us(50);
    buf[4] = ADIS_ReadReg(ZA_BIAS_LOW);
    Delay_us(50);
    buf[5] = ADIS_ReadReg(ZA_BIAS_HIGH);
    Delay_us(50);
}

/**
 * @brief   读取陀螺仪各个bias分量数据
 * @param   buf:各个陀螺仪分量数组
 * @retval  无
 */
void ADIS_ReadGyroBiasData(u16 *buf)
{
    buf[0] = ADIS_ReadReg(XG_BIAS_LOW );
    Delay_us(50);
    buf[1] = ADIS_ReadReg(XG_BIAS_HIGH);
    Delay_us(50);
    buf[2] = ADIS_ReadReg(YG_BIAS_LOW);
    Delay_us(50);
    buf[3] = ADIS_ReadReg(YG_BIAS_HIGH);
    Delay_us(50);
    buf[4] = ADIS_ReadReg(ZG_BIAS_LOW);
    Delay_us(50);
    buf[5] = ADIS_ReadReg(ZG_BIAS_HIGH);
    Delay_us(50);
}

/**
 * @brief   读取陀螺仪各个角位移分量数据
 * @param   buf:各个陀螺仪角位移分量数组
 * @retval  无
 */
void ADIS_ReadDeltaAngleData(u16 *buf)
{
    buf[0] = ADIS_ReadReg(X_DELTANG_LOW);
    Delay_us(50);
    buf[1] = ADIS_ReadReg(X_DELTANG_OUT);
    Delay_us(50);
    buf[2] = ADIS_ReadReg(Y_DELTANG_LOW);
    Delay_us(50);
    buf[3] = ADIS_ReadReg(Y_DELTANG_OUT);
    Delay_us(50);
    buf[4] = ADIS_ReadReg(Z_DELTANG_LOW);
    Delay_us(50);
    buf[5] = ADIS_ReadReg(Z_DELTANG_OUT);
    Delay_us(50);
}

/**
 * @brief   读取加速度计各个速度分量数据
 * @param   buf:各个加速度计速度分量数组
 * @retval  无
 */
void ADIS_ReadDeltaVelocityData(u16 *buf)
{
    buf[0] = ADIS_ReadReg(X_DELTVEL_LOW);
    Delay_us(50);
    buf[1] = ADIS_ReadReg(X_DELTVEL_OUT);
    Delay_us(50);
    buf[2] = ADIS_ReadReg(Y_DELTVEL_LOW);
    Delay_us(50);
    buf[3] = ADIS_ReadReg(Y_DELTVEL_OUT);
    Delay_us(50);
    buf[4] = ADIS_ReadReg(Z_DELTVEL_LOW);
    Delay_us(50);
    buf[5] = ADIS_ReadReg(Z_DELTVEL_OUT);
    Delay_us(50);
}

/**
 * @brief   读取ADIS的加速度数据
 * @param   加速度数组
 * @retval  无
 */
void ADIS16465ReadAcc(float *accData)
{
    u16 buf[6] = {0};
		/*Accelerate*/
    ADIS_ReadAccData(buf);
		/*通过移位，将高低16位组合*/
    accData[0] = (float)((buf[1] << 16) | buf[0]);
    accData[1] = (float)((buf[3] << 16) | buf[2]);
    accData[2] = (float)((buf[5] << 16) | buf[4]);
		//加速度校准
		ADIS_ReadAccBiasData(buf);
    accData[0] = accData[0] + (float)((buf[1] << 16) | buf[0]);
    accData[1] = accData[1] + (float)((buf[3] << 16) | buf[2]);
    accData[2] = accData[2] + (float)((buf[5] << 16) | buf[4]);
		
		ADIS16465TransData(accData,0);
		//printf("\nX_ACCL=%3.8f,Y_ACCL=%3.8f,Z_ACCL=%3.8f",accData[0],accData[1],accData[2]);	
}

/**
 * @brief   读取ADIS的角速度数据
 * @param   角速度数组
 * @retval  无
 */
void ADIS16465ReadGyro(float *gyroData)
{
    u16 buf[6] = {0};
    ADIS_ReadGyroData(buf);
    gyroData[0] = (float)((buf[1] << 16) | buf[0]);
    gyroData[1] = (float)((buf[3] << 16) | buf[2]);
    gyroData[2] = (float)((buf[5] << 16) | buf[4]);
		//加上偏差
    ADIS_ReadGyroBiasData(buf);
    gyroData[0] = gyroData[0] + (float)((buf[1] << 16) | buf[0]);
    gyroData[1] = gyroData[1] + (float)((buf[3] << 16) | buf[2]);
    gyroData[2] = gyroData[2] + (float)((buf[5] << 16) | buf[4]);
		ADIS16465TransData(gyroData,1);
		//printf("\nX_GYRO=%3.8f,Y_GYRO=%3.8f,Z_GYRO=%3.8f",gyroData[0],gyroData[1],gyroData[2]);
}

/**
 * @brief   读取ADIS的角位移数据
 * @param   角位移数组
 * @retval  无
 */
void ADIS16465ReadAngle(float *angleData)
{
    u16 buf[6] = {0};
    ADIS_ReadDeltaAngleData(buf);
    angleData[0] = (float)((buf[1] << 16) | buf[0]);
    angleData[1] = (float)((buf[3] << 16) | buf[2]);
    angleData[2] = (float)((buf[5] << 16) | buf[4]);
		ADIS16465TransData(angleData,2);
}

/**
 * @brief   读取ADIS的速度数据
 * @param   速度数组
 * @retval  无
 */
void ADIS16465ReadVelocity(float *velocityData)
{
    u16 buf[6] = {0};
    ADIS_ReadDeltaVelocityData(buf);
    velocityData[0] = (float)((buf[1] << 16) | buf[0]);
    velocityData[1] = (float)((buf[3] << 16) | buf[2]);
    velocityData[2] = (float)((buf[5] << 16) | buf[4]);
		ADIS16465TransData(velocityData,3);
}

/**
 * @brief   将ADIS各个寄存器的数据乘上scale factor
 * @param   数组
 * @retval  无
 */
void ADIS16465TransData(float *Data, int var)
{
	int i;
	for(i = 0; i<3; i++)
	{
		switch(var)
		{
			case 0:Data[i] = ( Data[i] * 0.25 ) / (65536 * 1000);break;
			case 1:Data[i] = Data[i] / (160 * 65536);break;
			case 2:Data[i] = ( Data[i] * 360 ) / 2147483648;break;
			case 3:Data[i] = ( Data[i] * 100 ) / 2147483648;break;
		}
	}		
}


/**
 * @brief   将加速度转化为角度 公式： x轴角度=(180/pi)*atan(accelx/sqrt(accely^2+accelz^2))
* @param   accData:加速度数组   angleData:最后得到的角度数组
 * @retval  无
 */
void accel2angle(float *accData,float *angleData)
{
    float buf[3] = {0};  //存放中间数据
		float a,b,c;
		
		a = (accData[0] * accData[0]);
		b = (accData[1] * accData[1]);
		c = (accData[2] * accData[2]);
		
		buf[0] = sqrt(b+c);
		buf[1] = sqrt(a+c);
		buf[2] = sqrt(a+b);
				
		buf[0] = atan2(accData[0] , buf[0]);
		buf[1] = atan2(accData[1] , buf[1]);
		buf[2] = atan2(accData[2] , buf[2]);
		
		angleData[0] = 180 * buf[0] / PI;
		angleData[1] = 180 * buf[1] / PI;
		angleData[2] = 180 * buf[2] / PI;
}

/**
  * @brief   读取ADIS的原始温度数据
  * @param
  * @retval
  */
void ADIS16465ReadTemp(float *tempData)
{
    float temperature = 0;
    temperature = 0.1 * ADIS_ReadReg(TEMP_OUT);
    *tempData = temperature;
}


/**
 * @brief   BurstRead模式,使用该函数前请设置SPI_BaudRatePrescaler
 * @param   无
* @param	 无
 * @retval  无
 */
void BrustRead(void)
{
    u16 Reg[10] = {0};  //存放10个数组读取出来的值
    u16  checksum = 0;  //计算出来的最后一个值
    u8 i;

    /* Select the ADIS: Chip Select low */
    SPI_ADIS_CS_LOW();
    Delay_us(20);
    SPI_ADIS_SendHalfWord(BURST_READ);
    SPI_ADIS_SendHalfWord(BURST_READ);
    Reg[0] = SPI_ADIS_SendHalfWord(Dummy_Byte);  //发送命令并读取第一个寄存器的值
    /*读取后9个寄存器的值*/
    for(i=1; i<10; i++)
    {
        Reg[i] = SPI_ADIS_SendHalfWord(Dummy_Byte);
    }
    Delay_us(20);
    SPI_ADIS_CS_HIGH();
    /* 比较接收的最后一个数据和算出来的数据是否一样，若不一样，则通信出错 */
    for(i=0; i<9; i++)
    {
        checksum += (( Reg[i] >> 8) + ( Reg[i] & 0x00ff ));  //计算最后一个值
    }
    if( checksum == Reg[9] )
    {
        ADIS_INFO("通信正常！");
        ADIS_INFO("checksum = 0x%X", checksum);
        for(i=0; i<10; i++)
        {
            ADIS_INFO("Reg[%d] = 0x%X", i, Reg[i]);
        }
    }
    else
    {
        ADIS_ERROR("通信错误！");
        ADIS_INFO("checksum = 0x%X", checksum);
        for(i=0; i<10; i++)
        {
            ADIS_INFO("Reg[%d] = 0x%X", i, Reg[i]);
        }
    }
}
void ADIS16465_Init(void)
{
	SPI_ADIS_Init();
	//软件复位
	Global_Commands(SOFTWARE_RESET);
	Delay_ms( 500 );
	ADIS_WriteReg(RANG_MDL,0x0007);   //规定陀螺仪测量范围
	ADIS_WriteReg(NULL_CNFG,0x3F0A);     //启动加速度计偏差校准
	Global_Commands(BIAS_CORRECTION_UPDATE);  //更新偏差校准
	
	ADIS_WriteReg(DEC_RATE,0x0009);  //配置采集速率为200Hz,即5ms读取一次	
	ADIS_INFO("ADIS initialized successfully!");
}
/**
  * @brief  等待超时回调函数
  * @param  None.
  * @retval None.
  */
static  uint16_t SPI_TIMEOUT_UserCallback(uint8_t errorCode)
{
    /* 等待超时后的处理,输出错误信息 */
    ADIS_ERROR("SPI 等待超时!errorCode = %d",errorCode);
    return 0;
}

/*********************************************END OF FILE**********************/
