/**
 ******************************************************************************
 * @file    bsp_spi_adis.c
 * @author  fire
 * @version V1.0
 * @date    2015-xx-xx
 * @brief   spi adis �ײ�Ӧ�ú���bsp
 ******************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:Ұ��STM32 F407 ������
 * ��̳    :http://www.firebbs.cn
 * �Ա�    :https://fire-stm32.taobao.com
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
 * @brief  SPI_ADIS��ʼ��
 * @param  ��
 * @retval ��
 */
void SPI_ADIS_Init(void)
{
    SPI_InitTypeDef  SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    /* ʹ�� ADIS_SPI ��GPIO ʱ�� */
    /*!< SPI_ADIS_SPI_CS_GPIO, SPI_ADIS_SPI_MOSI_GPIO,
         SPI_ADIS_SPI_MISO_GPIO,SPI_ADIS_SPI_SCK_GPIO ʱ��ʹ�� */
    RCC_AHB1PeriphClockCmd (ADIS_SPI_SCK_GPIO_CLK | ADIS_SPI_MISO_GPIO_CLK|ADIS_SPI_MOSI_GPIO_CLK|ADIS_CS_GPIO_CLK, ENABLE);

    /*!< SPI_ADIS_SPI ʱ��ʹ�� */
    ADIS_SPI_CLK_INIT(ADIS_SPI_CLK, ENABLE);

    //�������Ÿ���
    GPIO_PinAFConfig(ADIS_SPI_SCK_GPIO_PORT,ADIS_SPI_SCK_PINSOURCE,ADIS_SPI_SCK_AF);
    GPIO_PinAFConfig(ADIS_SPI_MISO_GPIO_PORT,ADIS_SPI_MISO_PINSOURCE,ADIS_SPI_MISO_AF);
    GPIO_PinAFConfig(ADIS_SPI_MOSI_GPIO_PORT,ADIS_SPI_MOSI_PINSOURCE,ADIS_SPI_MOSI_AF);

    /*!< ���� SPI_ADIS_SPI ����: SCK */
    GPIO_InitStructure.GPIO_Pin = ADIS_SPI_SCK_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_Init(ADIS_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

    /*!< ���� SPI_ADIS_SPI ����: MISO */
    GPIO_InitStructure.GPIO_Pin = ADIS_SPI_MISO_PIN;
    GPIO_Init(ADIS_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

    /*!< ���� SPI_ADIS_SPI ����: MOSI */
    GPIO_InitStructure.GPIO_Pin = ADIS_SPI_MOSI_PIN;
    GPIO_Init(ADIS_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

    /*!< ���� SPI_ADIS_SPI ����: CS */
    GPIO_InitStructure.GPIO_Pin = ADIS_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(ADIS_CS_GPIO_PORT, &GPIO_InitStructure);

    /* ֹͣ�ź� ADIS: CS���Ÿߵ�ƽ*/
    SPI_ADIS_CS_HIGH();

    /* ADIS_SPI ģʽ���� */
    // ADISоƬ ֧��SPIģʽ0��ģʽ3���ݴ�����CPOL CPHA
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
		/*SPIͨ��һ�η���16b*/
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

    /* ʹ�� ADIS_SPI  */
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
	  /*�ȴ�TXEΪ�ߵ�ƽʱ����д������*/
    /*RESET == 0 �͵�ƽ*/
		while (SPI_I2S_GetFlagStatus(ADIS_SPI, SPI_I2S_FLAG_TXE) == RESET)
    {
        if((SPITimeout--) == 0) return SPI_TIMEOUT_UserCallback(2);
    }

    /*ͨ��SPI�Ľӿڷ���16b���� */
    SPI_I2S_SendData(ADIS_SPI, HalfWord);

    SPITimeout = SPIT_FLAG_TIMEOUT;

    /* Wait to receive a Half Word */
		/*�ȴ�RXNE��Ϊ�ߵ�ƽ��Ϊд�����*/
    while (SPI_I2S_GetFlagStatus(ADIS_SPI, SPI_I2S_FLAG_RXNE) == RESET)
    {
        if((SPITimeout--) == 0) return SPI_TIMEOUT_UserCallback(3);
    }
    /* Return the Half Word read from the SPI bus */
		/*ReceiveDataΪ��*/
    return SPI_I2S_ReceiveData(ADIS_SPI);
}

/**
  * @brief   ��ȡADIS ID ��Ϊһ��STM32��spi�����ͨ�ź���
  * @param
  * @retval  ��������1���쳣����0
  */
uint8_t SPI_ADIS_ReadID(void)
{
    u16 Re = 0;
    Re = ADIS_ReadReg(PROD_ID);    //��������ַ
    if(Re == 0x4051)
    {
        ADIS_ERROR("ADIS dectected error!\r\n��ⲻ��ADIS16465ģ�飬����ģ���뿪����Ľ���\r\n");
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
 * @brief  ��ȡADIS Serial Num
 * @param 	��
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
 * @brief   ��ADIS reg ��ȡ���ݣ�ֻ�ܵ��ζ�ȡ�����Ĵ���
 * @param   reg_add:�Ĵ�����ַ
 * @retval  ���ض�ȡ����ֵ
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
 * @brief   ��ADIS reg д������
 * @param   reg_add:�Ĵ�����ַ
* @param	 reg_data:Ҫд�������
 * @retval  ��
 */
void ADIS_WriteReg(u16 reg_add,u16 reg_dat)
{
    u16 Temp1,Temp2;
    Temp1 = 0x8000 | reg_add | ( reg_dat & 0x00ff );  //��λ
    Temp2 = 0x8000 | ( reg_add + 0x0100 ) | (( reg_dat & 0xff00 ) >> 8 );  //��λ

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
  * @brief   �������Ĵ����Ƿ�����
  * @param   reg_dat��Global Commands�е�����
  * @retval  ��������1���쳣����0
  */
uint8_t ADIS_Error_Flag_Indicators(u16 reg_dat)
{
    u16 Re = 0;
    Delay_us(50);
    Re = ADIS_ReadReg(DIAG_STAT);    //��������ַ
    if(Re != 0x0000)
    {
        ADIS_ERROR("ADIS error:\r\n");
        switch(Re)
        {
        case 0x0002:
            printf("\tDatapath overrun!Please initiate a reset using the RST pin.\r");
            break;
        case 0x0004:
            printf("\tFlash memory update failure!Please ensure that VDD �� 3 V and repeat the update attempt.\r");
            break;
        case 0x0008:
            printf("\tSPI communication error!Please repeat the previous communication sequence.\r");
            break;
        case 0x0010:
            printf("\tStandby mode!When VDD �� 2.8 V for 250 ms, the ADIS16465 reinitializes and starts producing data again.\r");
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
 * @param   reg_datҪִ�е�����Ѻ궨��
 * @retval  ��
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
 * @brief   ��ȡ���ٶȼƸ�����������
 * @param   buf:�������ٶȷ�������
 * @retval  ��
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
 * @brief   ��ȡ�����Ǹ�����������
 * @param   buf:���������Ƿ�������
 * @retval  ��
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
 * @brief   ��ȡ���ٶȼƸ���bias��������
 * @param   buf:�������ٶȷ�������
 * @retval  ��
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
 * @brief   ��ȡ�����Ǹ���bias��������
 * @param   buf:���������Ƿ�������
 * @retval  ��
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
 * @brief   ��ȡ�����Ǹ�����λ�Ʒ�������
 * @param   buf:���������ǽ�λ�Ʒ�������
 * @retval  ��
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
 * @brief   ��ȡ���ٶȼƸ����ٶȷ�������
 * @param   buf:�������ٶȼ��ٶȷ�������
 * @retval  ��
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
 * @brief   ��ȡADIS�ļ��ٶ�����
 * @param   ���ٶ�����
 * @retval  ��
 */
void ADIS16465ReadAcc(float *accData)
{
    u16 buf[6] = {0};
		/*Accelerate*/
    ADIS_ReadAccData(buf);
		/*ͨ����λ�����ߵ�16λ���*/
    accData[0] = (float)((buf[1] << 16) | buf[0]);
    accData[1] = (float)((buf[3] << 16) | buf[2]);
    accData[2] = (float)((buf[5] << 16) | buf[4]);
		//���ٶ�У׼
		ADIS_ReadAccBiasData(buf);
    accData[0] = accData[0] + (float)((buf[1] << 16) | buf[0]);
    accData[1] = accData[1] + (float)((buf[3] << 16) | buf[2]);
    accData[2] = accData[2] + (float)((buf[5] << 16) | buf[4]);
		
		ADIS16465TransData(accData,0);
		//printf("\nX_ACCL=%3.8f,Y_ACCL=%3.8f,Z_ACCL=%3.8f",accData[0],accData[1],accData[2]);	
}

/**
 * @brief   ��ȡADIS�Ľ��ٶ�����
 * @param   ���ٶ�����
 * @retval  ��
 */
void ADIS16465ReadGyro(float *gyroData)
{
    u16 buf[6] = {0};
    ADIS_ReadGyroData(buf);
    gyroData[0] = (float)((buf[1] << 16) | buf[0]);
    gyroData[1] = (float)((buf[3] << 16) | buf[2]);
    gyroData[2] = (float)((buf[5] << 16) | buf[4]);
		//����ƫ��
    ADIS_ReadGyroBiasData(buf);
    gyroData[0] = gyroData[0] + (float)((buf[1] << 16) | buf[0]);
    gyroData[1] = gyroData[1] + (float)((buf[3] << 16) | buf[2]);
    gyroData[2] = gyroData[2] + (float)((buf[5] << 16) | buf[4]);
		ADIS16465TransData(gyroData,1);
		//printf("\nX_GYRO=%3.8f,Y_GYRO=%3.8f,Z_GYRO=%3.8f",gyroData[0],gyroData[1],gyroData[2]);
}

/**
 * @brief   ��ȡADIS�Ľ�λ������
 * @param   ��λ������
 * @retval  ��
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
 * @brief   ��ȡADIS���ٶ�����
 * @param   �ٶ�����
 * @retval  ��
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
 * @brief   ��ADIS�����Ĵ��������ݳ���scale factor
 * @param   ����
 * @retval  ��
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
 * @brief   �����ٶ�ת��Ϊ�Ƕ� ��ʽ�� x��Ƕ�=(180/pi)*atan(accelx/sqrt(accely^2+accelz^2))
* @param   accData:���ٶ�����   angleData:���õ��ĽǶ�����
 * @retval  ��
 */
void accel2angle(float *accData,float *angleData)
{
    float buf[3] = {0};  //����м�����
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
  * @brief   ��ȡADIS��ԭʼ�¶�����
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
 * @brief   BurstReadģʽ,ʹ�øú���ǰ������SPI_BaudRatePrescaler
 * @param   ��
* @param	 ��
 * @retval  ��
 */
void BrustRead(void)
{
    u16 Reg[10] = {0};  //���10�������ȡ������ֵ
    u16  checksum = 0;  //������������һ��ֵ
    u8 i;

    /* Select the ADIS: Chip Select low */
    SPI_ADIS_CS_LOW();
    Delay_us(20);
    SPI_ADIS_SendHalfWord(BURST_READ);
    SPI_ADIS_SendHalfWord(BURST_READ);
    Reg[0] = SPI_ADIS_SendHalfWord(Dummy_Byte);  //���������ȡ��һ���Ĵ�����ֵ
    /*��ȡ��9���Ĵ�����ֵ*/
    for(i=1; i<10; i++)
    {
        Reg[i] = SPI_ADIS_SendHalfWord(Dummy_Byte);
    }
    Delay_us(20);
    SPI_ADIS_CS_HIGH();
    /* �ȽϽ��յ����һ�����ݺ�������������Ƿ�һ��������һ������ͨ�ų��� */
    for(i=0; i<9; i++)
    {
        checksum += (( Reg[i] >> 8) + ( Reg[i] & 0x00ff ));  //�������һ��ֵ
    }
    if( checksum == Reg[9] )
    {
        ADIS_INFO("ͨ��������");
        ADIS_INFO("checksum = 0x%X", checksum);
        for(i=0; i<10; i++)
        {
            ADIS_INFO("Reg[%d] = 0x%X", i, Reg[i]);
        }
    }
    else
    {
        ADIS_ERROR("ͨ�Ŵ���");
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
	//�����λ
	Global_Commands(SOFTWARE_RESET);
	Delay_ms( 500 );
	ADIS_WriteReg(RANG_MDL,0x0007);   //�涨�����ǲ�����Χ
	ADIS_WriteReg(NULL_CNFG,0x3F0A);     //�������ٶȼ�ƫ��У׼
	Global_Commands(BIAS_CORRECTION_UPDATE);  //����ƫ��У׼
	
	ADIS_WriteReg(DEC_RATE,0x0009);  //���òɼ�����Ϊ200Hz,��5ms��ȡһ��	
	ADIS_INFO("ADIS initialized successfully!");
}
/**
  * @brief  �ȴ���ʱ�ص�����
  * @param  None.
  * @retval None.
  */
static  uint16_t SPI_TIMEOUT_UserCallback(uint8_t errorCode)
{
    /* �ȴ���ʱ��Ĵ���,���������Ϣ */
    ADIS_ERROR("SPI �ȴ���ʱ!errorCode = %d",errorCode);
    return 0;
}

/*********************************************END OF FILE**********************/
