#include "imu.h"
#include "delay.h"

mpu_data_t  mpu_data;
imu_t      imu={0};
u8 mpu_buff[14];

void SPI5_Init(void)
{	 
  GPIO_InitTypeDef  GPIO_InitStructure;
  SPI_InitTypeDef  SPI_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI5, ENABLE);
	
  /**SPI5 GPIO Configuration    
    PF7     ------> SPI5_SCK
    PF9     ------> SPI5_MOSI
    PF8     ------> SPI5_MISO 
		PF6     ------> SPI5_NSS
  */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOF, &GPIO_InitStructure);//��ʼ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;//����
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOF, &GPIO_InitStructure);//��ʼ��
	
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource7,GPIO_AF_SPI5); //PB3����Ϊ SPI1
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource8,GPIO_AF_SPI5); //PB4����Ϊ SPI1
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource9,GPIO_AF_SPI5); //PB5����Ϊ SPI1
 
	//����ֻ���SPI�ڳ�ʼ��
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI5,ENABLE);//��λSPI5
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI5,DISABLE);//ֹͣ��λSPI5

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//����ͬ��ʱ�ӵĿ���״̬Ϊ�͵�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//����ͬ��ʱ�ӵĵ�һ�������أ��������½������ݱ�����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ128
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 10;	//CRCֵ����Ķ���ʽ
	SPI_Init(SPI5, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
	SPI_Cmd(SPI5, ENABLE); //ʹ��SPI����
	
}


//SPI5 ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
u8 SPI5_ReadWriteByte(u8 TxData)
{		 			  
  while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_TXE) == RESET){}//�ȴ���������  
	SPI_I2S_SendData(SPI5, TxData); //ͨ������SPIx����һ��byte  ����
  while (SPI_I2S_GetFlagStatus(SPI5, SPI_I2S_FLAG_RXNE) == RESET){} //�ȴ�������һ��byte  
	return SPI_I2S_ReceiveData(SPI5); //����ͨ��SPIx������յ�����	
}

//spi read a byte
u8 mpu_read_byte(u8 reg)
{
	u8 tmp=0;
  MPU_NSS_LOW;
	SPI5_ReadWriteByte(reg|0x80);//r ���λΪ1
	tmp=SPI5_ReadWriteByte(0xff);
  MPU_NSS_HIGH;
	return tmp;
}

//spi write a byte
u8 mpu_write_byte(u8 reg,u8 data) 				 
{ 
		MPU_NSS_LOW;
		SPI5_ReadWriteByte(reg&0x7F); //w ���λΪ0
		SPI5_ReadWriteByte(data);
		MPU_NSS_HIGH;
		return 0;
}
 
//ͬʱ������Ĵ���
//reg:��ʼ�Ĵ�����ַ
//len�����Ĵ������ܸ���
//*buf���洢�ڴ���ʼָ��
u8 mpu_read_bytes(u8 reg,u8 *buf,u8 len)
{ 
 	u8 tmp=0;
	while(len)
	{
		MPU_NSS_LOW;
		SPI5_ReadWriteByte(reg|0x80);//r ���λΪ1
		*buf=SPI5_ReadWriteByte(0x00);
		len--;
		buf++;
		reg++;
		MPU_NSS_HIGH;
		delay_us(5); //ÿ��һ���Ĵ��������ӳ�һ��ʱ�䡣�������϶���һ���Ĵ���--yulong
	}	
	return tmp;
}

/**
	* @brief  set imu 6500 gyroscope measure range
  * @param  fsr: range(0,��250dps;1,��500dps;2,��1000dps;3,��2000dps)
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t mpu_set_gyro_fsr1(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_GYRO_CONFIG, fsr << 3);
}


/**
	* @brief  set imu 6050/6500 accelerate measure range
  * @param  fsr: range(0,��2g;1,��4g;2,��8g;3,��16g)
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t mpu_set_accel_fsr1(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_ACCEL_CONFIG, fsr << 3); 
}

/**
	* @brief  get the offset data of MPU6500
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void mpu_offset_call(void)
{
	int i;
	for (i=0; i<500;i++)
	{
		mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

		mpu_data.ax_offset += mpu_buff[0] << 8 | mpu_buff[1];
		mpu_data.ay_offset += mpu_buff[2] << 8 | mpu_buff[3];
		mpu_data.az_offset += mpu_buff[4] << 8 | mpu_buff[5];
	
		mpu_data.gx_offset += mpu_buff[8]  << 8 | mpu_buff[9];
		mpu_data.gy_offset += mpu_buff[10] << 8 | mpu_buff[11];
		mpu_data.gz_offset += mpu_buff[12] << 8 | mpu_buff[13];
		
		delay_ms(5);
	}
//	mpu_data.ax_offset=mpu_data.ax_offset / 300;
//	mpu_data.ay_offset=mpu_data.ay_offset / 300;
//	mpu_data.az_offset=mpu_data.az_offset / 300;
	mpu_data.gx_offset=mpu_data.gx_offset * 0.002 ;// 300; 
	mpu_data.gy_offset=mpu_data.gy_offset * 0.002 ;// 300;
	mpu_data.gz_offset=mpu_data.gz_offset * 0.002 ;// 300;
}


void MPU_Init1(void)
{
		uint8_t i                        = 0;
		uint8_t MPU6500_Init_Data[7][2] = { { MPU6500_PWR_MGMT_1, 0x03 },     /* Clock Source - Gyro-Z */ 
																				{ MPU6500_PWR_MGMT_2, 0x00 },     /* Enable Acc & Gyro */ 
																				{ MPU6500_CONFIG, 0x05 },         /* LPF 41Hz */ 
																				{ MPU6500_GYRO_CONFIG, 0x18 },    /* +-2000dps */ 
																				{ MPU6500_ACCEL_CONFIG, 0x10 },   /* +-8G */ 
																				{ MPU6500_ACCEL_CONFIG_2, 0x02 }, /* enable LowPassFilter  Set Acc LPF */ 
																				{ MPU6500_USER_CTRL, 0x00 }};     /* Disable AUX */
				  
		SPI5_Init();
		delay_ms(100);
																				
		mpu_write_byte(MPU6500_PWR_MGMT_1, 0x80);  // Reset the internal registers
		delay_ms(100);
		
		mpu_write_byte(MPU6500_SIGNAL_PATH_RESET, 0x07);// Reset gyro/accel/temp digital signal path
		delay_ms(100);

		if (MPU6500_ID == mpu_read_byte(MPU6500_WHO_AM_I))
 		{																
			for (i = 0; i < 7; i++)
			{
				mpu_write_byte(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
				delay_ms(10);
			}
		}
		else
		{
			while(1){}
		}
		mpu_offset_call();
}

/**
	* @brief  get the data of imu
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void mpu_get_data(void)
{
		if(mpu_read_byte(MPU6500_INT_STATUS) == 0x01) //����ready
		{
			delay_us(5);
  		mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
   
			mpu_data.ax   = mpu_buff[0] << 8 | mpu_buff[1];
			mpu_data.ay   = mpu_buff[2] << 8 | mpu_buff[3];
			mpu_data.az   = mpu_buff[4] << 8 | mpu_buff[5];
			mpu_data.temp = mpu_buff[6] << 8 | mpu_buff[7];

			mpu_data.gx = ((mpu_buff[8]  << 8 | mpu_buff[9])  - mpu_data.gx_offset) ; 
			mpu_data.gy = ((mpu_buff[10] << 8 | mpu_buff[11]) - mpu_data.gy_offset) ;
			mpu_data.gz = ((mpu_buff[12] << 8 | mpu_buff[13]) - mpu_data.gz_offset) ;

			/* +-8g -> 8192LSB/g */
			imu.ax = mpu_data.ax / 4096.0f;   
			imu.ay = mpu_data.ay / 4096.0f;   
			imu.az = mpu_data.az / 4096.0f;   
			
      imu.temp = 21 + mpu_data.temp / 333.87f;
			
			/* 2000dps -> rad/s */
			imu.wx   = mpu_data.gx / 16.384f ; 
			imu.wy   = mpu_data.gy / 16.384f ; 
			imu.wz   = mpu_data.gz / 16.384f ;
		}
}

///**
// ***************************************(C) COPYRIGHT 2018 DJI***************************************
// * @file       bsp_imu.c
// * @brief      mpu6500 module driver, configurate MPU6500 and Read the Accelerator
// *             and Gyrometer data using SPI interface      
// * @note         
// * @Version    V1.0.0
// * @Date       Jan-30-2018      
// ***************************************(C) COPYRIGHT 2018 DJI***************************************
// */

////#include "bsp_imu.h"
////#include "ist8310_reg.h" 
////#include "stm32f4xx_hal.h"
////#include <math.h>
////#include "mpu6500_reg.h"
////#include "spi.h"

//#define BOARD_DOWN (1)   
//#define IST8310
//#define MPU_HSPI hspi5

//#define Kp 2.0f                                              /* 
//                                                              * proportional gain governs rate of 
//                                                              * convergence to accelerometer/magnetometer 
//																															*/
//#define Ki 0.01f                                             /* 
//                                                              * integral gain governs rate of 
//                                                              * convergence of gyroscope biases 
//																															*/
//volatile float        q0 = 1.0f;
//volatile float        q1 = 0.0f;
//volatile float        q2 = 0.0f;
//volatile float        q3 = 0.0f;
//volatile float        exInt, eyInt, ezInt;                   /* error integral */
//static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;  
//volatile uint32_t     last_update, now_update;               /* Sampling cycle count, ubit ms */
//static uint8_t        tx, rx;
//static uint8_t        tx_buff[14] = { 0xff };
//uint8_t               mpu_buff[14];                          /* buffer to save imu raw data */
//uint8_t               ist_buff[6];                           /* buffer to save IST8310 raw data */
//mpu_data_t            mpu_data;
//imu_t                 imu={0};

///**
//  * @brief  fast inverse square-root, to calculate 1/Sqrt(x)
//  * @param  x: the number need to be calculated
//  * @retval 1/Sqrt(x)
//  * @usage  call in imu_ahrs_update() function
//  */
//float inv_sqrt(float x) 
//{
//	float halfx = 0.5f * x;
//	float y     = x;
//	long  i     = *(long*)&y;
//	
//	i = 0x5f3759df - (i >> 1);
//	y = *(float*)&i;
//	y = y * (1.5f - (halfx * y * y));
//	
//	return y;
//}

//uint8_t id;

///**
//	* @brief  Initialize quaternion
//  * @param  
//	* @retval 
//  * @usage  call in main() function
//	*/
//void init_quaternion(void)
//{
//	int16_t hx, hy;//hz;
//	
//	hx = imu.mx;
//	hy = imu.my;
//	//hz = imu.mz;
//	
//	#ifdef BOARD_DOWN
//	if (hx < 0 && hy < 0) 
//	{
//		if (fabs(hx / hy) >= 1)
//		{
//			q0 = -0.005;
//			q1 = -0.199;
//			q2 = 0.979;
//			q3 = -0.0089;
//		}
//		else
//		{
//			q0 = -0.008;
//			q1 = -0.555;
//			q2 = 0.83;
//			q3 = -0.002;
//		}
//		
//	}
//	else if (hx < 0 && hy > 0)
//	{
//		if (fabs(hx / hy)>=1)   
//		{
//			q0 = 0.005;
//			q1 = -0.199;
//			q2 = -0.978;
//			q3 = 0.012;
//		}
//		else
//		{
//			q0 = 0.005;
//			q1 = -0.553;
//			q2 = -0.83;
//			q3 = -0.0023;
//		}
//		
//	}
//	else if (hx > 0 && hy > 0)
//	{
//		if (fabs(hx / hy) >= 1)
//		{
//			q0 = 0.0012;
//			q1 = -0.978;
//			q2 = -0.199;
//			q3 = -0.005;
//		}
//		else
//		{
//			q0 = 0.0023;
//			q1 = -0.83;
//			q2 = -0.553;
//			q3 = 0.0023;
//		}
//		
//	}
//	else if (hx > 0 && hy < 0)
//	{
//		if (fabs(hx / hy) >= 1)
//		{
//			q0 = 0.0025;
//			q1 = 0.978;
//			q2 = -0.199;
//			q3 = 0.008;			
//		}
//		else
//		{
//			q0 = 0.0025;
//			q1 = 0.83;
//			q2 = -0.56;
//			q3 = 0.0045;
//		}		
//	}
//	#else
//		if (hx < 0 && hy < 0)
//	{
//		if (fabs(hx / hy) >= 1)
//		{
//			q0 = 0.195;
//			q1 = -0.015;
//			q2 = 0.0043;
//			q3 = 0.979;
//		}
//		else
//		{
//			q0 = 0.555;
//			q1 = -0.015;
//			q2 = 0.006;
//			q3 = 0.829;
//		}
//		
//	}
//	else if (hx < 0 && hy > 0)
//	{
//		if(fabs(hx / hy) >= 1)
//		{
//			q0 = -0.193;
//			q1 = -0.009;
//			q2 = -0.006;
//			q3 = 0.979;
//		}
//		else
//		{
//			q0 = -0.552;
//			q1 = -0.0048;
//			q2 = -0.0115;
//			q3 = 0.8313;
//		}
//		
//	}
//	else if (hx > 0 && hy > 0)
//	{
//		if(fabs(hx / hy) >= 1)
//		{
//			q0 = -0.9785;
//			q1 = 0.008;
//			q2 = -0.02;
//			q3 = 0.195;
//		}
//		else
//		{
//			q0 = -0.9828;
//			q1 = 0.002;
//			q2 = -0.0167;
//			q3 = 0.5557;
//		}
//		
//	}
//	else if (hx > 0 && hy < 0)
//	{
//		if(fabs(hx / hy) >= 1)
//		{
//			q0 = -0.979;
//			q1 = 0.0116;
//			q2 = -0.0167;
//			q3 = -0.195;			
//		}
//		else
//		{
//			q0 = -0.83;
//			q1 = 0.014;
//			q2 = -0.012;
//			q3 = -0.556;
//		}		
//	}
//	#endif
//}

///**
//	* @brief  update imu AHRS
//  * @param  
//	* @retval 
//  * @usage  call in main() function
//	*/
//void imu_ahrs_update(void) 
//{
//	float norm;
//	float hx, hy, hz, bx, bz;
//	float vx, vy, vz, wx, wy, wz;
//	float ex, ey, ez, halfT;
//	float tempq0,tempq1,tempq2,tempq3;

//	float q0q0 = q0*q0;
//	float q0q1 = q0*q1;
//	float q0q2 = q0*q2;
//	float q0q3 = q0*q3;
//	float q1q1 = q1*q1;
//	float q1q2 = q1*q2;
//	float q1q3 = q1*q3;
//	float q2q2 = q2*q2;   
//	float q2q3 = q2*q3;
//	float q3q3 = q3*q3;   

//	gx = imu.wx;
//	gy = imu.wy;
//	gz = imu.wz;
//	ax = imu.ax;
//	ay = imu.ay;
//	az = imu.az;
//	mx = imu.mx;
//	my = imu.my;
//	mz = imu.mz;

//	now_update  = HAL_GetTick(); //ms
//	halfT       = ((float)(now_update - last_update) / 2000.0f);
//	last_update = now_update;
//	
//	/* Fast inverse square-root */
//	norm = inv_sqrt(ax*ax + ay*ay + az*az);       
//	ax = ax * norm;
//	ay = ay * norm;
//	az = az * norm;
//	
//	#ifdef IST8310
//		norm = inv_sqrt(mx*mx + my*my + mz*mz);          
//		mx = mx * norm;
//		my = my * norm;
//		mz = mz * norm; 
//	#else
//		mx = 0;
//		my = 0;
//		mz = 0;		
//	#endif
//	/* compute reference direction of flux */
//	hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
//	hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
//	hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);         
//	bx = sqrt((hx*hx) + (hy*hy));
//	bz = hz; 
//	
//	/* estimated direction of gravity and flux (v and w) */
//	vx = 2.0f*(q1q3 - q0q2);
//	vy = 2.0f*(q0q1 + q2q3);
//	vz = q0q0 - q1q1 - q2q2 + q3q3;
//	wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
//	wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
//	wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  
//	
//	/* 
//	 * error is sum of cross product between reference direction 
//	 * of fields and direction measured by sensors 
//	 */
//	ex = (ay*vz - az*vy) + (my*wz - mz*wy);
//	ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
//	ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

//	/* PI */
//	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
//	{
//		exInt = exInt + ex * Ki * halfT;
//		eyInt = eyInt + ey * Ki * halfT;	
//		ezInt = ezInt + ez * Ki * halfT;
//		
//		gx = gx + Kp*ex + exInt;
//		gy = gy + Kp*ey + eyInt;
//		gz = gz + Kp*ez + ezInt;
//	}
//	
//	tempq0 = q0 + (-q1*gx - q2*gy - q3*gz) * halfT;
//	tempq1 = q1 + (q0*gx + q2*gz - q3*gy) * halfT;
//	tempq2 = q2 + (q0*gy - q1*gz + q3*gx) * halfT;
//	tempq3 = q3 + (q0*gz + q1*gy - q2*gx) * halfT;  

//	/* normalise quaternion */
//	norm = inv_sqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
//	q0 = tempq0 * norm;
//	q1 = tempq1 * norm;
//	q2 = tempq2 * norm;
//	q3 = tempq3 * norm;
//}

///**
//	* @brief  update imu attitude
//  * @param  
//	* @retval 
//  * @usage  call in main() function
//	*/
//void imu_attitude_update(void)
//{
//	/* yaw    -pi----pi */
//	imu.yaw = -atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1)* 57.3; 
//	/* pitch  -pi/2----pi/2 */
//	imu.pit = -asin(-2*q1*q3 + 2*q0*q2)* 57.3;   
//	/* roll   -pi----pi  */	
//	imu.rol =  atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1)* 57.3;
//}












//*
//******************************************************************
//**  Filename :  mpu6500.C
//**  Abstract :  mpu6500��spi��������
//**  Device   :  stm32f4xx
//**  Compiler :  keil 5
//**  By       :  yulong <hylong2111@163.com>
//**  Date     :  2017-09-21 17:25:39
//**  Changelog:1.�״δ���
//*******************************************************************
//*/
//#include "mpu6500.h"
//#include "stm32f4xx_exti.h"
//#include "stdio.h"
//#include "exti.h"
//#include "Show_Scope.h"
//#include "sys.h"
//#include "delay.h"
//#include "usart.h"   
//#include "led.h"
// 
// 
// 
///**��ʼ��mpu6500�˿�**/
//void mpu6500_Init(void) 
//{
//    GPIO_InitTypeDef    GPIO_InitStructure;
//	NVIC_InitTypeDef	NVIC_InitStructure;
//	EXTI_InitTypeDef 	EXTI_InitStructure;
//    
//    //mpu6500 CS��g11
//    GPIO_InitStructure.GPIO_Pin = mpu6500_CS;
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
// 	GPIO_Init(mpu6500_CS_G, &GPIO_InitStructure);
//    GPIO_SetBits(mpu6500_CS_G, mpu6500_CS);//CS �ߵ�ƽ���Ȳ�ѡ��
//    //��������SPI����������Ƭѡ����
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;//PG15 //flash_cs
//    GPIO_Init(GPIOG, &GPIO_InitStructure);//��ʼ��
//    GPIO_SetBits(GPIOG, GPIO_Pin_15);//PG15���1,��ֹNRF����SPI FLASH��ͨ�� 
//    
//    //mpu6500 DRDY��
//    GPIO_InitStructure.GPIO_Pin = mpu6500_DRDY;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100M
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
// 	GPIO_Init(mpu6500_DRDY_G, &GPIO_InitStructure);   
//    
//    SPI3_Init();		   			//��ʼ��SPI ģʽ3
//    SPI3_SetSpeed(SPI_BaudRatePrescaler_256);		//����Ϊ42Mʱ��,����ģʽ 
// 
//    //DRDY�жϽ��ճ�ʼ��
//    //EXTIX_Init();
//}
// 
// 
////��ʼ��MPU6500
////����ֵ:0,�ɹ�
////    ����,�������
//u8 MPU6500_Init(void)
//{ 
//	u8 res, t=5;
//	
//	mpu6500_Init();//��ʼ��spi����.exit�ⲿ�ж�
//	
//	mpu6500_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//��λmpu6500
//    delay_ms(100); //see ��register map��page 42 - delay 100ms
//	mpu6500_Write_Byte(MPU_SIGPATH_RST_REG,0X07);	//reset GYR+ACC+TEMP
//	delay_ms(100); //page 42 - delay 100ms
//	mpu6500_Write_Byte(MPU_USER_CTRL_REG, 0x11); //SET spi mode+Reset all gyro digital signal path, accel digital signal path, and temp
//	delay_ms(1000);
//	
//	res=mpu6500_Read_Byte(MPU_DEVICE_ID_REG);
//	if(res == 0x70)//����ID��ȷ
//	{
//		printf("mpu6500_ADDR INIT OK!\n");
//		
//		MPU_Set_Gyro_Fsr(0);					//�����Ǵ�����,��250dps
//		delay_ms(10);	//ÿдһ���Ĵ���ע����ʱ����Ȼ�������벻���Ĵ���lol --yulong
//		MPU_Set_Accel_Fsr(0);					//���ٶȴ�����,��2g
//		delay_ms(10);
//		mpu6500_Write_Byte(MPU_CFG_REG,0X03);	//gyr Fs=1khz,bandwidth=41hz
//		delay_ms(10); 
//		mpu6500_Write_Byte(MPU_ACCEL_CFG2_REG,0X03);	//Acc Fs=1khz, bandtidth=41hz
//		delay_ms(10);
//		//mpu6500_Write_Byte(MPU_INTBP_CFG_REG,0XA0);	//INT���ŵ͵�ƽ��Ч,�������
//		delay_ms(10);
//		//mpu6500_Write_Byte(MPU_INT_EN_REG,0X01);	//raw data inter open
//		delay_ms(10);
//		//mpu6500_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//����CLKSEL,PLL X��Ϊ�ο�
//		delay_ms(10);
//		mpu6500_Write_Byte(MPU_PWR_MGMT2_REG, 0X00); //���ٶ��������Ƕ�����
//		delay_ms(10);
//		MPU_Set_Rate(200);						//���ò�����Ϊ200Hz
//		delay_ms(10);
// 	}
//	else 
//	{
//		printf("ERROR! mpu6500_ADDR IS %x\n", mpu6500_Read_Byte(MPU_DEVICE_ID_REG));
//		return 1; //�����˳�
//	}
// 
//	//just for test --yulong 2017/9/20
//	//loop all the time(send data to com)
//	while(1)
//	{
//		short accx_original=0, accy_original=0, accz_original=0;
//		u16 ACC_DATA[7];
//		u8 raw_datas[14]={0}; //acc*6+temp*2+gyr*6
//		u8 res;
//		
//		res = mpu6500_Read_Byte(MPU_INT_STA_REG); //Ĭ�϶��˼Ĵ����ܹ���˱�־λ.��ѭ����ѯ�˼Ĵ�������
//		//printf("int status:%x\n", res);
//		delay_us(10);
//		if(res == 0x01) //����ready
//		{
//			mpu6500_Read_Len(MPU_ACCEL_XOUTH_REG, 8, &raw_datas[0]);
//			delay_us(10);
//			mpu6500_Read_Len(MPU_GYRO_XOUTH_REG, 6, &raw_datas[8]);
//			delay_us(10);
// 
//			ACC_DATA[0]=((u16)raw_datas[0]<<8)|raw_datas[1];//������ٶ�
//			ACC_DATA[1]=((u16)raw_datas[2]<<8)|raw_datas[3];
//			ACC_DATA[2]=((u16)raw_datas[4]<<8)|raw_datas[5];
//			ACC_DATA[3]=((u16)raw_datas[8]<<8)|raw_datas[9];//������ٶ�
//			ACC_DATA[4]=((u16)raw_datas[10]<<8)|raw_datas[11];
//			ACC_DATA[5]=((u16)raw_datas[12]<<8)|raw_datas[13];
//			ACC_DATA[6]=((u16)raw_datas[6]<<8)|raw_datas[7]; //�¶�����
//			Data_Send_Status(ACC_DATA, 0XF1, 7); //7ͨ�����ݣ����͸�������λ��������ʾ
//			
//			LED3_ON(); //open red led
//		}
//	}
//	return 0;
//}
////����MPU6050�����Ǵ����������̷�Χ
////fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
////����ֵ:0,���óɹ�
////    ����,����ʧ�� 
//u8 MPU_Set_Gyro_Fsr(u8 fsr)
//{
//	return mpu6500_Write_Byte(MPU_GYRO_CFG_REG, fsr<<3);//���������������̷�Χ  
//}
////����MPU6050���ٶȴ����������̷�Χ
////fsr:0,��2g;1,��4g;2,��8g;3,��16g
////����ֵ:0,���óɹ�
////    ����,����ʧ�� 
//u8 MPU_Set_Accel_Fsr(u8 fsr)
//{
//	return mpu6500_Write_Byte(MPU_ACCEL_CFG_REG, fsr<<3);//���ü��ٶȴ����������̷�Χ  
//}
////����MPU6050�����ֵ�ͨ�˲���
////lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
////����ֵ:0,���óɹ�
////    ����,����ʧ�� 
//u8 MPU_Set_LPF(u16 lpf)
//{
//	u8 data=0;
//	if(lpf>=188)data=1;
//	else if(lpf>=98)data=2;
//	else if(lpf>=42)data=3;
//	else if(lpf>=20)data=4;
//	else if(lpf>=10)data=5;
//	else data=6; 
//	return mpu6500_Write_Byte(MPU_CFG_REG,data);//�������ֵ�ͨ�˲���  
//}
////����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
////rate:4~1000(Hz)
////����ֵ:0,���óɹ�
////    ����,����ʧ�� 
//u8 MPU_Set_Rate(u16 rate)
//{
//	u8 data;
//	if(rate>1000)rate=1000; //���1khz������
//	if(rate<4)rate=4;
//	data=1000/rate-1; //���ݹ�ʽ�����
//	data=mpu6500_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//���ò�����
// 	//return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
//}
// 
// 

// 
// 
// 
