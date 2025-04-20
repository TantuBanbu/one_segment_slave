//#include "can1.h"
#include "task.h"
#include "delay.h"
//#include "uart1.h"
//#include "uart3.h"
#include "uart6.h"
#include "packet.h"
#include "imu_data_decode.h"
//#include "pwm.h"
//#include "imu.h"
//#include "gpio.h"
#include "includes.h"
//////////////////////////////////////////////////////////////////////////
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos ʹ��	  
#endif

u8 ahrs_rx_buffer[45]={0};
float Yaw_angle[2],Angle_Yaw;
//int round_count;
u8 ahrs_error=1;

//void AHRS_USART6_Init(void)   //��ߵ�ģ��ͨ��
//{                 
//		USART_InitTypeDef usart;
//    GPIO_InitTypeDef  gpio;
//	  NVIC_InitTypeDef  nvic;
//		DMA_InitTypeDef   dma;

//	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
//		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
// 
//		GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6); 
//		GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6); 
//	
//		gpio.GPIO_Pin = GPIO_Pin_9| GPIO_Pin_14; 
//		gpio.GPIO_Mode = GPIO_Mode_AF;
//		gpio.GPIO_Speed = GPIO_Speed_100MHz;	
//		gpio.GPIO_OType = GPIO_OType_PP; 
//		gpio.GPIO_PuPd = GPIO_PuPd_UP;
//		GPIO_Init(GPIOG,&gpio); 

//	  nvic.NVIC_IRQChannel = USART6_IRQn;
//    nvic.NVIC_IRQChannelPreemptionPriority = 0;
//    nvic.NVIC_IRQChannelSubPriority = 1;
//    nvic.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&nvic);
//		
//    USART_DeInit(USART6);
//    usart.USART_BaudRate = 115200;                //UART2 9600bps
//    usart.USART_WordLength = USART_WordLength_8b;//�ֳ�8B
//    usart.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
//    usart.USART_Parity = USART_Parity_No;//��żУ��λ
//    usart.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;//����ģʽ  ����ģʽ
//    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//    USART_Init(USART6,&usart);
//		
//		DMA_DeInit(DMA2_Stream1);
//    dma.DMA_Channel= DMA_Channel_5;
//    dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART6->DR);//�������ݼĴ�����ַ
//    dma.DMA_Memory0BaseAddr = (uint32_t)&ahrs_rx_buffer;//�ڴ��ַ
//    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;//���ݴ��䷽��  ���赽�ڴ�
//    dma.DMA_BufferSize =45;  //��������СΪ20�ֽ�
//    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ������
//    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;//�ڴ�����ַ����
//    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//    dma.DMA_Mode = DMA_Mode_Circular;//ѭ���ɼ�
//    dma.DMA_Priority = DMA_Priority_High;//DMAͨ�����ȼ�
//    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;//�����ȳ���FIFO��
//    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;//ѡ��FIFO��ֵ
//    dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ����������
//    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ����������
//		
//    DMA_Init(DMA2_Stream1,&dma);
//		
//		USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);//��������DMA���͹���
//		USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);//�������ڿ����ж�
//		USART_Cmd(USART6,ENABLE);
//		DMA_Cmd(DMA2_Stream1,ENABLE);
//		
////		imu_data_decode_init();
//		
//		
//		AHRS_Transmit_Start();
//		delay_ms(100);
//		AHRS_Transmit_Stop_Magnetic_Steering();
//		while(Angle_Yaw<0.001f && Angle_Yaw>-0.001f){}
//			
//}

//void USART6_IRQHandler(void)
//{  
//	 u8 Current_NDTR=0;
//	 OSIntEnter();
//   if (USART_GetITStatus(USART6,USART_IT_IDLE) != RESET)  
//   {  
//			 //����жϱ�־    
//			 (void)USART6->SR;   
//			 (void)USART6->DR;
////		    uint8_t tmp = USART_ReceiveData(USART6);
////		    Packet_Decode(tmp);
//		 
//			 DMA_Cmd(DMA2_Stream1, DISABLE);                      //�ر�DMA  
//			 Current_NDTR=DMA2_Stream1->NDTR;                     //��ȡʣ�໺����
//			 DMA2_Stream1->NDTR=45;                               //����DMA����ֵ
//			 DMA_Cmd(DMA2_Stream1, ENABLE);                       //����DMA	
//			 
//			 if(Current_NDTR==0x05)       //��ʣ��5�������ֽڣ�˵����һ֡��������
//			 {
//				 AHRS_Yaw_angle=((int16_t)ahrs_rx_buffer[3]<<8)|ahrs_rx_buffer[4];
//         Yaw_angle[0]=Yaw_angle[1];
//				 Yaw_angle[1]=(((int16_t)ahrs_rx_buffer[3]<<8)|ahrs_rx_buffer[4])*0.1f;
//				 if(Yaw_angle[1] - Yaw_angle[0] > 300)
//					{
//						round_count -- ;
//					}
//					if(Yaw_angle[1] - Yaw_angle[0] < -300)
//					{
//						round_count ++ ;
//					}
//					Angle_Yaw=round_count*360.0f+Yaw_angle[1];
//          ahrs_error=0;
//						
//			 }			
//		}		 		 	
//		OSIntExit();
//}

void AHRS_Transmit_Start(void)
{
	u8 command[6]={0xa5,0x5a,0x04,0x01,0x05,0xaa};
  u8 i=0;
	while(i<6)
	{
		USART_ClearFlag(USART6,USART_FLAG_TC);
		USART_SendData(USART6,command[i++]);
		while(USART_GetFlagStatus(USART6,USART_FLAG_TXE)==RESET);//�ж��Ƿ������
	}
}

void AHRS_Transmit_Stop(void)
{
	u8 command[6]={0xa5,0x5a,0x04,0x02,0x06,0xaa};
  u8 i=0;
	while(i<6)
	{
		USART_ClearFlag(USART6,USART_FLAG_TC);
		USART_SendData(USART6,command[i++]);
		while(USART_GetFlagStatus(USART6,USART_FLAG_TXE)==RESET);//�ж��Ƿ������
	}
}

void AHRS_Transmit_Stop_Magnetic_Steering(void)
{
	u8 command[6]={0xa5,0x5a,0x04,0xe4,0xe8,0xaa};
  u8 i=0;
	while(i<6)
	{
		USART_ClearFlag(USART6,USART_FLAG_TC);
		USART_SendData(USART6,command[i++]);
		while(USART_GetFlagStatus(USART6,USART_FLAG_TXE)==RESET);//�ж��Ƿ������
	}
}



