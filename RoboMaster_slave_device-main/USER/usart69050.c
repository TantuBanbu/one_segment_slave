#include "usart69050.h"	
#include "sys.h"	
#include "packet.h"
//#include "imu_data_decode.h"
uint8_t tmp;
u8 IMU_Data[41]={0};
float PITCH_IMU;

//private functions=========================
void imu_eulrangle_resolve(uint8_t * data);
//global values============================
uint8_t recv_buf[100];
uint8_t trans_buf[10];
uint16_t com6_recv_length=0;
uint16_t temp=0;
void USART6_IRQHandler(void)
{
	//���ڿ����ж�
	if(USART6->SR & USART_FLAG_IDLE)
	{
		com6_recv_length = 100 - DMA2_Stream1->NDTR;		//��õ����������ݵĳ���
		temp = USART6->DR;
		temp = USART6->SR;
		
		//��������
		imu_eulrangle_resolve(recv_buf);
		
		USART6->SR &=(~USART_FLAG_IDLE);//�����жϱ�־λ
		//����װ��dma
		DMA_Cmd(DMA2_Stream1,DISABLE);
		DMA2_Stream1->M0AR = (uint32_t)&recv_buf[0];
		DMA2_Stream1->NDTR = 100;
		DMA_Cmd(DMA2_Stream1,ENABLE);
	}
	
//	OSIntEnter();
//	if(USART_GetFlagStatus(USART6,USART_FLAG_ORE)!=RESET)
//	{
//		(void)USART6->SR;   
//		(void)USART6->DR;
//		return;
//	}
//	 if (USART_GetFlagStatus(USART6, USART_FLAG_ORE) != RESET)//��ֹ��������̫�쵼�����
//   {  	
//		//����жϱ�־    		 
//		(void)USART6->SR;   
//		(void)USART6->DR;
//		return;			
//   }
//	if(USART_GetITStatus(USART6, USART_IT_RXNE))
//	{	 
//		tmp=USART_ReceiveData(USART6);	
//		Packet_Decode(tmp);
////		DATA_IMU(tmp);
////		if(IMU_Data[0]==0xD0)
////		{PITCH_IMU=IMU_Data[1]|(IMU_Data[2]<<8);}
//	}
//	USART_ClearFlag(USART6, USART_FLAG_RXNE);  
//	USART_ClearITPendingBit(USART6, USART_IT_RXNE);
//	
//	  OSIntExit(); 
}
u8 imu_flag=0;
void DATA_IMU(u8 Res)
{
	
  static u8 count=0;
	static u8 i=0;
	if((Res==0xD0)&&(imu_flag==1)){count=30;imu_flag=0;}
	switch(count)
				{
					case 0:
						if(Res==0x5A)
							count++;
						else
							count=0;
						break;
					case 1:
						if(Res==0xA5)
							count++;
						else
							count=0;
						break;
          case 2:
            if(Res==0x23)
							imu_flag=1;
						else
							imu_flag=0;
						  count=0;
						break;
         case 30:
				    IMU_Data[i]=Res;
				    i++;
						count++;
						break;
				 case 31:
				    IMU_Data[i]=Res;
				    i++;
						count++;
						break;
					case 32:
						IMU_Data[i]=Res;
					  i++;
					  count++;
					case 33:
            IMU_Data[i]=Res;
					  i++;
					  count++;
						break;
					case 34:
						IMU_Data[i]=Res;
					  i++;
					  count++;
						break;
					case 35:
						IMU_Data[i]=Res;
					  i++;
					  count++;
						break;
					case 36:
						IMU_Data[i]=Res;
					  i++;
					  count++;
						break;
        default:
            count=0;
            break;
        }
				
}


/**
USART6_TX   DMA2_Stream6_ch5
USART6_RX   DMA2_Stream1_ch5
*/

void USART6_init(void)
	{
  //GPIO�˿�����
	
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	DMA_InitTypeDef		DMA_InitStructure;
		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6); 
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6); 
	
	
	//USART6_TX   GPIOA2
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14; //PA2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	//�����������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOG, &GPIO_InitStructure);


  //Usart6 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;    ////////////////
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
	NVIC_Init(&NVIC_InitStructure);	
  //��Ӧdma�ж�
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;    ////////////////
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
	NVIC_Init(&NVIC_InitStructure);	
	
	//USART ��ʼ������
	USART_DeInit(USART6);
	USART_InitStructure.USART_BaudRate 		= 115200;//���ڲ�����
	USART_InitStructure.USART_WordLength 	= USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits 		= USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity 			= USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_Mode 				= USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_Init(USART6,&USART_InitStructure);
  USART_Cmd(USART6, ENABLE);                    //ʹ�ܴ���2 
	
	USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);												//�������ڿ����ж�	
	USART_ClearITPendingBit(USART6, USART_IT_IDLE);

	USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);													//��USART1->DMA�����ж�
	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);													//��USART1->DMA�����ж�	

/*���ڷ���DMA����*/
	DMA_DeInit(DMA2_Stream6);																															 
	DMA_InitStructure.DMA_Channel 						= 	DMA_Channel_5;   					//DMAͨ������
	DMA_InitStructure.DMA_PeripheralBaseAddr 			= 	(uint32_t)(&USART6->DR);  			//�����ַ
	DMA_InitStructure.DMA_Memory0BaseAddr 				= 	(uint32_t)(&trans_buf[0]);	  	//�ڴ��ַ  
	DMA_InitStructure.DMA_DIR 							= 	DMA_DIR_MemoryToPeripheral;  		//dma���䷽��
	DMA_InitStructure.DMA_BufferSize 					= 	10;					//����DMA�ڴ���ʱ�������ĳ��� 
	DMA_InitStructure.DMA_PeripheralInc 				= 	DMA_PeripheralInc_Disable;  		//����DMA���������ģʽ��һ������ 
	DMA_InitStructure.DMA_MemoryInc 					= 	DMA_MemoryInc_Enable; 				//����DMA���ڴ����ģʽ
	DMA_InitStructure.DMA_PeripheralDataSize 			= 	DMA_PeripheralDataSize_Byte;  		//���������ֳ�
	DMA_InitStructure.DMA_MemoryDataSize 				= 	DMA_MemoryDataSize_Byte; 			//�ڴ������ֳ� 
	DMA_InitStructure.DMA_Mode 							= 	DMA_Mode_Normal; 					//����DMA�Ĵ���ģʽ��DMA_Mode_Circularʱ��NDTR�����Զ�����װ�أ������Է������ͽ��գ���ΪDMA_Mode_Normal��NDTR��Ҫÿ�ν��շ�����ɺ�����ֶ�װ��   
	DMA_InitStructure.DMA_Priority 						= 	DMA_Priority_High;  				//����DMA�����ȼ���
	DMA_InitStructure.DMA_FIFOMode 						= 	DMA_FIFOMode_Disable;      			//ָ�����FIFOģʽ��ֱ��ģʽ������ָ������ �� ��ʹ��FIFOģʽ
	DMA_InitStructure.DMA_FIFOThreshold 				=	DMA_FIFOThreshold_HalfFull;    		//ָ����FIFO��ֵˮƽ       
	DMA_InitStructure.DMA_MemoryBurst 					=	DMA_MemoryBurst_Single;         	//ָ����Burstת�������ڴ洫��
	DMA_InitStructure.DMA_PeripheralBurst 				=	DMA_PeripheralBurst_Single;  		//ָ����Burstת��������Χת��         
	DMA_Init(DMA2_Stream6, &DMA_InitStructure);    												//����DMA1��ͨ�� 
	DMA_Cmd(DMA2_Stream6,DISABLE);																//����DMA��USART1�ķ���ͨ������ʹ��ʱ��

	DMA_ITConfig(DMA2_Stream6,DMA_IT_TC,ENABLE);     											//ʹ�ܷ�������ж�
	DMA_ClearFlag(DMA2_Stream6,DMA_FLAG_TCIF6);

/*���ڽ���DMA����*/
	DMA_DeInit(DMA2_Stream1);
	DMA_InitStructure.DMA_Channel 						= 	DMA_Channel_5;
	DMA_InitStructure.DMA_PeripheralBaseAddr 			= 	(uint32_t)(&USART6->DR);  			//�����ַ    
	DMA_InitStructure.DMA_Memory0BaseAddr 				=	(uint32_t)(&recv_buf[0]);  		//�ڴ��ַ   
	DMA_InitStructure.DMA_DIR 							=	DMA_DIR_PeripheralToMemory;  		//dma���䷽��   
	DMA_InitStructure.DMA_BufferSize 					=	100;					//����DMA�ڴ���ʱ�������ĳ���     
	DMA_InitStructure.DMA_PeripheralInc 				=	DMA_PeripheralInc_Disable; 			//����DMA���������ģʽ��һ������   
	DMA_InitStructure.DMA_MemoryInc 					=	DMA_MemoryInc_Enable;    			//����DMA���ڴ����ģʽ
	DMA_InitStructure.DMA_PeripheralDataSize 			=	DMA_PeripheralDataSize_Byte;     	//���������ֳ�   
	DMA_InitStructure.DMA_MemoryDataSize 				=	DMA_MemoryDataSize_Byte;      		//�ڴ������ֳ� 
	DMA_InitStructure.DMA_Mode 							=	DMA_Mode_Circular;     				//����DMA�Ĵ���ģʽ��DMA_Mode_Circularʱ��NDTR�����Զ�����װ�أ������Է������ͽ��գ���ΪDMA_Mode_Normal��NDTR��Ҫÿ�ν��շ�����ɺ�����ֶ�װ��
	DMA_InitStructure.DMA_Priority 						=	DMA_Priority_High;      			//����DMA�����ȼ���        
	DMA_InitStructure.DMA_FIFOMode 						=	DMA_FIFOMode_Disable;      			//ָ�����FIFOģʽ��ֱ��ģʽ������ָ������ �� ��ʹ��FIFOģʽ
	DMA_InitStructure.DMA_FIFOThreshold 				=	DMA_FIFOThreshold_HalfFull;       	//ָ����FIFO��ֵˮƽ    
	DMA_InitStructure.DMA_MemoryBurst 					=	DMA_MemoryBurst_Single;           	//ָ����Burstת�������ڴ洫��  
	DMA_InitStructure.DMA_PeripheralBurst 				=	DMA_PeripheralBurst_Single;       	//ָ����Burstת��������Χת��         
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);        											//����DMA2��ͨ�� 
	DMA_Cmd(DMA2_Stream1,ENABLE);		
	
	
 // imu_data_decode_init();
	
}


int16_t pitch,yaw,roll;
float	f_pitch,f_yaw,f_roll;
void imu_eulrangle_resolve(uint8_t * data)
{
	pitch = (((int16_t)data[30])<<8) | (((int16_t)data[31]));
	f_pitch = ((float)pitch)*0.01f;
	
	roll = (((int16_t)data[32])<<8) | (((int16_t)data[33]));
	f_roll = ((float)roll)*0.01f;
	
	yaw = (((int16_t)data[34])<<8) | (((int16_t)data[35]));
	f_yaw = ((float)yaw)*0.01f;	
	
}