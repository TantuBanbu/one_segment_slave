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
	//串口空闲中断
	if(USART6->SR & USART_FLAG_IDLE)
	{
		com6_recv_length = 100 - DMA2_Stream1->NDTR;		//获得单包连续数据的长度
		temp = USART6->DR;
		temp = USART6->SR;
		
		//处理数据
		imu_eulrangle_resolve(recv_buf);
		
		USART6->SR &=(~USART_FLAG_IDLE);//清理中断标志位
		//重新装载dma
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
//	 if (USART_GetFlagStatus(USART6, USART_FLAG_ORE) != RESET)//防止接受数据太快导致溢出
//   {  	
//		//清除中断标志    		 
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
  //GPIO端口设置
	
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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	//复用推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOG, &GPIO_InitStructure);


  //Usart6 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;    ////////////////
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
	NVIC_Init(&NVIC_InitStructure);	
  //对应dma中断
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;    ////////////////
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
	NVIC_Init(&NVIC_InitStructure);	
	
	//USART 初始化设置
	USART_DeInit(USART6);
	USART_InitStructure.USART_BaudRate 		= 115200;//串口波特率
	USART_InitStructure.USART_WordLength 	= USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits 		= USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity 			= USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_Mode 				= USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_Init(USART6,&USART_InitStructure);
  USART_Cmd(USART6, ENABLE);                    //使能串口2 
	
	USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);												//开启串口空闲中断	
	USART_ClearITPendingBit(USART6, USART_IT_IDLE);

	USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);													//打开USART1->DMA发送中断
	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);													//打开USART1->DMA接收中断	

/*串口发送DMA配置*/
	DMA_DeInit(DMA2_Stream6);																															 
	DMA_InitStructure.DMA_Channel 						= 	DMA_Channel_5;   					//DMA通道配置
	DMA_InitStructure.DMA_PeripheralBaseAddr 			= 	(uint32_t)(&USART6->DR);  			//外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr 				= 	(uint32_t)(&trans_buf[0]);	  	//内存地址  
	DMA_InitStructure.DMA_DIR 							= 	DMA_DIR_MemoryToPeripheral;  		//dma传输方向
	DMA_InitStructure.DMA_BufferSize 					= 	10;					//设置DMA在传输时缓冲区的长度 
	DMA_InitStructure.DMA_PeripheralInc 				= 	DMA_PeripheralInc_Disable;  		//设置DMA的外设递增模式，一个外设 
	DMA_InitStructure.DMA_MemoryInc 					= 	DMA_MemoryInc_Enable; 				//设置DMA的内存递增模式
	DMA_InitStructure.DMA_PeripheralDataSize 			= 	DMA_PeripheralDataSize_Byte;  		//外设数据字长
	DMA_InitStructure.DMA_MemoryDataSize 				= 	DMA_MemoryDataSize_Byte; 			//内存数据字长 
	DMA_InitStructure.DMA_Mode 							= 	DMA_Mode_Normal; 					//设置DMA的传输模式，DMA_Mode_Circular时，NDTR可以自动重新装载，即可以反复发送接收；若为DMA_Mode_Normal，NDTR需要每次接收发送完成后进行手动装载   
	DMA_InitStructure.DMA_Priority 						= 	DMA_Priority_High;  				//设置DMA的优先级别
	DMA_InitStructure.DMA_FIFOMode 						= 	DMA_FIFOMode_Disable;      			//指定如果FIFO模式或直接模式将用于指定的流 ： 不使能FIFO模式
	DMA_InitStructure.DMA_FIFOThreshold 				=	DMA_FIFOThreshold_HalfFull;    		//指定了FIFO阈值水平       
	DMA_InitStructure.DMA_MemoryBurst 					=	DMA_MemoryBurst_Single;         	//指定的Burst转移配置内存传输
	DMA_InitStructure.DMA_PeripheralBurst 				=	DMA_PeripheralBurst_Single;  		//指定的Burst转移配置外围转移         
	DMA_Init(DMA2_Stream6, &DMA_InitStructure);    												//配置DMA1的通道 
	DMA_Cmd(DMA2_Stream6,DISABLE);																//禁用DMA对USART1的发送通道，等使用时打开

	DMA_ITConfig(DMA2_Stream6,DMA_IT_TC,ENABLE);     											//使能发送完成中断
	DMA_ClearFlag(DMA2_Stream6,DMA_FLAG_TCIF6);

/*串口接收DMA配置*/
	DMA_DeInit(DMA2_Stream1);
	DMA_InitStructure.DMA_Channel 						= 	DMA_Channel_5;
	DMA_InitStructure.DMA_PeripheralBaseAddr 			= 	(uint32_t)(&USART6->DR);  			//外设地址    
	DMA_InitStructure.DMA_Memory0BaseAddr 				=	(uint32_t)(&recv_buf[0]);  		//内存地址   
	DMA_InitStructure.DMA_DIR 							=	DMA_DIR_PeripheralToMemory;  		//dma传输方向   
	DMA_InitStructure.DMA_BufferSize 					=	100;					//设置DMA在传输时缓冲区的长度     
	DMA_InitStructure.DMA_PeripheralInc 				=	DMA_PeripheralInc_Disable; 			//设置DMA的外设递增模式，一个外设   
	DMA_InitStructure.DMA_MemoryInc 					=	DMA_MemoryInc_Enable;    			//设置DMA的内存递增模式
	DMA_InitStructure.DMA_PeripheralDataSize 			=	DMA_PeripheralDataSize_Byte;     	//外设数据字长   
	DMA_InitStructure.DMA_MemoryDataSize 				=	DMA_MemoryDataSize_Byte;      		//内存数据字长 
	DMA_InitStructure.DMA_Mode 							=	DMA_Mode_Circular;     				//设置DMA的传输模式，DMA_Mode_Circular时，NDTR可以自动重新装载，即可以反复发送接收；若为DMA_Mode_Normal，NDTR需要每次接收发送完成后进行手动装载
	DMA_InitStructure.DMA_Priority 						=	DMA_Priority_High;      			//设置DMA的优先级别        
	DMA_InitStructure.DMA_FIFOMode 						=	DMA_FIFOMode_Disable;      			//指定如果FIFO模式或直接模式将用于指定的流 ： 不使能FIFO模式
	DMA_InitStructure.DMA_FIFOThreshold 				=	DMA_FIFOThreshold_HalfFull;       	//指定了FIFO阈值水平    
	DMA_InitStructure.DMA_MemoryBurst 					=	DMA_MemoryBurst_Single;           	//指定的Burst转移配置内存传输  
	DMA_InitStructure.DMA_PeripheralBurst 				=	DMA_PeripheralBurst_Single;       	//指定的Burst转移配置外围转移         
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);        											//配置DMA2的通道 
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