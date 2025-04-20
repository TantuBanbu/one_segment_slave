#include "usart69050.h"	
#include "sys.h"	
#include "packet.h"
#include "imu_data_decode.h"
uint8_t tmp;
u8 IMU_Data[41]={0};
float PITCH_IMU;
void USART6_IRQHandler(void)
{
	OSIntEnter();
	if(USART_GetFlagStatus(USART6,USART_FLAG_ORE)!=RESET)
	{
		(void)USART6->SR;   
		(void)USART6->DR;
		return;
	}
	 if (USART_GetFlagStatus(USART6, USART_FLAG_ORE) != RESET)//��ֹ��������̫�쵼�����
   {  	
		//����жϱ�־    		 
		(void)USART6->SR;   
		(void)USART6->DR;
		return;			
   }
	if(USART_GetITStatus(USART6, USART_IT_RXNE))
	{	 
		tmp=USART_ReceiveData(USART6);	
		Packet_Decode(tmp);
//		DATA_IMU(tmp);
//		if(IMU_Data[0]==0xD0)
//		{PITCH_IMU=IMU_Data[1]|(IMU_Data[2]<<8);}
	}
	USART_ClearFlag(USART6, USART_FLAG_RXNE);  
	USART_ClearITPendingBit(USART6, USART_IT_RXNE);
	
	  OSIntExit(); 
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




void USART6_init(void)
	{
  //GPIO�˿�����
	
	USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
	
	
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6); 
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6); 
	
	
	//USART2_TX   GPIOA2
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14; //PA2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	//�����������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOG, &GPIO_InitStructure);


  //Usart2 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;    ////////////////
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
	NVIC_Init(&NVIC_InitStructure);	

	//USART ��ʼ������
	USART_DeInit(USART6);
	USART_InitStructure.USART_BaudRate = 115200;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_Init(USART6,&USART_InitStructure);

  USART_Init(USART6, &USART_InitStructure);     //��ʼ������2
  USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART6, ENABLE);                    //ʹ�ܴ���2 
	
  imu_data_decode_init();
	
}






