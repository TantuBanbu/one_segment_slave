#include "can1.h"
#include "task.h"
#include "delay.h"
//#include "uart1.h"
#include "uart3.h"
//#include "pwm.h"
//#include "imu.h"
//#include "gpio.h"
#include "includes.h"
/*************************************************************************						
										��ʼ��CAN1��1M������
*************************************************************************/
//////////////////////////////////////////////////////////////////////////
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos ʹ��	  
#endif

// ��Щȫ�ֱ������������������д洢�ʹ�����Ϣ��
int flag;
int16_t receive[4];
int32_t currentPosition_snake[12]={0,0,0,0,0,0,0,0,0,0,0,0};
int32_t offsetPosition_snake[12]={0,0,0,0,0,0,0,0,0,0,0,0};
int8_t currentSpeed_snake[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

int32_t time_counter; //�����ų�ǰ�ڵĲ��ɿ�����


// ����GM6020����λ�õĲ���
int32_t GM6020_absolute_position = 0;  // ����λ��
int16_t GM6020_last_raw_position;  // ��һ�ε�ԭʼλ�ã�0-360��
int16_t GM6020_current_raw_position;  // ��ǰ��ԭʼλ�ã�0-360��
int32_t GM6020_rotation_count = 0;  // ��ת������ÿ���һȦ����1�����1��
// ����C610����λ�õĲ���
int32_t C610_absolute_position = 0;  // ����λ��
int16_t C610_last_raw_position;  // ��һ�ε�ԭʼλ�ã�0-360��
int16_t C610_current_raw_position;  // ��ǰ��ԭʼλ�ã�0-360��
int32_t C610_rotation_count = 0;  // ��ת������ÿ���һȦ����1�����1��

// ����������ڳ�ʼ��CAN1�ӿڡ���������GPIO��NVIC���жϿ���������CAN�������ȡ�
void CAN1_Init(void)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef 	     gpio;
    NVIC_InitTypeDef   	   nvic;
		
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);    //��������ʱ�ӣ�����CAN1����CAN2��Ҫ�����
	
	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);  //IO���Ÿ���
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);

    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_AF;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &gpio);
		
    nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
		
		nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
	
	  CAN_StructInit(&can);	
		can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = ENABLE;
    can.CAN_AWUM = DISABLE;
    can.CAN_NART = DISABLE;
    can.CAN_RFLM = DISABLE;
    can.CAN_TXFP = DISABLE;
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_2tq;
    can.CAN_BS2 = CAN_BS2_4tq;
    can.CAN_Prescaler = 6;   //CAN BaudRate 42/(1+2+4)/6=1Mbps
		
	  CAN_DeInit(CAN1);
		CAN_Init(CAN1, &can);
		
	  can_filter.CAN_FilterNumber = 0;
    can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
    can_filter.CAN_FilterScale = CAN_FilterScale_32bit;
    can_filter.CAN_FilterIdHigh = 0x0000;
    can_filter.CAN_FilterIdLow = 0x0000;
    can_filter.CAN_FilterMaskIdHigh = 0x0000;
    can_filter.CAN_FilterMaskIdLow = 0x0000;
    can_filter.CAN_FilterFIFOAssignment = CAN_FIFO0;
    can_filter.CAN_FilterActivation=ENABLE;
		CAN_FilterInit(&can_filter);
		
		CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);  //ʹ�ܷ����ж�
		CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
}


/*************************************************************************
                          CAN1_TX_IRQHandler
������CAN1�ķ����жϺ���
*************************************************************************/
unsigned char can_tx_success_flag = 0;     //���ͳɹ���־λ
void CAN1_TX_IRQHandler(void)
{
		OSIntEnter();
    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET)
    {
        CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
        can_tx_success_flag=1;
    }
		OSIntExit(); 
}
	 
GripperMotor_ID_t GripperMotor_205_t; // �ֲ���GM6020���
GripperMotor_ID_t GripperMotor_201_t; // �ֲ���C610���
int32_t phase_2006,phase_2006_bodan[2],phase_mid_2006,round_bodan_2006=0;
int32_t phase_6020[2],round_6020_yaw=0;
float anger_bodan_2006,phase_6020_yaw;
float angle_G2C;//��̨����̼н�
/*************************************************************************
                          CAN1_RX0_IRQHandler
������CAN1�Ľ����жϺ���
*************************************************************************/
void CAN1_RX0_IRQHandler(void)
{	
    CanRxMsg rx_message;
    OSIntEnter();
    if ((CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) )
		{
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx_message); 
				
			
				if(time_counter >= 100) {
					switch (rx_message.StdId)
						{
							case 0x01:   //id
							{
								if(rx_message.Data[0]==0x04){
									if(rx_message.Data[1]==0x06)
										currentSpeed_snake[0]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]) * 60 / 65536;
									if(rx_message.Data[1]==0x07)
										currentPosition_snake[0]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]) - offsetPosition_snake[0];	
								}
							}break;	
							case 0x02:   //id
							{
								if(rx_message.Data[0]==0x04){
									if(rx_message.Data[1]==0x06)
										currentSpeed_snake[1]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]) * 60 / 65536;
									if(rx_message.Data[1]==0x07)
										currentPosition_snake[1]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]) - offsetPosition_snake[1];
								}
							}break;	
							case 0x03:   //id
							{
								if(rx_message.Data[0]==0x04){
									if(rx_message.Data[1]==0x06)
										currentSpeed_snake[2]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]) * 60 / 65536;
									if(rx_message.Data[1]==0x07)
										currentPosition_snake[2]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]) - offsetPosition_snake[2];
								}
							}break;	
							case 0x04:   //id
							{
								if(rx_message.Data[0]==0x04){
									if(rx_message.Data[1]==0x06)
										currentSpeed_snake[3]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]) * 60 / 65536;
									if(rx_message.Data[1]==0x07)
										currentPosition_snake[3]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]) - offsetPosition_snake[3];
								}
							}break;	
							case 0x05:   //id
							{
								if(rx_message.Data[0]==0x04){
									if(rx_message.Data[1]==0x06)
										currentSpeed_snake[4]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]) * 60 / 65536;
									if(rx_message.Data[1]==0x07)
										currentPosition_snake[4]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]) - offsetPosition_snake[4];
								}
							}break;	
							case 0x06:   //id
							{
								if(rx_message.Data[0]==0x04){
									if(rx_message.Data[1]==0x06)
										currentSpeed_snake[5]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]) * 60 / 65536;
									if(rx_message.Data[1]==0x07)
										currentPosition_snake[5]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]) - offsetPosition_snake[5];
								}
							}break;	
							case 0x07:   //id
							{
								if(rx_message.Data[0]==0x04){
									if(rx_message.Data[1]==0x06)
										currentSpeed_snake[6]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]) * 60 / 65536;
									if(rx_message.Data[1]==0x07)
										currentPosition_snake[6]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]) - offsetPosition_snake[6];
								}
							}break;	
							case 0x08:   //id
							{
								if(rx_message.Data[0]==0x04){
									if(rx_message.Data[1]==0x06)
										currentSpeed_snake[7]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]) * 60 / 65536;
									if(rx_message.Data[1]==0x07)
										currentPosition_snake[7]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]) - offsetPosition_snake[7];
								}
							}break;	
							case 0x09:   //id
							{
								if(rx_message.Data[0]==0x04){
									if(rx_message.Data[1]==0x06)
										currentSpeed_snake[8]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]) * 60 / 65536;
									if(rx_message.Data[1]==0x07)
										currentPosition_snake[8]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]) - offsetPosition_snake[8];
								}
							}break;	
							case 0x0A:   //id
							{
								if(rx_message.Data[0]==0x04){
									if(rx_message.Data[1]==0x06)
										currentSpeed_snake[9]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]) * 60 / 65536;
									if(rx_message.Data[1]==0x07)
										currentPosition_snake[9]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]) - offsetPosition_snake[9];
								}
							}break;	
							case 0x0B:   //id
							{
								if(rx_message.Data[0]==0x04){
									if(rx_message.Data[1]==0x06)
										currentSpeed_snake[10]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]) * 60 / 65536;
									if(rx_message.Data[1]==0x07)
										currentPosition_snake[10]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]) - offsetPosition_snake[10];
								}
							}break;	
							case 0x0C:   //id
							{
								if(rx_message.Data[0]==0x04){
									if(rx_message.Data[1]==0x06)
										currentSpeed_snake[11]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]) * 60 / 65536;
									if(rx_message.Data[1]==0x07)
										currentPosition_snake[11]= ((rx_message.Data[2]<<24)|(rx_message.Data[3]<<16)|(rx_message.Data[4]<<8)|rx_message.Data[5]) - offsetPosition_snake[11];
								}
							}break;	
							
							//�ֲ����
							case 0x00000205:				// GM6020 �ش����ݣ�ID=1	
							{
								GripperMotor_205_t.position = ((rx_message.Data[0]<<8)|rx_message.Data[1]) *360/8192;;
								GripperMotor_205_t.velocity = (rx_message.Data[2]<<8)|rx_message.Data[3];
								GripperMotor_205_t.current = (rx_message.Data[4]<<8)|rx_message.Data[5];
								GripperMotor_205_t.temperature = rx_message.Data[6];
							}break;
							
							case 0x00000201:				// C610 �ش����ݣ�ID=1	
							{
								GripperMotor_201_t.position = ((rx_message.Data[0]<<8)|rx_message.Data[1])*360/8192;
								GripperMotor_201_t.velocity = (rx_message.Data[2]<<8)|rx_message.Data[3];
								GripperMotor_201_t.current = (rx_message.Data[4]<<8)|rx_message.Data[5];
								GripperMotor_201_t.temperature = rx_message.Data[6];
							}break;

							default:
								break;
						}

						
						
						
				}

    }
		OSIntExit();
}


// ����������ڷ��͵��̵���ĵ���ֵ��
void Chasis_ESC_Send(int16_t current_201,int16_t current_202,int16_t current_203,int16_t current_204)
{
    CanTxMsg tx_message;
  
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (u8)(current_201 >> 8);
    tx_message.Data[1] = (u8)current_201;
    tx_message.Data[2] = (u8)(current_202 >> 8);
    tx_message.Data[3] = (u8)current_202;
		tx_message.Data[4] = (u8)(current_203 >> 8);
    tx_message.Data[5] = (u8)current_203;
    tx_message.Data[6] = (u8)(current_204 >> 8);
    tx_message.Data[7] = (u8)current_204;
    
    CAN_Transmit(CAN1,&tx_message);
}




// ���Ͷ�ȡ�������������ָ��
void readSnakeEncorder(u8 STdId,u8 dlc,u8 D0,u8 D1)	
{
    CanTxMsg tx_message;
  
    tx_message.StdId = STdId;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = dlc;
    
    tx_message.Data[0] = D0;
    tx_message.Data[1] = D1;
    CAN_Transmit(CAN1,&tx_message);
}

// ���Ͷ�ȡ�������������ָ��
void readSnakeSpeed(u8 STdId,u8 dlc,u8 D0,u8 D1)	
{
    CanTxMsg tx_message;
  
    tx_message.StdId = STdId;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = dlc;
    
    tx_message.Data[0] = D0;
    tx_message.Data[1] = D1;
    CAN_Transmit(CAN1,&tx_message);
}

// GM6020�ؽڵ��
void GM6020_Can_Send_Msg(int16_t current_1,int16_t current_2,int16_t current_3,int16_t current_4)
{
    CanTxMsg tx_message; 
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;   
    tx_message.Data[0] = (u8)(current_1 >> 8);
    tx_message.Data[1] = (u8)current_1;
    tx_message.Data[2] = (u8)(current_2 >> 8); 
    tx_message.Data[3] = (u8)current_2;
	  tx_message.Data[4] = (u8)(current_3 >> 8);
    tx_message.Data[5] = (u8)current_3;
    tx_message.Data[6] = (u8)(current_4 >> 8); 
    tx_message.Data[7] = (u8)current_4;
    CAN_Transmit(CAN1,&tx_message);
}

// C610���
void C610_Can_Send_Msg(int16_t current_1,int16_t current_2,int16_t current_3,int16_t current_4)
{
    CanTxMsg tx_message; 
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;   
    tx_message.Data[0] = (u8)(current_1 >> 8);
    tx_message.Data[1] = (u8)current_1;
    tx_message.Data[2] = (u8)(current_2 >> 8); 
    tx_message.Data[3] = (u8)current_2;
	  tx_message.Data[4] = (u8)(current_3 >> 8);
    tx_message.Data[5] = (u8)current_3;
    tx_message.Data[6] = (u8)(current_4 >> 8); 
    tx_message.Data[7] = (u8)current_4;
    CAN_Transmit(CAN1,&tx_message);
}

// ���õ��ʹ��
void motorEnable(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5)
{
    CanTxMsg tx_message;
  
    tx_message.StdId = STdId;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = dlc;
    
    tx_message.Data[0] = D0;
    tx_message.Data[1] = D1;
	  tx_message.Data[2] = D2;
    tx_message.Data[3] = D3;
	  tx_message.Data[4] = D4;
    tx_message.Data[5] = D5;
    CAN_Transmit(CAN1,&tx_message);
}


// ���õ��Ŀ��λ��
void setMotorTargetPosition(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5)
{
    CanTxMsg tx_message;
  
    tx_message.StdId = STdId;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = dlc;
    
    tx_message.Data[0] = D0;
    tx_message.Data[1] = D1;
	  tx_message.Data[2] = D2;
    tx_message.Data[3] = D3;
	  tx_message.Data[4] = D4;
    tx_message.Data[5] = D5;
    CAN_Transmit(CAN1,&tx_message);
}

// ���õ��λ��ƫ��ֵ��D0Ϊ���ID���룬D1Ϊ0x01д�룬D2Ϊ0x3Bλ�õ�ƫ�ò���ֵ
void setMotorPositionOffset(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5)
{
    CanTxMsg tx_message;
  
    tx_message.StdId = STdId;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = dlc;
    
    tx_message.Data[0] = D0;
    tx_message.Data[1] = D1;
	  tx_message.Data[2] = D2;
    tx_message.Data[3] = D3;
	  tx_message.Data[4] = D4;
    tx_message.Data[5] = D5;
    CAN_Transmit(CAN1,&tx_message);
}

// ��ȡ�������
void readMotorCurrentValue(u8 STdId,u8 dlc,u8 D0,u8 D1)
{

    CanTxMsg tx_message;
  
    tx_message.StdId = STdId;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = dlc;
    
    tx_message.Data[0] = D0;
    tx_message.Data[1] = D1;
    CAN_Transmit(CAN1,&tx_message);
}

// ����������ڷ��͵��̵�����ٶ�Ŀ��ֵ��
void setMotorTargetSpeed(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5)//���ķ���	
{
    CanTxMsg tx_message;
  
    tx_message.StdId = STdId;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = dlc;
    
    tx_message.Data[0] = D0;
    tx_message.Data[1] = D1;
	  tx_message.Data[2] = D2;
    tx_message.Data[3] = D3;
	  tx_message.Data[4] = D4;
    tx_message.Data[5] = D5;
    CAN_Transmit(CAN1,&tx_message);
}



// ���õ��Ŀ�����ֵ
void setMotorTargetCurrent(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5)
{
    CanTxMsg tx_message;
  
    tx_message.StdId = STdId;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = dlc;
    
    tx_message.Data[0] = D0;
    tx_message.Data[1] = D1;
	  tx_message.Data[2] = D2;
    tx_message.Data[3] = D3;
	  tx_message.Data[4] = D4;
    tx_message.Data[5] = D5;
    CAN_Transmit(CAN1,&tx_message);
}

// ���õ��Ŀ����ټ��ٶ�
void setMotorTargetAcspeed(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5)	
{
    CanTxMsg tx_message;
  
    tx_message.StdId = STdId;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = dlc;
    
    tx_message.Data[0] = D0;
    tx_message.Data[1] = D1;
	  tx_message.Data[2] = D2;
    tx_message.Data[3] = D3;
	  tx_message.Data[4] = D4;
    tx_message.Data[5] = D5;
    CAN_Transmit(CAN1,&tx_message);
}

// ���õ��Ŀ����ټ��ٶ�
void setMotorTargetDespeed(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5)	
{
    CanTxMsg tx_message;
  
    tx_message.StdId = STdId;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = dlc;
    
    tx_message.Data[0] = D0;
    tx_message.Data[1] = D1;
	  tx_message.Data[2] = D2;
    tx_message.Data[3] = D3;
	  tx_message.Data[4] = D4;
    tx_message.Data[5] = D5;
    CAN_Transmit(CAN1,&tx_message);
}
