#ifndef __USART6_H
#define __USART6_H	 
#include "sys.h"  
#include "stdio.h"

#define USART6_MAX_RECV_LEN		256				//�����ջ����ֽ���
#define USART6_MAX_SEND_LEN		256				//����ͻ����ֽ���
#define USART6_RX_EN 			1				      //0,������;1,����.

extern u8  USART6_RX_BUF[USART6_MAX_RECV_LEN]; 		
extern u8  USART6_TX_BUF[USART6_MAX_SEND_LEN]; 		
extern vu16 USART6_RX_STA;   		//��������״̬


void USART6_init(void);
void DATA_IMU(u8 Res);
void Usart6_Send(unsigned char *DataToSend ,u8 data_num);
#endif

