#ifndef __USART2_H
#define __USART2_H	 
#include "sys.h"  
#include "stdio.h"

#include <stdint.h>
#include <stdbool.h>

#define USART2_MAX_RECV_LEN		256				//�����ջ����ֽ���
#define USART2_MAX_SEND_LEN		256				//����ͻ����ֽ���
#define USART2_RX_EN 			1				      //0,������;1,����.

extern u8  USART2_RX_BUF[USART2_MAX_RECV_LEN]; 		
extern u8  USART2_TX_BUF[USART2_MAX_SEND_LEN]; 		
extern vu16 USART2_RX_STA;   		//��������״̬


void USART2_init(void);
void DATA_IMU(u8 Res);
void USART2_Send(unsigned char *DataToSend ,u8 data_num);


int imu_data_decode_init(void);
int get_raw_acc(int16_t* a);
int get_raw_gyo(int16_t* g);
int get_raw_mag(int16_t* m);
int get_id(uint8_t *user_id);
int get_eular(float* e);
int get_quat(float* q);

// 添加 get_eular2 的声明
extern int get_eular2(float* e);

#endif



