#ifndef __UART8_H
#define __UART8_H	 
#include "sys.h"  
#include "stdio.h"

#include <stdint.h>
#include <stdbool.h>

#define UART8_MAX_RECV_LEN		256				//�����ջ����ֽ���
#define UART8_MAX_SEND_LEN		256				//����ͻ����ֽ���
#define UART8_RX_EN 			1				      //0,������;1,����.

extern u8  UART8_RX_BUF[UART8_MAX_RECV_LEN]; 		
extern u8  UART8_TX_BUF[UART8_MAX_SEND_LEN]; 		
extern vu16 UART8_RX_STA;   		//��������״̬


void UART8_init(void);
void DATA_IMU(u8 Res);
void UART8_Send(unsigned char *DataToSend ,u8 data_num);


int imu_data_decode_init(void);
int get_raw_acc(int16_t* a);
int get_raw_gyo(int16_t* g);
int get_raw_mag(int16_t* m);
int get_id(uint8_t *user_id);
int get_eular(float* e);
int get_quat(float* q);

extern int get_eular8(float* e);

#endif



