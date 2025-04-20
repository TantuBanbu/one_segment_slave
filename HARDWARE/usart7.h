#ifndef __UART7_H
#define __UART7_H	 
#include "sys.h"  
#include "stdio.h"

#include <stdint.h>
#include <stdbool.h>

#define UART7_MAX_RECV_LEN		256				//最大接收缓存字节数
#define UART7_MAX_SEND_LEN		256				//最大发送缓存字节数
#define UART7_RX_EN 			1				      //0,不接收;1,接收.

extern u8  UART7_RX_BUF[UART7_MAX_RECV_LEN]; 		
extern u8  UART7_TX_BUF[UART7_MAX_SEND_LEN]; 		
extern vu16 UART7_RX_STA;   		//接收数据状态


void UART7_init(void);
void DATA_IMU(u8 Res);
void UART7_Send(unsigned char *DataToSend ,u8 data_num);


int imu_data_decode_init(void);
int get_raw_acc(int16_t* a);
int get_raw_gyo(int16_t* g);
int get_raw_mag(int16_t* m);
int get_id(uint8_t *user_id);
int get_eular(float* e);
int get_quat(float* q);
#endif



