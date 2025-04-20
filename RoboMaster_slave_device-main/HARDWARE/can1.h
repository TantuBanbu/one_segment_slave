#ifndef __CAN1_H__
#define __CAN1_H__

#include <stm32f4xx.h>

void CAN1_Init(void);
void CAN1_TX_IRQHandler(void);
void CAN1_RX0_IRQHandler(void);

void Chasis_ESC_Send(int16_t current_201,int16_t current_202,int16_t current_203,int16_t current_204);

void motorEnable(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5);
void setMotorTargetSpeed(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5);
void setMotorTargetPosition(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5);
void setMotorTargetCurrent(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5);
void setMotorTargetAcspeed(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5);
void setMotorTargetDespeed(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5);
void readMotorCurrentValue(u8 STdId,u8 dlc,u8 D0,u8 D1);
void setMotorPositionOffset(u8 STdId,u8 dlc,u8 D0,u8 D1,u8 D2,u8 D3,u8 D4,u8 D5);

void readSnakeEncorder(u8 STdId,u8 dlc,u8 D0,u8 D1); //��ȡ��������ı�����ֵ
void readSnakeSpeed(u8 STdId,u8 dlc,u8 D0,u8 D1); // ��ȡ��������ٶ�ֵ	

void GM6020_Can_Send_Msg(int16_t current_1,int16_t current_2,int16_t current_3,int16_t current_4); // GM6020�ؽڵ������
void C610_Can_Send_Msg(int16_t current_1,int16_t current_2,int16_t current_3,int16_t current_4); // C610�ؽڵ������


typedef struct
{
	int16_t position;
	int16_t velocity;
	int16_t current;
	int8_t  temperature;
}GripperMotor_ID_t;


extern GripperMotor_ID_t GripperMotor_205_t; // �ֲ���GM6020���
extern GripperMotor_ID_t GripperMotor_201_t; // �ֲ���C610���
extern int ENABLE_yes;
extern int flag;
extern int16_t receive[4];

extern int32_t round_6020_yaw;
extern float phase_6020_yaw;
extern int32_t currentPosition_snake[12];
extern int32_t offsetPosition_snake[12];
extern int8_t currentSpeed_snake[12];

extern int32_t GM6020_absolute_position;  // ����λ��
extern int16_t GM6020_last_raw_position;  // ��һ�ε�ԭʼλ�ã�0-8191��
extern int16_t GM6020_current_raw_position;  // ��ǰ��ԭʼλ�ã�0-8191��
extern int32_t GM6020_rotation_count;  // ��ת������ÿ���һȦ����1�����1��

extern int32_t C610_absolute_position;  // ����λ��
extern int16_t C610_last_raw_position;  // ��һ�ε�ԭʼλ�ã�0-8191��
extern int16_t C610_current_raw_position;  // ��ǰ��ԭʼλ�ã�0-8191��
extern int32_t C610_rotation_count;
extern int32_t time_counter;

#endif
