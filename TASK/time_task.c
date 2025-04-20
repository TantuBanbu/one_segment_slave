#include "can1.h"
#include "task.h"
#include "delay.h"
#include "uart3.h"
#include "includes.h"

//#include "uart1.h"
//#include "uart2.h"
//#include "pwm.h"
//#include "imu.h"
//#include "gpio.h"
//extern float pos_x;//��λ��������ƫ��
//extern float pos_y;//��ά�������±���
//extern float pos_z;//��ά����Զ��ƫ��
////////////u8 time_again_flag=1;    //����2�ص������ı�־λ
//////////////�ص�������־λ��ֵΪ1�����ִ��һ�λص������ͱ��1
//////////////��ʼ��ʱ��ʱ����0 ��ʱ������ִ�лص��������1
//extern u8 again_spin;    //������ת���ź�
//extern u8 flag_get_pos;

// 定义数据发送周期(ms)
#define MOTOR_IMU_DATA_SEND_PERIOD 50

/**
 * @brief 时间相关任务
 * 
 * 1. 定期向上位机发送电机和IMU数据
 */
void Time_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	
	// 初始化计数器
	uint32_t send_counter = 0;
	
	while(1)
	{
		// 增加时间计数器
		time_counter++;
		
		// 每隔MOTOR_IMU_DATA_SEND_PERIOD毫秒发送一次电机和IMU数据
		if(++send_counter >= MOTOR_IMU_DATA_SEND_PERIOD/10)
		{
			// 发送电机和IMU数据给上位机
			TX2_Send_Motor_IMU_Data();
			send_counter = 0;
		}
		
		// 任务周期为10ms
		OSTimeDly(10, OS_OPT_TIME_PERIODIC, &err);
	}
}
