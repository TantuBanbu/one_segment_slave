#include "can2.h"
#include "can1.h"
#include "task.h"
#include "delay.h"
#include "uart1.h"
#include "uart3.h"
//#include "uart6.h"
#include "pwm.h"
#include "imu.h"
#include "gpio.h"
#include "includes.h"
//#include "uart8.h"

#include "SCServo.h" // ���ش��ڶ��
#include "SMS_STS.h"

int continue_flag=0; 




void Gun_task(void *p_arg)
{
	OS_ERR err;
	while(1)
	{  
		
		OSTimeDly(10,OS_OPT_TIME_PERIODIC,&err); //��ʱ10ms	
	}
} 
	




 











//��ʱ��1�Ļص�����
void tmr1_callback(void *p_tmr, void *p_arg)
{
	continue_flag=1;		//��ʱ��1ִ�д�����1
}

void tmr2_callback(void *p_tmr, void *p_arg)
{
//			flag_get_pos=0;  //û�����ݱ�0
//      again_spin=0;
//	    time_again_flag=1;  
}

