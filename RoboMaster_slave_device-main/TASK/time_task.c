//#include "can1.h"
//#include "task.h"
//#include "delay.h"
//#include "uart1.h"
//#include "uart2.h"
//#include "uart3.h"
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

//void Time_task(void *p_arg)
//{
//	OS_ERR err;
//	p_arg = p_arg;
//	
//while(1)
//{
//			USART_SendData(USART3,again_spin);//��̨����̷�������  0Ϊ���̶� ��̨��ת 1Ϊȫ��ֹͣ

//        if(pos_x==0.0f&&pos_y==0.0f&&pos_z==0.0f)
//				{	
//           again_spin=0;     //������ת�ı�־λ �յ���Ч���ݵı�־λ
//					 flag_get_pos=0;  //û�����ݱ�0     �����������Ժ�ı�־λ
//				}				

//		OSTimeDly(60,OS_OPT_TIME_PERIODIC,&err); 
//	}
//}
