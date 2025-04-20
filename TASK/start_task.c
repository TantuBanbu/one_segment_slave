#include "can1.h"
#include "can2.h"
#include "task.h"
#include "delay.h"
#include "uart1.h"
#include "uart3.h"
#include "pwm.h"
//#include "imu.h"
#include "gpio.h"
//#include "uart8.h"
#include "pstwo.h"
#include "includes.h"
#include "usart69050.h"
 
//�������ȼ�
#define START_TASK_PRIO		3
//�����ջ��С	
#define START_STK_SIZE 		128
//������ƿ�
OS_TCB StartTaskTCB;
//�����ջ	
CPU_STK START_TASK_STK[START_STK_SIZE];

#define INIT_TASK_PRIO		4
#define INIT_STK_SIZE 		128
OS_TCB InitTaskTCB;
CPU_STK INIT_TASK_STK[INIT_STK_SIZE];

#if EN_CHASIS_TASK
#define CHASIS_TASK_PRIO		6
#define CHASIS_STK_SIZE 		512
OS_TCB CHASISTaskTCB;
CPU_STK CHASIS_TASK_STK[CHASIS_STK_SIZE];
#endif

#if EN_GIMBAL_TASK
#define GIMBAL_TASK_PRIO		5
#define GIMBAL_STK_SIZE 		512
OS_TCB GIMBALTaskTCB;
CPU_STK GIMBAL_TASK_STK[GIMBAL_STK_SIZE];
#endif


#if EN_GUN_TASK
#define GUN_TASK_PRIO		6
#define GUN_STK_SIZE 		128
OS_TCB GUNTaskTCB;
CPU_STK GUN_TASK_STK[GUN_STK_SIZE];
#endif

#if EN_TIME_TASK
#define TIME_TASK_PRIO		6
#define TIME_STK_SIZE 		128
OS_TCB TIMETaskTCB;
CPU_STK TIME_TASK_STK[TIME_STK_SIZE];
#endif

OS_TMR tmr1;    //��ʱ��1
OS_TMR tmr2;    //��ʱ��2

void tmr1_callback(void *p_tmr, void *p_arg); 	//��ʱ��1�ص�����
void tmr2_callback(void *p_tmr, void *p_arg); 	//��ʱ��2�ص�����

//��ʼ������
void start_task(void *p_arg)
	{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//ͳ������                
#endif
		delay_ms(3000);
	
	//������ʱ��1
	OSTmrCreate((OS_TMR		*)&tmr1,		//��ʱ��1
                (CPU_CHAR	*)"tmr1",		//��ʱ������
                (OS_TICK	 )50,			//50*10=500ms
                (OS_TICK	 )0,         
                (OS_OPT		 )OS_OPT_TMR_ONE_SHOT, //���ζ�ʱ��
                (OS_TMR_CALLBACK_PTR)tmr1_callback,//��ʱ��1�ص�����
                (void	    *)0,			//����Ϊ0
                (OS_ERR	    *)&err);		//���صĴ�����					
  //������ʱ��2
	OSTmrCreate((OS_TMR		*)&tmr2,		//��ʱ��1
                (CPU_CHAR	*)"tmr2",		//��ʱ������
                (OS_TICK	 )10,			//10*10=100ms
                (OS_TICK	 )0,         
                (OS_OPT		 )OS_OPT_TMR_ONE_SHOT, //���ζ�ʱ��
                (OS_TMR_CALLBACK_PTR)tmr2_callback,//��ʱ��1�ص�����
                (void	    *)0,			//����Ϊ0
                (OS_ERR	    *)&err);		//���صĴ�����			
	OS_CRITICAL_ENTER();	//�����ٽ���					
//������ʼ������
	OSTaskCreate(  (OS_TCB 	* )&InitTaskTCB,		
				         (CPU_CHAR	* )"Init task", 		
                 (OS_TASK_PTR )Init_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )INIT_TASK_PRIO,     	
                 (CPU_STK   * )&INIT_TASK_STK[0],	
                 (CPU_STK_SIZE)INIT_STK_SIZE/10,	
                 (CPU_STK_SIZE)INIT_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);		
									
#if EN_CHASIS_TASK
	OSTaskCreate(  (OS_TCB 	* )&CHASISTaskTCB,		
				         (CPU_CHAR	* )"Chasis task", 		
                 (OS_TASK_PTR )Chasis_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )CHASIS_TASK_PRIO,     	
                 (CPU_STK   * )&CHASIS_TASK_STK[0],	
                 (CPU_STK_SIZE)CHASIS_STK_SIZE/10,	
                 (CPU_STK_SIZE)CHASIS_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);			
#endif					
								 
#if EN_GIMBAL_TASK
	OSTaskCreate(  (OS_TCB 	* )&GIMBALTaskTCB,		
				         (CPU_CHAR	* )"Gimbal task", 		
                 (OS_TASK_PTR )Gimbal_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )GIMBAL_TASK_PRIO,     	
                 (CPU_STK   * )&GIMBAL_TASK_STK[0],	
                 (CPU_STK_SIZE)GIMBAL_STK_SIZE/10,	
                 (CPU_STK_SIZE)GIMBAL_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);			
#endif								 
										 
								 
#if EN_GUN_TASK
	OSTaskCreate(  (OS_TCB 	* )&GUNTaskTCB,		
				         (CPU_CHAR	* )"Gun task", 		
                 (OS_TASK_PTR )Gun_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )GUN_TASK_PRIO,     	
                 (CPU_STK   * )&GUN_TASK_STK[0],	
                 (CPU_STK_SIZE)GUN_STK_SIZE/10,	
                 (CPU_STK_SIZE)GUN_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);			
#endif								 
	
#if EN_TIME_TASK
	OSTaskCreate(  (OS_TCB 	* )&TIMETaskTCB,		
				         (CPU_CHAR	* )"Time task", 		
                 (OS_TASK_PTR )Time_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )TIME_TASK_PRIO,     	
                 (CPU_STK   * )&TIME_TASK_STK[0],	
                 (CPU_STK_SIZE)TIME_STK_SIZE/10,	
                 (CPU_STK_SIZE)TIME_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
                 (OS_ERR 	* )&err);			
#endif						
								 
#if EN_CHASIS_TASK								 
		OS_TaskSuspend((OS_TCB*)&CHASISTaskTCB,&err);       //								 
#endif
		OS_TaskSuspend((OS_TCB*)&GIMBALTaskTCB,&err);       //̨
    OS_TaskSuspend((OS_TCB*)&GUNTaskTCB,&err);          //�����������
//	  OS_TaskSuspend((OS_TCB*)&TIMETaskTCB,&err);          //����ʱ������
		OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);		    //����ʼ����			
								 
	OS_CRITICAL_EXIT();	//�Ƴ��ٽ���   
}

void StartTaskCreate(void){
	OS_ERR err;
	OSTaskCreate(  (OS_TCB 	* )&StartTaskTCB,		//������ƿ�
								 (CPU_CHAR * )"start task", 		//��������
                 (OS_TASK_PTR )start_task, 			//������
                 (void		* )0,					//���ݸ��������Ĳ���
                 (OS_PRIO	  )START_TASK_PRIO,     //�������ȼ�
                 (CPU_STK  * )&START_TASK_STK[0],	//�����ջ����ַ
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//�����ջ�����λ
                 (CPU_STK_SIZE)START_STK_SIZE,		//�����ջ��С
                 (OS_MSG_QTY)0,					//�����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ,Ϊ0ʱ��ֹ������Ϣ
                 (OS_TICK	  )0,					//��ʹ��ʱ��Ƭ��תʱ��ʱ��Ƭ���ȣ�Ϊ0ʱΪĬ�ϳ��ȣ�
                 (void   	* )0,					//�û�����Ĵ洢��
                 (OS_OPT    )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //����ѡ��
                 (OS_ERR 	* )&err);				//��Ÿú�������ʱ�ķ���ֵ
}

void Init_task(void *p_arg){
		OS_ERR err;
		p_arg = p_arg;
	 
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�жϷ�������
  	User_GPIO_Init();
		
	
		time_counter = 0;
		CAN1_Init();                //CAN��������
	  CAN2_Init();                //CAN2��������
		TX2_USART3_Init();       //TX2ͨ��
		
//	  USART6_init(); //��ʼ������6
//		uart8_init(); //��ʼ������8


		
	  TIM2_PWM_Init();
	  TIM4_PWM_Init();
	  TIM5_PWM_Init();
	  TIM8_PWM_Init();
//		//Heat_PWM_Init();

		// snake_motorsʹ�ܺͲ����趨
		for(uint8_t i=0;i<12;i++){
			uint8_t can_id = i+1;
			motorEnable(can_id,0x06,0x01,0x10,0x00,0x00,0x00,0x01);
			delay_us(200);
			setMotorTargetCurrent(can_id,0x06,0x01,0x08,0x00,0x00,0x07,0xd0);
			delay_us(200);
			setMotorTargetSpeed(can_id,0x06,0x01,0x09,0x00,0x00,0x2a,0xaa);
			delay_us(200);
			setMotorTargetAcspeed(can_id,0x06,0x01,0x0B,0x00,0x10,0xAA,0xAA);
			delay_us(200);
			setMotorTargetDespeed(can_id,0x06,0x01,0x0C,0x00,0x10,0xAA,0xAA);		
			delay_us(200);
		}	

		
		
		// ��ʼ��GM6020 pid���Ƶ����õĵ�ǰλ�ò���
		GM6020_last_raw_position = GripperMotor_205_t.position;  // ��һ�ε�ԭʼλ�ã�0-8191��
		GM6020_current_raw_position = GripperMotor_205_t.position;  // ��ǰ��ԭʼλ�ã�0-8191��
		gripper_gm6020_position_control = GM6020_last_raw_position; // ȥ���ϵ��Ķ���
		// ��ʼ��C610 pid���Ƶ����õĵ�ǰλ�ò���
		C610_last_raw_position = GripperMotor_201_t.position;  // ��һ�ε�ԭʼλ�ã�0-8191��
		C610_current_raw_position = GripperMotor_201_t.position;  // ��ǰ��ԭʼλ�ã�0-8191��
  	
	

#if EN_CHASIS_TASK
		OS_TaskResume((OS_TCB*)&CHASISTaskTCB,&err);
#endif
	  OS_TaskResume((OS_TCB*)&GIMBALTaskTCB,&err);     //̨
//		OS_TaskResume((OS_TCB*)&GUNTaskTCB,&err);
//		OS_TaskResume((OS_TCB*)&TIMETaskTCB,&err);
// 
	  OS_TaskSuspend((OS_TCB*)&InitTaskTCB,&err);		   //		
}

