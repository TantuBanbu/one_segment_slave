#include "delay.h"
#include "task.h"
#include "includes.h"
#include "usart69050.h"

 int main(void){
	 
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	 
	 USART6_init();
	 
//	OS_ERR err;       //�������ϵͳ��������
//	CPU_SR_ALLOC();   //��ջָ�붨��
//	delay_init(180);  //ʱ�ӳ�ʼ��
//	
//	OSInit(&err);		//��ʼ��UCOSIII	
//	OS_CRITICAL_ENTER();	//�����ٽ�����������������
//	StartTaskCreate();   //������ʼ����
//	OS_CRITICAL_EXIT();	//�˳��ٽ���	 
//	OSStart(&err);  //����UCOSIII

	while(1); 
}

