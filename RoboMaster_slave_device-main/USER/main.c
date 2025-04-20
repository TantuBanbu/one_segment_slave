#include "delay.h"
#include "task.h"
#include "includes.h"
#include "usart69050.h"

 int main(void){
	 
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	 
	 USART6_init();
	 
//	OS_ERR err;       //定义操作系统错误类型
//	CPU_SR_ALLOC();   //堆栈指针定向
//	delay_init(180);  //时钟初始化
//	
//	OSInit(&err);		//初始化UCOSIII	
//	OS_CRITICAL_ENTER();	//进入临界区（调度器上锁）
//	StartTaskCreate();   //创建开始任务
//	OS_CRITICAL_EXIT();	//退出临界区	 
//	OSStart(&err);  //开启UCOSIII

	while(1); 
}

