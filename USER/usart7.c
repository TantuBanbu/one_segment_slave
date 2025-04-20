#include "usart7.h"	
#include "stm32f4xx.h" // 包含核心 STM32F4 设备头文件
#include "sys.h"	
#include "hipnuc_dec.h"
#include <string.h>
#include <stdio.h>
#include "includes.h"   // uC/OS-II 核心头文件

// DMA接收缓冲区
#define UART7_RX_BUF_SIZE 256
static uint8_t UART7_RX_BUF[UART7_RX_BUF_SIZE];
// IMU解析结构体
static hipnuc_raw_t imu7_raw;
// 欧拉角数据
float eular7[3]; // 分别对应pitch, roll, yaw

/**
 * @brief 获取欧拉角数据
 * 
 * @param e 存储欧拉角的数组：pitch, roll, yaw
 * @return int 0表示成功
 */
int get_eular7(float* e)
{
    memcpy(e, eular7, sizeof(eular7));
    return 0;
}

/**
 * @brief UART7初始化函数
 */
void UART7_init(void){
    // GPIO端口设置
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); // 正确：使能 GPIOE 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7, ENABLE);    // 标准的使能 UART7 时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE); // 使能DMA1时钟

    GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_UART7); // 正确：配置 PE7 为 UART7 TX
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_UART7); // 正确：配置 PE8 为 UART7 RX
    
    //初始 GPIOE7 (TX) PE8 (RX)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8; // PE7 和 PE8
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	//
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &GPIO_InitStructure); // 正确：初始化 GPIOE

    // DMA1_Stream3通道5,UART7_RX
    DMA_DeInit(DMA1_Stream3);
    DMA_InitStructure.DMA_Channel = DMA_Channel_5;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART7->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)UART7_RX_BUF;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = UART7_RX_BUF_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream3, &DMA_InitStructure);
    DMA_Cmd(DMA1_Stream3, ENABLE);

    // NVIC 优先级
    NVIC_InitStructure.NVIC_IRQChannel = UART7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // 示例优先级，根据 uC/OS-III 配置调整
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
    NVIC_Init(&NVIC_InitStructure);	

    //USART 初始化设置
    USART_DeInit(UART7);
    USART_InitStructure.USART_BaudRate = 115200;//波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件流控制
    USART_Init(UART7,&USART_InitStructure);
    
    // 配置DMA接收
    USART_DMACmd(UART7, USART_DMAReq_Rx, ENABLE);
     
    // 使能串口接收中断
    USART_ITConfig(UART7, USART_IT_IDLE, ENABLE); // 使能空闲中断
    
    // 使能串口
    USART_Cmd(UART7, ENABLE);
    
    // 初始化HiPNUC解析结构体
    memset(&imu7_raw, 0, sizeof(hipnuc_raw_t));
}

/**
 * @brief UART7中断服务函数
 */
void UART7_IRQHandler(void)
{
	OSIntEnter();

	// 判断是否是空闲中断
	if(USART_GetITStatus(UART7, USART_IT_IDLE) != RESET)
	{
		// 清除空闲中断标志
		(void)UART7->SR;   
		(void)UART7->DR;
		
		// 获取DMA接收的数据长度
		uint16_t recv_size = UART7_RX_BUF_SIZE - DMA_GetCurrDataCounter(DMA1_Stream3);
		
		// 处理接收到的数据
		for(uint16_t i = 0; i < recv_size; i++)
		{
			// 使用HiPNUC解析库解析数据
			if(hipnuc_input(&imu7_raw, UART7_RX_BUF[i]) > 0)
			{
				// 成功解析到一个数据包，提取欧拉角数据
				if(imu7_raw.hi91.tag == 0x91)
				{
					eular7[0] = imu7_raw.hi91.pitch; // pitch
					eular7[1] = imu7_raw.hi91.roll;  // roll
					eular7[2] = imu7_raw.hi91.yaw;   // yaw
				}
			}
		}
		
		// 重新设置DMA
		DMA_Cmd(DMA1_Stream3, DISABLE);
		DMA_SetCurrDataCounter(DMA1_Stream3, UART7_RX_BUF_SIZE);
		DMA_Cmd(DMA1_Stream3, ENABLE);
	}
	
	// 处理溢出错误
	if(USART_GetFlagStatus(UART7, USART_FLAG_ORE) != RESET)
	{
		(void)UART7->SR;   
		(void)UART7->DR;
	}
	
	OSIntExit();
}
