#include "usart69050.h"	
#include "sys.h"	
#include "hipnuc_dec.h"
#include <string.h>
#include <stdio.h>
#include "includes.h"   // uC/OS-II 相关头文件

// DMA接收缓冲区
#define USART6_RX_BUF_SIZE 256
static uint8_t USART6_RX_BUF[USART6_RX_BUF_SIZE];
// IMU解析结构体
static hipnuc_raw_t imu6_raw;
// 欧拉角数据
float eular6[3]; // 分别对应pitch, roll, yaw

/**
 * @brief 获取欧拉角数据
 * 
 * @param e 存储欧拉角的数组：pitch, roll, yaw
 * @return int 0表示成功
 */
int get_eular6(float* e)
{
    memcpy(e, eular6, sizeof(eular6));
    return 0;
}

/**
 * @brief USART6初始化函数
 */
void USART6_init(void){
    // GPIO端口设置
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
	 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); // 使能DMA2时钟
	
    // 串口6对应引脚复用映射
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6); 
    GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_USART6); 
	
    // USART6_TX:PG9  USART6_RX:PG14
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOG, &GPIO_InitStructure);

    // DMA2_Stream1通道5,USART6_RX
    DMA_DeInit(DMA2_Stream1);
    DMA_InitStructure.DMA_Channel = DMA_Channel_5;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART6->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)USART6_RX_BUF;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = USART6_RX_BUF_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream1, &DMA_InitStructure);
    DMA_Cmd(DMA2_Stream1, ENABLE);

    // USART NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
    NVIC_Init(&NVIC_InitStructure);	

    // USART 初始化设置
    USART_DeInit(USART6);
    USART_InitStructure.USART_BaudRate = 115200; // 波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; // 字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1; // 一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No; // 无奇偶校验位
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // 收发模式
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件数据流控制
    USART_Init(USART6, &USART_InitStructure);

    // 配置DMA接收
    USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
     
    // 使能串口接收中断
    USART_ITConfig(USART6, USART_IT_IDLE, ENABLE); // 使能空闲中断
    
    // 使能串口
    USART_Cmd(USART6, ENABLE);
    
    // 初始化HiPNUC解析结构体
    memset(&imu6_raw, 0, sizeof(hipnuc_raw_t));
}

/**
 * @brief USART6中断服务函数
 */
void USART6_IRQHandler(void)
{
    OSIntEnter();
    
    // 判断是否是空闲中断
    if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
    {
        // 清除空闲中断标志
        (void)USART6->SR;   
        (void)USART6->DR;
        
        // 获取DMA接收的数据长度
        uint16_t recv_size = USART6_RX_BUF_SIZE - DMA_GetCurrDataCounter(DMA2_Stream1);
        
        // 处理接收到的数据
        for(uint16_t i = 0; i < recv_size; i++)
        {
            // 使用HiPNUC解析库解析数据
            if(hipnuc_input(&imu6_raw, USART6_RX_BUF[i]) > 0)
            {
                // 成功解析到一个数据包，提取欧拉角数据
                if(imu6_raw.hi91.tag == 0x91)
                {
                    eular6[0] = imu6_raw.hi91.pitch; // pitch
                    eular6[1] = imu6_raw.hi91.roll;  // roll
                    eular6[2] = imu6_raw.hi91.yaw;   // yaw
                }
            }
        }
        
        // 重新设置DMA
        DMA_Cmd(DMA2_Stream1, DISABLE);
        DMA_SetCurrDataCounter(DMA2_Stream1, USART6_RX_BUF_SIZE);
        DMA_Cmd(DMA2_Stream1, ENABLE);
    }
    
    // 处理溢出错误
    if(USART_GetFlagStatus(USART6, USART_FLAG_ORE) != RESET)
    {
        (void)USART6->SR;   
        (void)USART6->DR;
    }
    
    OSIntExit();
}
