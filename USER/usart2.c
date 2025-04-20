#include "usart2.h"	
#include "sys.h"	
#include "hipnuc_dec.h"
#include <string.h>
#include <stdio.h>
#include "includes.h"   // uC/OS-II 头文件
#include "stm32f4xx.h"

// DMA接收缓冲区
#define USART2_RX_BUF_SIZE 256
static uint8_t USART2_RX_BUF[USART2_RX_BUF_SIZE];
// IMU解析结构体
static hipnuc_raw_t imu2_raw;
// 欧拉角数据
float eular2[3]; // 分别对应pitch, roll, yaw

/**
 * @brief 获取欧拉角数据
 * 
 * @param e 存储欧拉角的数组：pitch, roll, yaw
 * @return int 0表示成功
 */
int get_eular2(float* e)
{
    memcpy(e, eular2, sizeof(eular2));
    return 0;
}

/**
 * @brief USART2初始化函数
 */
void USART2_init(void){
    // GPIO端口设置
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); // 使能 GPIOD 时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); // 使能 USART2 时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE); // 使能DMA1时钟

    // 串口2对应引脚复用映射
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2); // PD5 复用为 USART2 TX
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2); // PD6 复用为 USART2 RX

    // USART2端口配置 PD5(TX), PD6(RX)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6; //选择 PD5 和 PD6
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //复用模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
    GPIO_Init(GPIOD, &GPIO_InitStructure); //初始化 GPIOD

    // DMA1_Stream5通道4,USART2_RX
    DMA_DeInit(DMA1_Stream5);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)USART2_RX_BUF;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = USART2_RX_BUF_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream5, &DMA_InitStructure);
    DMA_Cmd(DMA1_Stream5, ENABLE);

    // Usart NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // USART 初始化设置
    USART_DeInit(USART2); // 先反初始化
    USART_InitStructure.USART_BaudRate = 115200; //波特率
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; //字长为8位
    USART_InitStructure.USART_StopBits = USART_StopBits_1; //一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No; //无奇偶校验位
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; //收发模式
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
    USART_Init(USART2, &USART_InitStructure); //初始化USART2

    // 配置DMA接收
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
     
    // 使能串口接收中断
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE); // 使能空闲中断
    
    // 使能串口
    USART_Cmd(USART2, ENABLE);
    
    // 初始化HiPNUC解析结构体
    memset(&imu2_raw, 0, sizeof(hipnuc_raw_t));
}

/**
 * @brief USART2中断服务函数
 */
void USART2_IRQHandler(void)
{
    OSIntEnter();
    
    // 判断是否是空闲中断
    if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
        // 清除空闲中断标志
        (void)USART2->SR;   
        (void)USART2->DR;
        
        // 获取DMA接收的数据长度
        uint16_t recv_size = USART2_RX_BUF_SIZE - DMA_GetCurrDataCounter(DMA1_Stream5);
        
        // 处理接收到的数据
        for(uint16_t i = 0; i < recv_size; i++)
        {
            // 使用HiPNUC解析库解析数据
            if(hipnuc_input(&imu2_raw, USART2_RX_BUF[i]) > 0)
            {
                // 成功解析到一个数据包，提取欧拉角数据
                if(imu2_raw.hi91.tag == 0x91)
                {
                    eular2[0] = imu2_raw.hi91.pitch; // pitch
                    eular2[1] = imu2_raw.hi91.roll;  // roll
                    eular2[2] = imu2_raw.hi91.yaw;   // yaw
                }
            }
        }
        
        // 重新设置DMA
        DMA_Cmd(DMA1_Stream5, DISABLE);
        DMA_SetCurrDataCounter(DMA1_Stream5, USART2_RX_BUF_SIZE);
        DMA_Cmd(DMA1_Stream5, ENABLE);
    }
    
    // 处理溢出错误
    if(USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET)
    {
        (void)USART2->SR;   
        (void)USART2->DR;
    }
    
    OSIntExit();
}
