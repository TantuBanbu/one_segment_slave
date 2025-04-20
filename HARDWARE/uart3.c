// 添加对IMU相关头文件的引用
#include "can1.h"
#include "task.h"
#include "delay.h"
#include "uart1.h"
#include "uart3.h"
//#include "uart6.h"
#include "pwm.h"
#include "imu.h"
#include "gpio.h"
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "usart69050.h"
#include "usart2.h"
#include "usart7.h"
#include "usart8.h"

// 如果使用ucos操作系统的话需要下面的头文件
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos 使用	  
#endif

// CRC-16 Modbus校验
 const uint16_t crc16_table[256] = {
0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

// 计算CRC-16 Modbus校验
uint16_t calc_crc16_modbus(uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF; // 初始值
    for (uint16_t i = 0; i < length; ++i) {
				uint8_t pos = crc ^ data[i];
        crc = crc16_table[pos] ^ (crc >> 8);
    }
    return crc;
}

// 重定向fputc函数到USART3
int fputc(int ch,FILE *f)
{
	USART_SendData(USART3,ch); // 发送数据
	while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
	return ch; 
	
}



// USART3初始化
void TX2_USART3_Init(void)
{                
		USART_InitTypeDef usart;	// USART初始化结构体
    GPIO_InitTypeDef  gpio;		// GPIO初始化结构体
	  NVIC_InitTypeDef  nvic;		// NVIC初始化结构体
	
		// 使能时钟
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
		// 配置GPIO的GPIO_PinAFConfig
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3); 
		GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3); 
		// GPIO初始化
		gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; 
		gpio.GPIO_Mode = GPIO_Mode_AF;
		gpio.GPIO_Speed = GPIO_Speed_50MHz;	
		gpio.GPIO_OType = GPIO_OType_PP; 
		gpio.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOD,&gpio); 
		// USART初始化
		usart.USART_BaudRate = 115200;//波特率
		usart.USART_WordLength = USART_WordLength_8b;
		usart.USART_StopBits = USART_StopBits_1;
		usart.USART_Parity = USART_Parity_No;
		usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		usart.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;	
		USART_Init(USART3, &usart); 
		// NVIC初始化
		nvic.NVIC_IRQChannel = USART3_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority=1;
		nvic.NVIC_IRQChannelSubPriority =1;		
		nvic.NVIC_IRQChannelCmd = ENABLE;			
		NVIC_Init(&nvic);	 
		// 使能USART3中断
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
		// 使能USART3
		USART_Cmd(USART3,ENABLE);
}


static uint8_t rx_buffer[256]; // 接收缓冲区
static uint16_t rx_index = 0;  // 接收缓冲区索引
static uint16_t expected_length = 0; // 预期数据长度
static uint8_t header_count = 0; // 固定头0xAA55
uint16_t calculated_crc;
uint16_t received_crc;

// 一些全局变量
u8 Receive;

uint8_t controller_address; // 控制器地址

uint8_t function_code;      // 功能码

// 初始化一些信息，每个数据包包含一个地址一个速度
// 共12个数据包（包括蛇形机器人和两个夹爪）
int32_t snake_motor_position_control[12];	// 蛇形机器人位置，每个数据包指一个位置
int32_t snake_motor_speed_control[12];	// 速度
// gripper gm6020 位置
int16_t gripper_gm6020_position_control = 0;
// gripper c610 位置
int16_t gripper_c610_position_control = 0;
// gripper sts3032 位置
int16_t gripper_sts3032_position_control = 0;
int16_t last_sts3032_control_value = 0;
// 状态
int16_t reset_control = 0;
int16_t last_reset_control = 0;
int16_t gripper_gm6020_position_reset_offset = 0;
int16_t gripper_c610_position_reset_offset = 0;
int16_t gripper_sts3032_position_reset_offset = 0;


// 初始化一些信息
uint8_t header_sequence[] = {0xAA, 0x55, 0x01, 0x4B, 0x31, 0x10}; // 固定包头
uint8_t header_match_index = 0; // 接收包头匹配索引


// USART3中断处理函数
void USART3_IRQHandler(void)
{  					
		OSIntEnter();

		// 接收错误标志
		if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)//防止接收错误导致死循环
		{  	
				//清除接收错误标志    		 
				(void)USART3->SR;   
				(void)USART3->DR;
				return;			
		}
		
    // 接收错误标志
    if (USART_GetITStatus(USART3, USART_IT_RXNE)) {
				
        uint8_t byte = USART_ReceiveData(USART3); // 获取接收到的字符
			
			  // 数据头部匹配
				if (byte == header_sequence[header_match_index]) {
						header_match_index++;
						
						// 匹配固定包头
						if (header_match_index == sizeof(header_sequence)) {
								header_match_index = 0; // 重置
								rx_index = 0;
								// 将固定包头复制到rx_buffer
								for (uint8_t i = 0; i < sizeof(header_sequence) - 1; ++i) {
										rx_buffer[rx_index++] = header_sequence[i];
								}
						}
				} else {
						// 接收到的字符不是包头的一部分，重置匹配索引
						header_match_index = (byte == header_sequence[0]) ? 1 : 0;
				}

				
				// 如果已经接收了数据，匹配固定包头， 包头后面一位已经接收完成，需要去掉一位
				if (rx_index >= sizeof(header_sequence) - 1) {

						rx_buffer[rx_index++] = byte; // 将接收到的字符复制到rx_buffer

					// 数据长度为0x AA 55 01之后一位为4B
					expected_length = 0x4B;
						// 接收预接收数据
            if (rx_index == expected_length + 4) { // +4 因为包头2个字节，地址和速度各2个字节
                // 接收数据CRC-16 modbus校验
                calculated_crc = calc_crc16_modbus(rx_buffer, rx_index - 2); // 计算CRC-16校验

                // 获取接收到的CRC-16 modbus校验
                received_crc = (rx_buffer[rx_index - 2] << 8) | rx_buffer[rx_index - 1];

                // 比较校验
                 if (calculated_crc == received_crc) {
                    // 校验成功，获取数据
                    // 数据从第4位开始，长度为(expected_length)
                    // 数据处理
										controller_address = rx_buffer[2]; // 控制器地址
										function_code = rx_buffer[4];      // 功能码
										uint8_t motor_count = rx_buffer[5];        // 电机数量
									
										if(function_code == 0x31){			// 表示接收控制命令		
												// 接收数据
												uint16_t offset = 6; // 数据从第6位开始
												for (uint8_t i = 0; i < 12; ++i) {
														snake_motor_speed_control[i] = rx_buffer[offset++] * 65536 / 60;	// 取速度，将rpm转换为pulse per second
														snake_motor_position_control[i] = (rx_buffer[offset] << 24) | (rx_buffer[offset + 1] << 16) | (rx_buffer[offset + 2] << 8) | rx_buffer[offset + 3];
														offset += 4;
												}
												

												offset += 1;
												gripper_gm6020_position_control = (rx_buffer[offset] << 8) | rx_buffer[offset + 1];
												offset += 3;
												gripper_c610_position_control = (rx_buffer[offset] << 8) | rx_buffer[offset + 1];
												offset += 3;
												last_sts3032_control_value = gripper_sts3032_position_control; // 取上一时刻的值
												gripper_sts3032_position_control = (rx_buffer[offset] << 8) | rx_buffer[offset + 1];
												offset += 3;
												reset_control = rx_buffer[offset++];	
										}
										
										if(reset_control == 1){
													// snake_motors失效和停止设定
													for(uint8_t i=0;i<12;i++){
														uint8_t can_id = i+1;
														motorEnable(can_id,0x06,0x01,0x10,0x00,0x00,0x00,0x00); // 失效
														delay_us(200);														
													}	
													// snake_motors_encorders_offset重置
													for(uint8_t i=0;i<12;i++){
															offsetPosition_snake[i] = currentPosition_snake[i] + offsetPosition_snake[i];													
													}			
														// 去掉偏移量计算功能
//													gripper_gm6020_position_reset_offset = GripperMotor_205_t.position;
//													gripper_c610_position_reset_offset = GripperMotor_201_t.position;
//													GM6020_rotation_count = 0;
//													C610_rotation_count = 0;
//													gripper_sts3032_position_reset_offset = last_sts3032_control_value + gripper_sts3032_position_reset_offset;
																
													// snake_motors有效和停止设定
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
													reset_control = 0;
										}
										
										if(reset_control == 6){
													// snake_motors失效和停止设定
													for(uint8_t i=0;i<12;i++){
														uint8_t can_id = i+1;
														motorEnable(can_id,0x06,0x01,0x10,0x00,0x00,0x00,0x00); // 失效
														delay_us(200);														
													}										
										}
										
										if(reset_control == 9){
													// snake_motors有效和停止设定
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
													reset_control = 0;
										}
																											
                }
								 else 
								{
										// 校验失败，重置接收缓冲区
//										for (uint16_t i = 0; i < 256; ++i) {
//												rx_buffer[i] = 0;
//										}
										// 需要处理的错误
										// ... [需要处理的错误]
										// 重置接收缓冲区
//										header_count = 0;
//										rx_index = 0;
                }

                // 重置接收缓冲区
                rx_index = 0;
            }
				}
				
    }
		
		
    // 清除接收错误标志
    USART_ClearFlag(USART3, USART_FLAG_RXNE);
    USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		OSIntExit();
}


// 数据发送和接收
void TX2_Transmit_Start(void)
{
	USART_ClearFlag(USART3,USART_FLAG_TC);
	USART_SendData(USART3,Receive);
	while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET);//判断是否发送完成

}

/**
 * @brief 发送电机和IMU数据给上位机
 * 数据格式：
 * - 帧头: 0xBB 0x66 (2字节)
 * - 长度: 数据长度(1字节)
 * - 数据:
 *   - 蛇形机器人电机位置(12*4字节)
 *   - 蛇形机器人电机速度(12字节)
 *   - GM6020电机位置(4字节)
 *   - C610电机位置(4字节)
 *   - 四个IMU欧拉角数据(4*3*4字节)
 * - CRC16校验(2字节)
 */
void TX2_Send_Motor_IMU_Data(void)
{
    // 定义发送缓冲区
    #define TX_BUF_SIZE 150
    uint8_t tx_buffer[TX_BUF_SIZE];
    uint16_t offset = 0;
    
    // 添加帧头
    tx_buffer[offset++] = 0xBB;
    tx_buffer[offset++] = 0x66;
    
    // 保留长度字节位置，后面计算
    uint8_t length_pos = offset;
    offset++;
    
    // 添加蛇形机器人电机位置 (12 * 4 = 48字节)
    for(uint8_t i = 0; i < 12; i++)
    {
        tx_buffer[offset++] = (currentPosition_snake[i] >> 24) & 0xFF;
        tx_buffer[offset++] = (currentPosition_snake[i] >> 16) & 0xFF;
        tx_buffer[offset++] = (currentPosition_snake[i] >> 8) & 0xFF;
        tx_buffer[offset++] = currentPosition_snake[i] & 0xFF;
    }
    
    // 添加蛇形机器人电机速度 (12字节)
    for(uint8_t i = 0; i < 12; i++)
    {
        tx_buffer[offset++] = currentSpeed_snake[i] & 0xFF;
    }
    
    // 添加GM6020电机绝对位置 (4字节)
    tx_buffer[offset++] = (GM6020_absolute_position >> 24) & 0xFF;
    tx_buffer[offset++] = (GM6020_absolute_position >> 16) & 0xFF;
    tx_buffer[offset++] = (GM6020_absolute_position >> 8) & 0xFF;
    tx_buffer[offset++] = GM6020_absolute_position & 0xFF;
    
    // 添加C610电机绝对位置 (4字节)
    tx_buffer[offset++] = (C610_absolute_position >> 24) & 0xFF;
    tx_buffer[offset++] = (C610_absolute_position >> 16) & 0xFF;
    tx_buffer[offset++] = (C610_absolute_position >> 8) & 0xFF;
    tx_buffer[offset++] = C610_absolute_position & 0xFF;
    
    // 添加IMU欧拉角数据 (4个IMU * 3轴 * 4字节 = 48字节)
    // IMU6数据
    float tmp_eular[3];
    
    get_eular6(tmp_eular);
    for(uint8_t i = 0; i < 3; i++)
    {
        uint8_t* p = (uint8_t*)&tmp_eular[i];
        tx_buffer[offset++] = p[0];
        tx_buffer[offset++] = p[1];
        tx_buffer[offset++] = p[2];
        tx_buffer[offset++] = p[3];
    }
    
    // IMU2数据
    get_eular2(tmp_eular);
    for(uint8_t i = 0; i < 3; i++)
    {
        uint8_t* p = (uint8_t*)&tmp_eular[i];
        tx_buffer[offset++] = p[0];
        tx_buffer[offset++] = p[1];
        tx_buffer[offset++] = p[2];
        tx_buffer[offset++] = p[3];
    }
    
    // IMU7数据
    get_eular7(tmp_eular);
    for(uint8_t i = 0; i < 3; i++)
    {
        uint8_t* p = (uint8_t*)&tmp_eular[i];
        tx_buffer[offset++] = p[0];
        tx_buffer[offset++] = p[1];
        tx_buffer[offset++] = p[2];
        tx_buffer[offset++] = p[3];
    }
    
    // IMU8数据
    get_eular8(tmp_eular);
    for(uint8_t i = 0; i < 3; i++)
    {
        uint8_t* p = (uint8_t*)&tmp_eular[i];
        tx_buffer[offset++] = p[0];
        tx_buffer[offset++] = p[1];
        tx_buffer[offset++] = p[2];
        tx_buffer[offset++] = p[3];
    }
    
    // 填写数据长度 (不包括帧头、长度字节和CRC)
    tx_buffer[length_pos] = offset - 3;
    
    // 计算CRC
    uint16_t crc = calc_crc16_modbus(tx_buffer, offset);
    tx_buffer[offset++] = (crc >> 8) & 0xFF;  // 高字节
    tx_buffer[offset++] = crc & 0xFF;         // 低字节
    
    // 发送数据
    for(uint16_t i = 0; i < offset; i++)
    {
        USART_ClearFlag(USART3, USART_FLAG_TC);
        USART_SendData(USART3, tx_buffer[i]);
        while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
    }
}






////时2的回调函数
//void tmr2_callback(void *p_tmr, void *p_arg)
//{
////			flag_get_pos=0;  //û接收数据为0
////      again_spin=0;
////	    time_again_flag=1;  
//}
