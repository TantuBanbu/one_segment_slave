#include "can1.h"
#include "can2.h"
#include "uart1.h"
#include "uart3.h"
#include "task.h"
#include "delay.h"
//#include "uart8.h"
#include "gpio.h"
//#include "pstwo.h"
#include "pwm.h"
#include "includes.h"
#include "usart69050.h"
#include "usart2.h"
#include "usart7.h"
#include "usart8.h"

int16_t Adc_Volt,limit_shift; //canͨѶ����



void TX2_USART3_Init(void);


void Chasis_task(void *p_arg)
{
		OS_ERR err;
		p_arg = p_arg; 
		for(int i = 0; i < 12; i++){
			snake_motor_position_control[i] = 0;
		}
		delay_us(200);
		while(1)
	{  		
			if(time_counter < 201){
				time_counter = time_counter +1;	
			}
			// send commands to 12 CAN module to control the position of snake motors
			// ����can���ߵĵ��λ�ÿ���ָ��
			for(uint8_t i=0; i < 12; i++){
				uint8_t bytes[4];  // ���ڴ洢4���ֽڵ�����
				uint8_t can_id = i + 1;

				if(reset_control != 1){
					motorEnable(can_id,0x06,0x01,0x0F,0x00,0x00,0x00,0x01); // ����ģʽΪ1
					// �ֽ� int32_t ����Ϊ4���ֽ�
					bytes[0] = (snake_motor_speed_control[i] >> 24) & 0xFF;  // �����Ч�ֽ� (MSB)
					bytes[1] = (snake_motor_speed_control[i] >> 16) & 0xFF;  // �θ���Ч�ֽ�
					bytes[2] = (snake_motor_speed_control[i] >> 8) & 0xFF;   // �ε���Ч�ֽ�
					bytes[3] = (snake_motor_speed_control[i]) & 0xFF;          // �����Ч�ֽ� (LSB)
					if(time_counter >= 100){
						setMotorTargetSpeed(can_id,0x06,0x01,0x09,bytes[0],bytes[1],bytes[2],bytes[3]); //�趨Ŀ��λ��ֵ��32λ�з�������
					}
					delay_us(200);
					// �ֽ� int32_t ����Ϊ4���ֽ�
					bytes[0] = ((snake_motor_position_control[i] + offsetPosition_snake[i]) >> 24) & 0xFF;  // �����Ч�ֽ� (MSB)
					bytes[1] = ((snake_motor_position_control[i] + offsetPosition_snake[i]) >> 16) & 0xFF;  // �θ���Ч�ֽ�
					bytes[2] = ((snake_motor_position_control[i] + offsetPosition_snake[i]) >> 8) & 0xFF;   // �ε���Ч�ֽ�
					bytes[3] = (snake_motor_position_control[i] + offsetPosition_snake[i]) & 0xFF;          // �����Ч�ֽ� (LSB)
					if(time_counter >= 100){
						setMotorTargetPosition(can_id,0x06,0x01,0x0A,bytes[0],bytes[1],bytes[2],bytes[3]); //�趨Ŀ��λ��ֵ��32λ�з�������
					}
					delay_us(200);													
				}

				delay_us(200);
				// reading the encorder
				readSnakeEncorder(can_id,0x02,0x03,0x07);
				
				delay_us(200);
				// ��ȡ�ٶ�ֵ
				readSnakeSpeed(can_id,0x02,0x03,0x06);
			}
			
			/************************************************************* 
			  Send motor and IMU data to the master device via UART3
			  (Replaces the previous UART sending logic in this task)
			***************************************************************/
			{ // Add scope for local variables
				uint8_t tx_buffer[256]; // 发送缓冲区 (Use local buffer)
				uint16_t tx_index = 0;  // 发送缓冲区索引
				uint16_t crc16; // CRC16校验值
				float temp_eular[3]; // 临时存储欧拉角数据

				// 1. 添加帧头 (0xAA 0x55)
				tx_buffer[tx_index++] = 0xAA;
				tx_buffer[tx_index++] = 0x55;

				// 2. 添加设备ID、长度、功能码、计数 (新帧头结构)
				tx_buffer[tx_index++] = 0x01;       // Device ID
				tx_buffer[tx_index++] = 0x6E;       // Payload Length (110 = 0x6E)
				tx_buffer[tx_index++] = 0x41;       // Function Code
				tx_buffer[tx_index++] = 0x10;       // Item Count (12 motors + 4 IMUs = 16)

				// 3. 添加12个电机的位置数据 (48字节) - 大端格式
				for (int i = 0; i < 12; i++) {
					tx_buffer[tx_index++] = (currentPosition_snake[i] >> 24) & 0xFF;
					tx_buffer[tx_index++] = (currentPosition_snake[i] >> 16) & 0xFF;
					tx_buffer[tx_index++] = (currentPosition_snake[i] >> 8) & 0xFF;
					tx_buffer[tx_index++] = currentPosition_snake[i] & 0xFF;
				}

				// 4. 添加12个电机的速度数据 (12字节)
				for (int i = 0; i < 12; i++) {
					tx_buffer[tx_index++] = currentSpeed_snake[i] & 0xFF;
				}

				// 5. 添加4个IMU的欧拉角数据 (4*3*4=48字节) - 小端格式
				// (Ensure get_eularX function prototypes are available via included headers)
				get_eular2(temp_eular);
				for (int i = 0; i < 3; i++) { uint8_t *p = (uint8_t*)&temp_eular[i]; tx_buffer[tx_index++] = p[0]; tx_buffer[tx_index++] = p[1]; tx_buffer[tx_index++] = p[2]; tx_buffer[tx_index++] = p[3]; }
				get_eular7(temp_eular);
				for (int i = 0; i < 3; i++) { uint8_t *p = (uint8_t*)&temp_eular[i]; tx_buffer[tx_index++] = p[0]; tx_buffer[tx_index++] = p[1]; tx_buffer[tx_index++] = p[2]; tx_buffer[tx_index++] = p[3]; }
				get_eular8(temp_eular);
				for (int i = 0; i < 3; i++) { uint8_t *p = (uint8_t*)&temp_eular[i]; tx_buffer[tx_index++] = p[0]; tx_buffer[tx_index++] = p[1]; tx_buffer[tx_index++] = p[2]; tx_buffer[tx_index++] = p[3]; }
				get_eular6(temp_eular);
				for (int i = 0; i < 3; i++) { uint8_t *p = (uint8_t*)&temp_eular[i]; tx_buffer[tx_index++] = p[0]; tx_buffer[tx_index++] = p[1]; tx_buffer[tx_index++] = p[2]; tx_buffer[tx_index++] = p[3]; }

				// 6. 计算CRC16校验值并添加到缓冲区末尾 (校验前 114 字节)
                // (Ensure calc_crc16_modbus prototype is available via included uart3.h)
				crc16 = calc_crc16_modbus(tx_buffer, tx_index); 
				tx_buffer[tx_index++] = (crc16 >> 8) & 0xFF; // CRC高字节
				tx_buffer[tx_index++] = crc16 & 0xFF;        // CRC低字节

				// 7. 通过USART3发送整个数据包 (116 字节)
				for (uint16_t i = 0; i < tx_index; i++) {
					USART_ClearFlag(USART3,USART_FLAG_TC); // Clear TC flag before sending
					USART_SendData(USART3, tx_buffer[i]);
					while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET); // 等待发送寄存器为空
				}
			} // End of scope for tx_buffer etc.
			
			OSTimeDly(10,OS_OPT_TIME_PERIODIC,&err); // 任务周期保持 8ms
	}
	
}

