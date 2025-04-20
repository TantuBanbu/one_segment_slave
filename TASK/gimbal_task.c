#include "can1.h"
#include "can2.h"
#include "task.h"
#include "delay.h"
#include "pstwo.h"
#include "uart3.h"
//#include "uart6.h"
//#include "uart8.h"
#include "pwm.h"
#include "imu.h"
#include "gpio.h"
#include "includes.h"
#include "math.h"
#include <stdint.h>  // ����int16_t��ͷ�ļ�



void USART6_Init(void);
/****************************************************************************
													GM6020�����PID����
****************************************************************************/

int32_t GM6020_output_limit = 30000; // �������ѹ��������

int32_t GM6020_error = 0; // ���ֵ
int32_t GM6020_position_difference = 0; // ����λ�ò�
// PID����
double GM6020_Kp = 100;  // ��������
double GM6020_Ki = 0;  // ���ֳ���
double GM6020_Kd = 10; // ΢�ֳ���
int32_t GM6020_output = 0;    // �����ѹ -30000 - 30000 ��ֹ���
int32_t GM6020_prev_error = 0; // ��һ�ε����
int32_t GM6020_integral = 0;   // ������
int32_t GM6020_derivative; 	// ���΢��			
// ����GM6020_PID������
void GM6020_update_pid() {
	

		// ����λ�ò�
		GM6020_position_difference = GM6020_current_raw_position - GM6020_last_raw_position;
		// ����Ƿ���λ�ð���
		if (abs(GM6020_position_difference) > 180) {  
				if (GM6020_position_difference > 0) {
						--GM6020_rotation_count;
				} else {
						++GM6020_rotation_count;
				}
		}
		// �������λ��
		GM6020_absolute_position = GM6020_current_raw_position + GM6020_rotation_count * 360 - gripper_gm6020_position_reset_offset;
    GM6020_error = gripper_gm6020_position_control - GM6020_absolute_position;  // �������
    GM6020_integral += GM6020_error;                // ����������
    GM6020_derivative = GM6020_error - GM6020_prev_error;  // �������΢��
    // ���������ѹ
		GM6020_output = GM6020_Kp * GM6020_error + GM6020_Ki * GM6020_integral + GM6020_Kd * GM6020_derivative; // PID����ϵͳ�г�����ƫ�steady-state error��ʱ��ʹ��PID��
		// ����GM6020_output��-GM6020_output_limit��GM6020_output_limit֮��
    if (GM6020_output > GM6020_output_limit) {
        GM6020_output = GM6020_output_limit;
    } else if (GM6020_output < -GM6020_output_limit) {
        GM6020_output = -GM6020_output_limit;
    }
		GM6020_output = PID_GM6020_Velocity(GM6020_output,GripperMotor_205_t.velocity); // pid for velocity
		// ���Ϳ���ָ��
		if(reset_control == 0){
			GM6020_Can_Send_Msg(GM6020_output,0,0,0);
		}
    GM6020_prev_error = GM6020_error;  // ������һ�ε����
		// ������һ�ε�ԭʼλ��
		GM6020_last_raw_position = GM6020_current_raw_position;
		// ���µ�ǰλ��
		GM6020_current_raw_position = GripperMotor_205_t.position; 

}

int32_t PID_GM6020_Velocity(int32_t target_velocity,int32_t current_velocity) 
{
	const float Kp = 1;  //16.5
	const float Ki=0;   //0.03
	const float Kd =0.0f;
    
	static float error_v[2] = {0.0,0.0};
	static float error_sum=0;
	static float error =0; 
	
	error_v[0] = error_v[1];
	error_v[1] = target_velocity - current_velocity;	
	error_sum += error_v[1];

	if(error_sum > 10000)  error_sum =  10000;      //
	else if(error_sum < -10000) error_sum = -10000;
	error = error_v[1]  * Kp 
				 +  error_sum * Ki 
				 + (error_v[1] - error_v[0]) * Kd; 
	
	if(error > GM6020_output_limit)  error = GM6020_output_limit;
	else if(error < -GM6020_output_limit) error = -GM6020_output_limit;
	return error;
}

/****************************************************************************/

/****************************************************************************
													C610�����PID����
****************************************************************************/
int32_t C610_output_limit = 10000; // �����������������

int32_t C610_error = 0; // ���ֵ
int32_t C610_position_difference = 0; // ����λ�ò�
// PID����
double C610_Kp = 1;  // ��������   ��Ҫ����
double C610_Ki = 0;  // ���ֳ���
double C610_Kd = 10; // ΢�ֳ���
int32_t C610_output = 0;    // �����ѹ -10000 - 10000 ��ֹ���
int32_t C610_prev_error = 0; // ��һ�ε����
int32_t C610_integral = 0;   // ������
int32_t C610_derivative; 	// ���΢��	
// ����C610_PID������
void C610_update_pid() 
	{
		// ����λ�ò�
		C610_position_difference = C610_current_raw_position - C610_last_raw_position;
		// ����Ƿ���λ�ð���
		if (abs(C610_position_difference) > 180) {  
				if (C610_position_difference > 0) {
						--C610_rotation_count;
				} else {
						++C610_rotation_count;
				}
		}
		// �������λ��
		C610_absolute_position = C610_current_raw_position + C610_rotation_count * 360 - gripper_c610_position_reset_offset;
    C610_error = gripper_c610_position_control - C610_absolute_position;  // �������
    C610_integral += C610_error;                // ����������
    C610_derivative = C610_error - C610_prev_error;  // �������΢��
    // ���������ѹ
		C610_output = 1 * (C610_Kp * C610_error + C610_Ki * C610_integral + C610_Kd * C610_derivative); // PID����ϵͳ�г�����ƫ�steady-state error��ʱ��ʹ��PID��
		// ����C610_output��-C610_output_limit��C610_output_limit֮��
    if (C610_output > C610_output_limit) {
        C610_output = C610_output_limit;
    } else if (C610_output < -C610_output_limit) {
        C610_output = -C610_output_limit;
    }
		C610_output = PID_C610_Velocity(C610_output,GripperMotor_201_t.velocity); // pid for velocity
		// ���Ϳ���ָ��
		if(reset_control == 0){
		C610_Can_Send_Msg(C610_output,0,0,0);
		}
    C610_prev_error = C610_error;  // ������һ�ε����
		// ������һ�ε�ԭʼλ��
		C610_last_raw_position = C610_current_raw_position;
		// ���µ�ǰλ��
		C610_current_raw_position = GripperMotor_201_t.position; 
		
}
	
int32_t PID_C610_Velocity(int32_t target_velocity,int32_t current_velocity) 
{
	const float Kp =1;  //16.5
	const float Ki=0.001f;   //0.03
	const float Kd =0.0f;
    
	static float error_v[2] = {0.0,0.0};
	static float error_sum=0;
	static float error =0; 
	
	error_v[0] = error_v[1];
	error_v[1] = target_velocity - current_velocity;	
	error_sum += error_v[1];

	if(error_sum > 1000)  error_sum =  1000;      //
	else if(error_sum < -1000) error_sum = -1000;
	error = error_v[1]  * Kp 
				 +  error_sum * Ki 
				 + (error_v[1] - error_v[0]) * Kd; 
	
	if(error > 20000)  error = 20000;
	else if(error < -20000) error = -20000;
	return error;
}

/****************************************************************************/

/****************************************************************************
															STS3032�������
****************************************************************************/


/****************************************************************************/

void Gimbal_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;

	while(1)
	{  	
	
			
			GM6020_update_pid(); // ����GM6020_PID������
			C610_update_pid();  // ����C610_PID������

			OSTimeDly(2,OS_OPT_TIME_PERIODIC,&err); //��ʱ2ms
  } 
}



