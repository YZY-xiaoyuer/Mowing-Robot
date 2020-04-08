/**
  ******************************************************************************
  * @file    pid.h
  * @author  С�����Ѿ��
  * @version V0.0.1
  * @date    2019/7/25
  * @brief   ٤���Ի�����PID�㷨
  ******************************************************************************        
*/
#ifndef __PID_H__
#define __PID_H__

/*===========ͷ�ļ�==============*/
#include "main.h"
#include "tim.h"
#include "MOTOR_driver.h"
#include "speed.h"
#include "usart.h"
#include "car.h"
/*===========�궨��==============*/
//--�޷����
#define max(a, b)			(a>b? a:b)
#define min(a, b)			(a<b? a:b)
#define range(x, a, b)		(min(max(x, a), b))
//--
/*===========����==============*/
typedef struct _pid{
	  int32_t output_add;         //--PID�����ۼ�ֵ
    int32_t to_value;        //--Ŀ��ֵ
    int32_t current_value;   //--��ǰֵ
	  int32_t output_value;    //--���ֵ     
    int32_t current_err;     //--��ǰ���
    int32_t last_err;        //--��һ�����
	  int32_t last_last_err;   //--����һ����� 
    int32_t kp;
	  int32_t ki;
	  int32_t kd;           
    int32_t integral;        //--����ֵ
	  int32_t PWM_umax;           //--������ֵ
    int32_t PWM_umin;           //--�����Сֵ
	  int32_t speed_umax;           //--������ֵ
    int32_t speed_umin;           //--�����Сֵ
}ST_pid_variable,*ST_pid_variable_p;

/*===========�����ӿ�==============*/
typedef struct _pid_function_interface{	
int32_t  (*wheel_integral_separation_pid)(int32_t rang);
uint8_t  (*set_wheel_pid_speed)(int32_t to_speed);
int32_t  (*read_pwm)(void);	
uint8_t (*set_pid_value)(uint16_t kp,uint16_t ki, uint16_t kd);
uint8_t (*pwm_value_reset)(void);
int32_t (*car_walk_distance_pid)(int32_t speed,uint32_t now_distance ,uint32_t to_distance);
}ST_pid_function_interface_handle,*ST_pid_function_interface_handle_p;
extern ST_pid_function_interface_handle   left_pid_handle;
extern ST_pid_function_interface_handle_p left_pid_handle_p;
extern ST_pid_function_interface_handle   right_pid_handle;
extern ST_pid_function_interface_handle_p right_pid_handle_p;
typedef struct _car_pid_function_interface{	
int32_t (*car_walk_distance_pid)(int32_t speed,uint32_t now_distance ,uint32_t to_distance);
int32_t (*car_walk_yaw_pid)(int32_t yaw_err);
uint8_t (*car_walk_distance_pid_value_reset)(void);
uint8_t (*car_walk_yaw_pid_value_reset)(void);
uint8_t (*set_car_walk_distance_pid_value)(uint16_t kp,uint16_t ki, uint16_t kd);
uint8_t (*set_car_walk_yaw_pid_value)(uint16_t kp,uint16_t ki, uint16_t kd);
}ST_car_pid_function_interface_handle,*ST_car_pid_function_interface_handle_p;
extern ST_car_pid_function_interface_handle   car_pid_handle;
extern ST_car_pid_function_interface_handle_p car_pid_handle_p;
#endif
