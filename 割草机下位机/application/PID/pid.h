/**
  ******************************************************************************
  * @file    pid.h
  * @author  小鱼儿飞丫飞
  * @version V0.0.1
  * @date    2019/7/25
  * @brief   伽利略机器人PID算法
  ******************************************************************************        
*/
#ifndef __PID_H__
#define __PID_H__

/*===========头文件==============*/
#include "main.h"
#include "tim.h"
#include "MOTOR_driver.h"
#include "speed.h"
#include "usart.h"
#include "car.h"
/*===========宏定义==============*/
//--限幅输出
#define max(a, b)			(a>b? a:b)
#define min(a, b)			(a<b? a:b)
#define range(x, a, b)		(min(max(x, a), b))
//--
/*===========变量==============*/
typedef struct _pid{
	  int32_t output_add;         //--PID计算累加值
    int32_t to_value;        //--目标值
    int32_t current_value;   //--当前值
	  int32_t output_value;    //--输出值     
    int32_t current_err;     //--当前误差
    int32_t last_err;        //--上一个误差
	  int32_t last_last_err;   //--上上一个误差 
    int32_t kp;
	  int32_t ki;
	  int32_t kd;           
    int32_t integral;        //--积分值
	  int32_t PWM_umax;           //--输出最大值
    int32_t PWM_umin;           //--输出最小值
	  int32_t speed_umax;           //--输出最大值
    int32_t speed_umin;           //--输出最小值
}ST_pid_variable,*ST_pid_variable_p;

/*===========函数接口==============*/
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
