/**
  ******************************************************************************
  * @file    WHEEL.h
  * @author  小鱼儿飞丫飞
  * @version V0.0.1
  * @date    2019/7/30
  * @brief   伽利略机器人轮子控制
  ******************************************************************************/
#ifndef __WHEEL_H__
#define __WHEEL_H__

/*===========头文件==============*/
#include "main.h"
#include "tim.h"
#include "MOTOR_driver.h"
#include "speed.h"
#include "pid.h"
#include "car.h"
/*===========宏定义==============*/
#define  LEFT_MOTOR               (0)//--左电机
#define  RIGHT_MOTOR              (1)//--右电机
#define  MOWING_MOTOR             (2) //--割草电机
#define WHEEL_GO   (1) //--前进
#define WHEEL_BACK (2) //--后退
#define PI        (314)
#define WHEEL_D    (205) //--mm单位
#define WHEEL_STATION_SWITCH_DEFAULT (-1) //--状态选择错误
/*===========变量==============*/
typedef struct _wheel{
 int32_t speed;
 uint32_t wheel_mm;
 uint32_t wheel_cm;
 uint32_t wheel_m;
}ST_wheel_variable,*ST_wheel_variable_p;

/*===========函数接口==============*/
typedef struct _wheel_function_interface{
 /*速度更新*/
uint8_t (*speed_updata)(void);
 /*初始化*/
uint8_t (*init)(void);
 /*反初始化*/
uint8_t (*deinit)(void);
 /*轮子启动*/
uint8_t (*start)(uint32_t speed);
 /*正常刹车*/
uint8_t (*stop)(void);
 /*紧急刹车*/
uint8_t (*brake)(void);
/*计算轮子里程*/
uint32_t (*count_wheel_walk_distance)(void);
/*获取轮子距离*/
uint32_t (*get_walk_distance)(void);
/*轮子距离清零*/
uint8_t (*walk_distance_reset)(void);
/*读取速度*/
int32_t (*read_speed)(void);
}ST_wheel_function_interface_handle,*ST_wheel_function_interface_handle_p;
extern ST_wheel_function_interface_handle   left_wheel_handle;
extern ST_wheel_function_interface_handle_p left_wheel_handle_p;
extern ST_wheel_function_interface_handle   right_wheel_handle;
extern ST_wheel_function_interface_handle_p right_wheel_handle_p;

#endif

