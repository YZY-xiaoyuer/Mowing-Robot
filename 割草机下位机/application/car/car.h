/**
  ******************************************************************************
  * @file    car_control.h
  * @author  小鱼儿飞丫飞
  * @version V0.0.1
  * @date    2019/7/30
  * @brief   伽利略机器人小车底盘控制
  ******************************************************************************/
#ifndef __CAR_H__
#define __CAR_H__


/*===========头文件==============*/
#include "main.h"
#include "tim.h"
#include "MOTOR_driver.h"
#include "speed.h"
#include "pid.h"
#include "wheel.h"
#include "inv_mpu.h"
#include "shell.h"
#include "data.h"
/*===========宏定义==============*/
#define CAR_LEFT  (1) //--左转
#define CAR_RIGHT (2) //--右转
#define CAR_FWD   (1)  //--前进
#define CAR_REV   (2)  //--后退
#define CAR_START (1)
#define CAR_STOP  (0)
#define CAR_WALK_STRAIGHT_LINE (1)
#define WHEEL_SPACING  (332)  //--车轮半径，mm单位
#define YAW_ERR   (4.0E-05)
/*===========变量==============*/
		/*读取小车里程*/
		/*判断是否走完指定距离*/
		/*是，刹车停机*/
		/*不是，计算误差，调整速度基础量*/
		/*计算角度误差，调整速度偏差量*/
		/*设置小车状态*/
typedef struct _car{
int8_t   car_start_stop;//--小车停止启动标志
int8_t car_clear_flg_en;
int8_t car_q;//--走8字圈数
int32_t yaw_add ;
int32_t err_add ;
int32_t  car_speed;  //--小车速度设定
int32_t  now_car_speed;//--小车当前速度
uint32_t car_distance;  //--小车行驶距离	设定
uint32_t last_car_distance;//--保存小车初始距离
uint32_t now_car_distance;  //--当前小车行驶距离
float    car_angle;  //--小车转动角度设定
float    now_car_angle;//--小车当前转动角度
float    car_start_angle;//--小车初始角度
float    car_angular_speed;//--小车设定角速度
float    now_car_angular_speed;//--小车当前角速度

////----------------------------------------------------------
//uint32_t differential_ratio;//--差速比，mm级别
//uint32_t inner_wheel_radius;  //--内轮转弯半径，mm单位
//uint32_t outer_wheel_radius;//--外轮转弯半径，mm单位
//uint32_t car_radius;     //--小车底盘中心转弯半径。mm单位
//int32_t speed_add;        //--左右轮速度和
//----------------------------------------------------------
float now_pitch;                   //--俯仰角 -90.0<---> +90.0
float now_roll;                    //--翻滚角  -180.0---> +180.0
float now_yaw;                     //--航向角  -180.0�<---> +180.0
float last_pitch;                   //--俯仰角 -90.0<---> +90.0
float last_roll;                    //--翻滚角  -180.0---> +180.0
float last_yaw;                     //--航向角  
}ST_car_variable,*ST_car_variable_p;
extern int8_t car_clear_flg_en;
/*===========函数接口==============*/
typedef struct _car_function_interface{
	  /*小车初始化*/
	uint8_t (*car_init)(void);
	/*小车反初始化*/
	uint8_t (*car_deinit)(void);
	/*小车速度更新*/
	uint8_t (*car_speed_updata)(void);
	/*小车正常停止*/
	uint8_t (*car_stop)(void);
	/*小车紧急停止*/
	uint8_t (*car_brake)(void);
	/*获取直线行走参数*/
  int8_t (*car_walk_straight_line_updata)(int32_t v,uint32_t s);
	/*接收上位机数据*/
  int8_t (*car_turn_updata)(int32_t V,float W,float A);
	/*小车遥控走直线*/
  int8_t (*car_remote_line)(void);
/*小车遥控转弯*/
  int8_t (*car_remote_turn)(void);
	/*小车走直线*/
	int8_t (*car_walk_straight_line)(void);
	/*小车原地转弯*/
	/*小车沿某条曲线前进或者后退*/
	uint8_t (*car_turn)(void);
	/*旋转拐弯*/
	/*读取小车线速度*/
	int32_t (*read_speed)(void);
	/*读取小车角速度*/
	/*读取小车行驶距离*/
 uint32_t (*read_car_distance)(void);
	/*读取小车状态命令*/
 uint8_t (*read_car_start_stop)(void);
 /*小车参数清零*/
uint8_t (*car_variable_reset)(void);
/*计算小车直线行走的角度误差*/
uint32_t (*car_walk_straight_line_calculate_angle_difference)(uint16_t range,int32_t naw_angle , int32_t last_angle);
/*计算小车直线行走左右轮速度*/
int32_t*  (*car_walk_straight_line_calculate_wheel_speed)(int32_t reference_value,int32_t adjusted_value);
/*减速停车*/
uint32_t (*slow_down_and_stop)(void);
/*计算转弯角度和*/
int32_t (*car_turn_calculate_angle_difference)(int32_t w,int32_t naw_angle , int32_t last_angle);
/*获取小车启停状态*/
uint8_t (*get_car_stop_or_start)(void);
uint8_t (*car_turn_8)(void);
int8_t (*car_turn_8_updata)(int32_t V,float W,int Q);
}ST_car_function_interface_handle,*ST_car_function_interface_handle_p;
extern ST_car_function_interface_handle   car_handle;
extern ST_car_function_interface_handle_p car_handle_p;

#endif
