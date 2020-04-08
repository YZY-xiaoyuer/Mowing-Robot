#ifndef __SPEED_H__
#define __SPEED_H__

/*===========头文件==============*/
#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "MOTOR_driver.h"
/*===========宏定义==============*/
#define SPEED_SIZE         (15) //--缓存大小
#define REDUCTION_RATIO    (125) //--减速比 //78
#define WHEEL_D            (205) //--车轮直径mm单位
#define PI                 (314)
/*===========变量==============*/

typedef struct _speed_variable{
uint8_t first_electrification_flag ;//--初次上电标志位
uint8_t   smoothing_en;    /*滤波使能*/
uint16_t  HAL_CNT;  //--霍尔脉冲计数值
uint16_t  overflow_count;  //--霍尔脉冲溢出计数
int32_t   motor_speed;
}ST_speed_variable,*ST_speed_variable_p;

typedef struct _speed_ctrl_handle {
 IRQn_Type            external_interrupts;  /*电机霍尔脉冲测量外部中断号*/
 TIM_HandleTypeDef   *phtim_Speed_cnt; /*计数器*/
} ST_speed_ctrl, *ST_speed_ctrl_p;

/*===========函数接口==============*/
typedef struct _speed_function_interface{
/*转一圈脉冲数值计算*/
uint32_t  (*counting_difference)(void);
/*速度更新，mm/s*/
uint8_t (*refresh_speed)(void);
/*读取速度*/
int32_t (*read_speed)(void);
/*速度初始化*/
uint8_t (*speed_init)(void);
/*速度反初始化*/
uint8_t (*speed_deinit)(void);
/*获取脉冲计数值*/
uint32_t (*get_motor_cnt)(void);
/*清零脉冲计数值*/
uint8_t (*motor_cnt_reset)(void);
}ST_speed_function_interface_handle,*ST_speed_function_interface_handle_p;
extern ST_speed_function_interface_handle   left_speed_handle;
extern ST_speed_function_interface_handle_p left_speed_handle_p;
extern ST_speed_function_interface_handle   right_speed_handle;
extern ST_speed_function_interface_handle_p right_speed_handle_p;

#endif


