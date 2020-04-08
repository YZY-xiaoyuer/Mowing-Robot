/**
  ******************************************************************************
  * @file    MOTOR_drive.h
  * @author  小鱼儿飞丫飞
  * @version V0.0.1
  * @date    2019/7/17
  * @brief   伽利略机器人电机驱动
  ******************************************************************************        
*/
#ifndef __MOTOR_DRIVER_H
#define __MOTOR_DRIVER_H
  
#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "shell.h"


//=========================================================

/* 电机驱动器输出使能标志 */     
typedef enum {     
   output_enable = 0,//--中间经过反相器。所以低电平使能
   output_disable 
} E_motor_output_en_flag;

/* 电机正反转选择标志 */
typedef enum {
   motor_fwd = 0,       /* 小车前进 */
   motor_rev            /* 小车后退 */
} E_motor_fwd_rev_flag;

/* 电机刹车标志 */
typedef enum {
    brake_enable = 0, //--中间经过反相器。所以低电平使能
    brake_disable
} E_motor_brake_flag;
/* 电机驱动芯片工作模式 */
typedef enum {
    work_mode_normal = 0, 
    work_mode_output_disable,    
    work_mode_brake_hard
} E_motor_driver_work_mode;
/* 电机选择 */
typedef enum {
    left_motor = 0,
    right_motor,    
    mowing_motor
} E_motor_switch;
//================================================================
/* 电机控制参数结构体 */
typedef struct _motor_ctrl_parameter {
    
    E_motor_output_en_flag        output_enable_flag;         //--电机驱动器输出使能标志
    E_motor_fwd_rev_flag          fwd_rev_flag;               //--电机旋转方向标志
    E_motor_brake_flag            brake_flag;                 //--刹车使能标志 
	  E_motor_driver_work_mode      motor_driver_work_mode;    //--电机驱动模式选择
	  E_motor_switch                motor_switc;                //--电机选择              
    uint16_t                      pwm_value;                 //--转速PWM脉宽 
} ST_motor_ctrl_parameter, *ST_motor_ctrl_parameter_p;

/* 电机控制句柄结构体 */
typedef struct _motor_ctrl_handle {
  
    GPIO_TypeDef        *pGPIOx_OutputEnable;              /* 电机驱动器输出使能控制端口   */
    uint16_t             GPIO_Pin_OutputEnable;            /* 电机驱动器输出使能控制引脚   */
    
    GPIO_TypeDef        *pGPIOx_FwdRevSelect;              /* 电机正反转选择端口          */
    uint16_t             GPIO_Pin_FwdRevSelect;            /* 电机正反转选择引脚          */
    
    GPIO_TypeDef        *pGPIOx_Brake;                     /* 电机驱动器制动输入端口      */
    uint16_t             GPIO_Pin_Brake;                   /* 电机驱动器制动输入引脚      */
 
    TIM_HandleTypeDef   *phtim_Speed_Adjust_PWM;             /* 电机调速PWM定时器           */
    uint32_t             Channel_Speed_Adjust_PWM;           /* 电机调速PWM定时器通道       */
     
} ST_motor_ctrl, *ST_motor_ctrl_p;

//===========================外部函数接口=======================================
/*函数接口*/
typedef struct _motor_function_interface{
/*初始化*/
uint8_t (*init)(void);
/*反初始化*/
uint8_t (*deinit)(void);
/*启动*/
uint8_t (*start)(int32_t pwm);
/*正常停止*/
uint8_t (*stop)(void);
/*紧急刹车*/
uint8_t (*brake)(void);
/*获取电机状态*/
uint8_t  (*get_motor_station)(uint16_t motor_station[2]);
}ST_motor_handle,*ST_motor_handle_p;
extern ST_motor_handle   left_motor_handle;
extern ST_motor_handle_p left_motor_handle_p;
extern ST_motor_handle   right_motor_handle;
extern ST_motor_handle_p right_motor_handle_p;


#endif



