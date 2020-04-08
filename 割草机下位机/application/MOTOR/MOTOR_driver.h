/**
  ******************************************************************************
  * @file    MOTOR_drive.h
  * @author  С�����Ѿ��
  * @version V0.0.1
  * @date    2019/7/17
  * @brief   ٤���Ի����˵������
  ******************************************************************************        
*/
#ifndef __MOTOR_DRIVER_H
#define __MOTOR_DRIVER_H
  
#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "shell.h"


//=========================================================

/* ������������ʹ�ܱ�־ */     
typedef enum {     
   output_enable = 0,//--�м侭�������������Ե͵�ƽʹ��
   output_disable 
} E_motor_output_en_flag;

/* �������תѡ���־ */
typedef enum {
   motor_fwd = 0,       /* С��ǰ�� */
   motor_rev            /* С������ */
} E_motor_fwd_rev_flag;

/* ���ɲ����־ */
typedef enum {
    brake_enable = 0, //--�м侭�������������Ե͵�ƽʹ��
    brake_disable
} E_motor_brake_flag;
/* �������оƬ����ģʽ */
typedef enum {
    work_mode_normal = 0, 
    work_mode_output_disable,    
    work_mode_brake_hard
} E_motor_driver_work_mode;
/* ���ѡ�� */
typedef enum {
    left_motor = 0,
    right_motor,    
    mowing_motor
} E_motor_switch;
//================================================================
/* ������Ʋ����ṹ�� */
typedef struct _motor_ctrl_parameter {
    
    E_motor_output_en_flag        output_enable_flag;         //--������������ʹ�ܱ�־
    E_motor_fwd_rev_flag          fwd_rev_flag;               //--�����ת�����־
    E_motor_brake_flag            brake_flag;                 //--ɲ��ʹ�ܱ�־ 
	  E_motor_driver_work_mode      motor_driver_work_mode;    //--�������ģʽѡ��
	  E_motor_switch                motor_switc;                //--���ѡ��              
    uint16_t                      pwm_value;                 //--ת��PWM���� 
} ST_motor_ctrl_parameter, *ST_motor_ctrl_parameter_p;

/* ������ƾ���ṹ�� */
typedef struct _motor_ctrl_handle {
  
    GPIO_TypeDef        *pGPIOx_OutputEnable;              /* ������������ʹ�ܿ��ƶ˿�   */
    uint16_t             GPIO_Pin_OutputEnable;            /* ������������ʹ�ܿ�������   */
    
    GPIO_TypeDef        *pGPIOx_FwdRevSelect;              /* �������תѡ��˿�          */
    uint16_t             GPIO_Pin_FwdRevSelect;            /* �������תѡ������          */
    
    GPIO_TypeDef        *pGPIOx_Brake;                     /* ����������ƶ�����˿�      */
    uint16_t             GPIO_Pin_Brake;                   /* ����������ƶ���������      */
 
    TIM_HandleTypeDef   *phtim_Speed_Adjust_PWM;             /* �������PWM��ʱ��           */
    uint32_t             Channel_Speed_Adjust_PWM;           /* �������PWM��ʱ��ͨ��       */
     
} ST_motor_ctrl, *ST_motor_ctrl_p;

//===========================�ⲿ�����ӿ�=======================================
/*�����ӿ�*/
typedef struct _motor_function_interface{
/*��ʼ��*/
uint8_t (*init)(void);
/*����ʼ��*/
uint8_t (*deinit)(void);
/*����*/
uint8_t (*start)(int32_t pwm);
/*����ֹͣ*/
uint8_t (*stop)(void);
/*����ɲ��*/
uint8_t (*brake)(void);
/*��ȡ���״̬*/
uint8_t  (*get_motor_station)(uint16_t motor_station[2]);
}ST_motor_handle,*ST_motor_handle_p;
extern ST_motor_handle   left_motor_handle;
extern ST_motor_handle_p left_motor_handle_p;
extern ST_motor_handle   right_motor_handle;
extern ST_motor_handle_p right_motor_handle_p;


#endif



