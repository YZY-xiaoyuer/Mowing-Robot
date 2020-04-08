#ifndef __SPEED_H__
#define __SPEED_H__

/*===========ͷ�ļ�==============*/
#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "MOTOR_driver.h"
/*===========�궨��==============*/
#define SPEED_SIZE         (15) //--�����С
#define REDUCTION_RATIO    (125) //--���ٱ� //78
#define WHEEL_D            (205) //--����ֱ��mm��λ
#define PI                 (314)
/*===========����==============*/

typedef struct _speed_variable{
uint8_t first_electrification_flag ;//--�����ϵ��־λ
uint8_t   smoothing_en;    /*�˲�ʹ��*/
uint16_t  HAL_CNT;  //--�����������ֵ
uint16_t  overflow_count;  //--���������������
int32_t   motor_speed;
}ST_speed_variable,*ST_speed_variable_p;

typedef struct _speed_ctrl_handle {
 IRQn_Type            external_interrupts;  /*���������������ⲿ�жϺ�*/
 TIM_HandleTypeDef   *phtim_Speed_cnt; /*������*/
} ST_speed_ctrl, *ST_speed_ctrl_p;

/*===========�����ӿ�==============*/
typedef struct _speed_function_interface{
/*תһȦ������ֵ����*/
uint32_t  (*counting_difference)(void);
/*�ٶȸ��£�mm/s*/
uint8_t (*refresh_speed)(void);
/*��ȡ�ٶ�*/
int32_t (*read_speed)(void);
/*�ٶȳ�ʼ��*/
uint8_t (*speed_init)(void);
/*�ٶȷ���ʼ��*/
uint8_t (*speed_deinit)(void);
/*��ȡ�������ֵ*/
uint32_t (*get_motor_cnt)(void);
/*�����������ֵ*/
uint8_t (*motor_cnt_reset)(void);
}ST_speed_function_interface_handle,*ST_speed_function_interface_handle_p;
extern ST_speed_function_interface_handle   left_speed_handle;
extern ST_speed_function_interface_handle_p left_speed_handle_p;
extern ST_speed_function_interface_handle   right_speed_handle;
extern ST_speed_function_interface_handle_p right_speed_handle_p;

#endif


