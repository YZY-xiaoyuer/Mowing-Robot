/**
  ******************************************************************************
  * @file    WHEEL.h
  * @author  С�����Ѿ��
  * @version V0.0.1
  * @date    2019/7/30
  * @brief   ٤���Ի��������ӿ���
  ******************************************************************************/
#ifndef __WHEEL_H__
#define __WHEEL_H__

/*===========ͷ�ļ�==============*/
#include "main.h"
#include "tim.h"
#include "MOTOR_driver.h"
#include "speed.h"
#include "pid.h"
#include "car.h"
/*===========�궨��==============*/
#define  LEFT_MOTOR               (0)//--����
#define  RIGHT_MOTOR              (1)//--�ҵ��
#define  MOWING_MOTOR             (2) //--��ݵ��
#define WHEEL_GO   (1) //--ǰ��
#define WHEEL_BACK (2) //--����
#define PI        (314)
#define WHEEL_D    (205) //--mm��λ
#define WHEEL_STATION_SWITCH_DEFAULT (-1) //--״̬ѡ�����
/*===========����==============*/
typedef struct _wheel{
 int32_t speed;
 uint32_t wheel_mm;
 uint32_t wheel_cm;
 uint32_t wheel_m;
}ST_wheel_variable,*ST_wheel_variable_p;

/*===========�����ӿ�==============*/
typedef struct _wheel_function_interface{
 /*�ٶȸ���*/
uint8_t (*speed_updata)(void);
 /*��ʼ��*/
uint8_t (*init)(void);
 /*����ʼ��*/
uint8_t (*deinit)(void);
 /*��������*/
uint8_t (*start)(uint32_t speed);
 /*����ɲ��*/
uint8_t (*stop)(void);
 /*����ɲ��*/
uint8_t (*brake)(void);
/*�����������*/
uint32_t (*count_wheel_walk_distance)(void);
/*��ȡ���Ӿ���*/
uint32_t (*get_walk_distance)(void);
/*���Ӿ�������*/
uint8_t (*walk_distance_reset)(void);
/*��ȡ�ٶ�*/
int32_t (*read_speed)(void);
}ST_wheel_function_interface_handle,*ST_wheel_function_interface_handle_p;
extern ST_wheel_function_interface_handle   left_wheel_handle;
extern ST_wheel_function_interface_handle_p left_wheel_handle_p;
extern ST_wheel_function_interface_handle   right_wheel_handle;
extern ST_wheel_function_interface_handle_p right_wheel_handle_p;

#endif

