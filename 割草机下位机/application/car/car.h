/**
  ******************************************************************************
  * @file    car_control.h
  * @author  С�����Ѿ��
  * @version V0.0.1
  * @date    2019/7/30
  * @brief   ٤���Ի�����С�����̿���
  ******************************************************************************/
#ifndef __CAR_H__
#define __CAR_H__


/*===========ͷ�ļ�==============*/
#include "main.h"
#include "tim.h"
#include "MOTOR_driver.h"
#include "speed.h"
#include "pid.h"
#include "wheel.h"
#include "inv_mpu.h"
#include "shell.h"
#include "data.h"
/*===========�궨��==============*/
#define CAR_LEFT  (1) //--��ת
#define CAR_RIGHT (2) //--��ת
#define CAR_FWD   (1)  //--ǰ��
#define CAR_REV   (2)  //--����
#define CAR_START (1)
#define CAR_STOP  (0)
#define CAR_WALK_STRAIGHT_LINE (1)
#define WHEEL_SPACING  (332)  //--���ְ뾶��mm��λ
#define YAW_ERR   (4.0E-05)
/*===========����==============*/
		/*��ȡС�����*/
		/*�ж��Ƿ�����ָ������*/
		/*�ǣ�ɲ��ͣ��*/
		/*���ǣ������������ٶȻ�����*/
		/*����Ƕ��������ٶ�ƫ����*/
		/*����С��״̬*/
typedef struct _car{
int8_t   car_start_stop;//--С��ֹͣ������־
int8_t car_clear_flg_en;
int8_t car_q;//--��8��Ȧ��
int32_t yaw_add ;
int32_t err_add ;
int32_t  car_speed;  //--С���ٶ��趨
int32_t  now_car_speed;//--С����ǰ�ٶ�
uint32_t car_distance;  //--С����ʻ����	�趨
uint32_t last_car_distance;//--����С����ʼ����
uint32_t now_car_distance;  //--��ǰС����ʻ����
float    car_angle;  //--С��ת���Ƕ��趨
float    now_car_angle;//--С����ǰת���Ƕ�
float    car_start_angle;//--С����ʼ�Ƕ�
float    car_angular_speed;//--С���趨���ٶ�
float    now_car_angular_speed;//--С����ǰ���ٶ�

////----------------------------------------------------------
//uint32_t differential_ratio;//--���ٱȣ�mm����
//uint32_t inner_wheel_radius;  //--����ת��뾶��mm��λ
//uint32_t outer_wheel_radius;//--����ת��뾶��mm��λ
//uint32_t car_radius;     //--С����������ת��뾶��mm��λ
//int32_t speed_add;        //--�������ٶȺ�
//----------------------------------------------------------
float now_pitch;                   //--������ -90.0<---> +90.0
float now_roll;                    //--������  -180.0---> +180.0
float now_yaw;                     //--�����  -180.0�<---> +180.0
float last_pitch;                   //--������ -90.0<---> +90.0
float last_roll;                    //--������  -180.0---> +180.0
float last_yaw;                     //--�����  
}ST_car_variable,*ST_car_variable_p;
extern int8_t car_clear_flg_en;
/*===========�����ӿ�==============*/
typedef struct _car_function_interface{
	  /*С����ʼ��*/
	uint8_t (*car_init)(void);
	/*С������ʼ��*/
	uint8_t (*car_deinit)(void);
	/*С���ٶȸ���*/
	uint8_t (*car_speed_updata)(void);
	/*С������ֹͣ*/
	uint8_t (*car_stop)(void);
	/*С������ֹͣ*/
	uint8_t (*car_brake)(void);
	/*��ȡֱ�����߲���*/
  int8_t (*car_walk_straight_line_updata)(int32_t v,uint32_t s);
	/*������λ������*/
  int8_t (*car_turn_updata)(int32_t V,float W,float A);
	/*С��ң����ֱ��*/
  int8_t (*car_remote_line)(void);
/*С��ң��ת��*/
  int8_t (*car_remote_turn)(void);
	/*С����ֱ��*/
	int8_t (*car_walk_straight_line)(void);
	/*С��ԭ��ת��*/
	/*С����ĳ������ǰ�����ߺ���*/
	uint8_t (*car_turn)(void);
	/*��ת����*/
	/*��ȡС�����ٶ�*/
	int32_t (*read_speed)(void);
	/*��ȡС�����ٶ�*/
	/*��ȡС����ʻ����*/
 uint32_t (*read_car_distance)(void);
	/*��ȡС��״̬����*/
 uint8_t (*read_car_start_stop)(void);
 /*С����������*/
uint8_t (*car_variable_reset)(void);
/*����С��ֱ�����ߵĽǶ����*/
uint32_t (*car_walk_straight_line_calculate_angle_difference)(uint16_t range,int32_t naw_angle , int32_t last_angle);
/*����С��ֱ�������������ٶ�*/
int32_t*  (*car_walk_straight_line_calculate_wheel_speed)(int32_t reference_value,int32_t adjusted_value);
/*����ͣ��*/
uint32_t (*slow_down_and_stop)(void);
/*����ת��ǶȺ�*/
int32_t (*car_turn_calculate_angle_difference)(int32_t w,int32_t naw_angle , int32_t last_angle);
/*��ȡС����ͣ״̬*/
uint8_t (*get_car_stop_or_start)(void);
uint8_t (*car_turn_8)(void);
int8_t (*car_turn_8_updata)(int32_t V,float W,int Q);
}ST_car_function_interface_handle,*ST_car_function_interface_handle_p;
extern ST_car_function_interface_handle   car_handle;
extern ST_car_function_interface_handle_p car_handle_p;

#endif
