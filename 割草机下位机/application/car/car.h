/**
  ******************************************************************************
  * @file    car_control.h
  * @author  Ğ¡Óã¶ù·ÉÑ¾·É
  * @version V0.0.1
  * @date    2019/7/30
  * @brief   Ù¤ÀûÂÔ»úÆ÷ÈËĞ¡³µµ×ÅÌ¿ØÖÆ
  ******************************************************************************/
#ifndef __CAR_H__
#define __CAR_H__


/*===========Í·ÎÄ¼ş==============*/
#include "main.h"
#include "tim.h"
#include "MOTOR_driver.h"
#include "speed.h"
#include "pid.h"
#include "wheel.h"
#include "inv_mpu.h"
#include "shell.h"
#include "data.h"
/*===========ºê¶¨Òå==============*/
#define CAR_LEFT  (1) //--×ó×ª
#define CAR_RIGHT (2) //--ÓÒ×ª
#define CAR_FWD   (1)  //--Ç°½ø
#define CAR_REV   (2)  //--ºóÍË
#define CAR_START (1)
#define CAR_STOP  (0)
#define CAR_WALK_STRAIGHT_LINE (1)
#define WHEEL_SPACING  (332)  //--³µÂÖ°ë¾¶£¬mmµ¥Î»
#define YAW_ERR   (4.0E-05)
/*===========±äÁ¿==============*/
		/*¶ÁÈ¡Ğ¡³µÀï³Ì*/
		/*ÅĞ¶ÏÊÇ·ñ×ßÍêÖ¸¶¨¾àÀë*/
		/*ÊÇ£¬É²³µÍ£»ú*/
		/*²»ÊÇ£¬¼ÆËãÎó²î£¬µ÷ÕûËÙ¶È»ù´¡Á¿*/
		/*¼ÆËã½Ç¶ÈÎó²î£¬µ÷ÕûËÙ¶ÈÆ«²îÁ¿*/
		/*ÉèÖÃĞ¡³µ×´Ì¬*/
typedef struct _car{
int8_t   car_start_stop;//--Ğ¡³µÍ£Ö¹Æô¶¯±êÖ¾
int8_t car_clear_flg_en;
int8_t car_q;//--×ß8×ÖÈ¦Êı
int32_t yaw_add ;
int32_t err_add ;
int32_t  car_speed;  //--Ğ¡³µËÙ¶ÈÉè¶¨
int32_t  now_car_speed;//--Ğ¡³µµ±Ç°ËÙ¶È
uint32_t car_distance;  //--Ğ¡³µĞĞÊ»¾àÀë	Éè¶¨
uint32_t last_car_distance;//--±£´æĞ¡³µ³õÊ¼¾àÀë
uint32_t now_car_distance;  //--µ±Ç°Ğ¡³µĞĞÊ»¾àÀë
float    car_angle;  //--Ğ¡³µ×ª¶¯½Ç¶ÈÉè¶¨
float    now_car_angle;//--Ğ¡³µµ±Ç°×ª¶¯½Ç¶È
float    car_start_angle;//--Ğ¡³µ³õÊ¼½Ç¶È
float    car_angular_speed;//--Ğ¡³µÉè¶¨½ÇËÙ¶È
float    now_car_angular_speed;//--Ğ¡³µµ±Ç°½ÇËÙ¶È

////----------------------------------------------------------
//uint32_t differential_ratio;//--²îËÙ±È£¬mm¼¶±ğ
//uint32_t inner_wheel_radius;  //--ÄÚÂÖ×ªÍä°ë¾¶£¬mmµ¥Î»
//uint32_t outer_wheel_radius;//--ÍâÂÖ×ªÍä°ë¾¶£¬mmµ¥Î»
//uint32_t car_radius;     //--Ğ¡³µµ×ÅÌÖĞĞÄ×ªÍä°ë¾¶¡£mmµ¥Î»
//int32_t speed_add;        //--×óÓÒÂÖËÙ¶ÈºÍ
//----------------------------------------------------------
float now_pitch;                   //--¸©Ñö½Ç -90.0<---> +90.0
float now_roll;                    //--·­¹ö½Ç  -180.0---> +180.0
float now_yaw;                     //--º½Ïò½Ç  -180.0°<---> +180.0
float last_pitch;                   //--¸©Ñö½Ç -90.0<---> +90.0
float last_roll;                    //--·­¹ö½Ç  -180.0---> +180.0
float last_yaw;                     //--º½Ïò½Ç  
}ST_car_variable,*ST_car_variable_p;
extern int8_t car_clear_flg_en;
/*===========º¯Êı½Ó¿Ú==============*/
typedef struct _car_function_interface{
	  /*Ğ¡³µ³õÊ¼»¯*/
	uint8_t (*car_init)(void);
	/*Ğ¡³µ·´³õÊ¼»¯*/
	uint8_t (*car_deinit)(void);
	/*Ğ¡³µËÙ¶È¸üĞÂ*/
	uint8_t (*car_speed_updata)(void);
	/*Ğ¡³µÕı³£Í£Ö¹*/
	uint8_t (*car_stop)(void);
	/*Ğ¡³µ½ô¼±Í£Ö¹*/
	uint8_t (*car_brake)(void);
	/*»ñÈ¡Ö±ÏßĞĞ×ß²ÎÊı*/
  int8_t (*car_walk_straight_line_updata)(int32_t v,uint32_t s);
	/*½ÓÊÕÉÏÎ»»úÊı¾İ*/
  int8_t (*car_turn_updata)(int32_t V,float W,float A);
	/*Ğ¡³µÒ£¿Ø×ßÖ±Ïß*/
  int8_t (*car_remote_line)(void);
/*Ğ¡³µÒ£¿Ø×ªÍä*/
  int8_t (*car_remote_turn)(void);
	/*Ğ¡³µ×ßÖ±Ïß*/
	int8_t (*car_walk_straight_line)(void);
	/*Ğ¡³µÔ­µØ×ªÍä*/
	/*Ğ¡³µÑØÄ³ÌõÇúÏßÇ°½ø»òÕßºóÍË*/
	uint8_t (*car_turn)(void);
	/*Ğı×ª¹ÕÍä*/
	/*¶ÁÈ¡Ğ¡³µÏßËÙ¶È*/
	int32_t (*read_speed)(void);
	/*¶ÁÈ¡Ğ¡³µ½ÇËÙ¶È*/
	/*¶ÁÈ¡Ğ¡³µĞĞÊ»¾àÀë*/
 uint32_t (*read_car_distance)(void);
	/*¶ÁÈ¡Ğ¡³µ×´Ì¬ÃüÁî*/
 uint8_t (*read_car_start_stop)(void);
 /*Ğ¡³µ²ÎÊıÇåÁã*/
uint8_t (*car_variable_reset)(void);
/*¼ÆËãĞ¡³µÖ±ÏßĞĞ×ßµÄ½Ç¶ÈÎó²î*/
uint32_t (*car_walk_straight_line_calculate_angle_difference)(uint16_t range,int32_t naw_angle , int32_t last_angle);
/*¼ÆËãĞ¡³µÖ±ÏßĞĞ×ß×óÓÒÂÖËÙ¶È*/
int32_t*  (*car_walk_straight_line_calculate_wheel_speed)(int32_t reference_value,int32_t adjusted_value);
/*¼õËÙÍ£³µ*/
uint32_t (*slow_down_and_stop)(void);
/*¼ÆËã×ªÍä½Ç¶ÈºÍ*/
int32_t (*car_turn_calculate_angle_difference)(int32_t w,int32_t naw_angle , int32_t last_angle);
/*»ñÈ¡Ğ¡³µÆôÍ£×´Ì¬*/
uint8_t (*get_car_stop_or_start)(void);
uint8_t (*car_turn_8)(void);
int8_t (*car_turn_8_updata)(int32_t V,float W,int Q);
}ST_car_function_interface_handle,*ST_car_function_interface_handle_p;
extern ST_car_function_interface_handle   car_handle;
extern ST_car_function_interface_handle_p car_handle_p;

#endif
