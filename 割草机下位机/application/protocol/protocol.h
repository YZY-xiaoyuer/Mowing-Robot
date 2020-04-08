/**
  ******************************************************************************
  * @file    protocol.h
  * @author  zz
  * @version
  * @date    2019-11-26
  * @brief
  ******************************************************************************
  * @attention
  *
  *
  * Copyright (c) 2011-2027 by zz (xxxxx@xxxx.xxx)
  * All rights reserved.
  *-----------------------------Revision History--------------------------------
  * No    Version      Data       Revised By      Item        Description
  * 1.0   V1.0     creat files    zz-morning     lawn mower    protocol module     
  *
  *
  ******************************************************************************
  */

#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

/* Include -------------------------------------------------------------------*/
#include <stdint.h>


/* Define --------------------------------------------------------------------*/
#define PROTOCOL_HEAD			  (0x55)
#define PROTOCOL_TAIL			(0xaa)

/* Types ---------------------------------------------------------------------*/

typedef enum _protocol_type {
	  E_PROTOCOL_NULL_CMD = 0,//--没有命令
    E_PROTOCOL_MOVING_CMD,//--运动控制包
    E_PROTOCOL_HEADING_SYCN,//--航向同步包
    E_PROTOCOL_POSITION_SYCN,//--位置同步包
    E_PROTOCOL_LOCATION_DATA,//--实时定位包
    E_PROTOCOL_MACHINE_STATE,//--机器状态包
	  E_PROTOCOL_ANSWER_CMD,//--回音接收包
	  E_PROTOCOL_REMOTE_CMD,
    E_PROTOCOL_SIZE, //--协议数量
} E_protocol_type,*E_protocol_type_p;
extern E_protocol_type_p protocol_type_p;
typedef enum _protocol_moving_cmd{
	E_PROTOCOL_MOVING_STOP = 0, //--停止命令
	E_PROTOCOL_MOVING_BRAKE, //--刹车命令
	E_PROTOCOL_MOVING_LINE_STRAIGHT,//--走直线命令
	E_PROTOCOL_TURN_TO,//--转弯命令
	E_REMOTE_PROTOCOL_STOP,
  E_REMOTE_PROTOCOL_BRAKE,
  E_REMOTE_PROTOCOL_MOVING_LINE,
  E_REMOTE_PROTOCOL_TURN,
}E_protocool_moving_cmd,*E_protocool_moving_cmd_p;
extern E_protocool_moving_cmd_p protocool_moving_cmd_p;

typedef enum _gps_location_state{
	     E_GPS_STATE_V =0,
       E_GPS_STATE_A,
}E_gps_location_state;

typedef enum _gps_location_mode{
	E_GPS_MODE_N = 0,
	E_GPS_MODE_E,
	E_GPS_MODE_A,
	E_GPS_MODE_D,
	E_GPS_MODE_F,
	E_GPS_MODE_R,
}E_gps_location_mode;

typedef enum _gps_latitude{
	E_GPS_N= 0,
	E_GPS_S,
}E_gps_latitude;

typedef enum _gps_longitude{
	E_GPS_E= 0,
	E_GPS_W,
}E_gps_longitude;

#pragma pack (1)
typedef struct __moving_cmd{
	uint8_t  id_num;
	uint8_t  moving_cmd;
	int16_t v; 
	float w;
	float s_or_angle;
}ST_moving_cmd,*ST_moving_cmd_p;

typedef struct __answer_cmd{
uint8_t ans;
}ST_answer_cmd,*ST_answer_cmd_p;

typedef struct _ekf_location{
	uint32_t  base_time;
	uint32_t  imu_time;
	double    pitch;
	double    roll;
	double    yaw;
	double    acc_y;
	double    gyo_x;
	double    gyo_y;
	double    gyo_z;
	uint32_t  gps_time;
	uint8_t   e_gps_state;
	double    latitude;
	uint8_t   e_latitude;
	double    longitude;
	uint8_t   e_longitude;
	float     gps_speed;
	uint8_t   e_gps_mode;
	uint16_t  gps_rms;
	uint32_t  odo_time;
	int32_t  odo_speed;	
}ST_ekf_location,*ST_ekf_location_p;


typedef struct _protpcol_info {
    uint8_t   head;
    uint32_t  package_len;
	  uint8_t   protocol_type;
    uint8_t*  p_data;
	  uint8_t   is_answer;
    uint8_t   crc;
	  uint8_t   tail;
} ST_protocol_info,*ST_protocol_info_p;
#pragma pack ()
extern  ST_moving_cmd      moving_cmd;
extern ST_moving_cmd_p    moving_cmd_p;
extern ST_ekf_location    ekf_location;
extern ST_ekf_location_p  ekf_location_p;
extern ST_protocol_info   protocol_info;
extern ST_protocol_info_p protocol_info_p;
extern ST_answer_cmd_p answer_cmd_p;
extern ST_answer_cmd answer_cmd;
typedef struct _protocol {
void (*get_la_and_lo_station)(char la,char lo);
void (*get_gps_station)(char station);
void (*get_gps_mode)(char mode);
void (*get_moving_data)(int16_t v,float w,float s_or_angle);
void (*get_efk_pry_data)(double pitch,double roll,double yaw);
void (*get_efk_gyo_data)(double gyo_x,double gyo_y,double gyo_z);
void (*get_efk_acc_data)(double acc_x,double acc_y,double acc_z); 
void (*get_base_time)(uint32_t base_time);
void (*get_gps_time)(uint32_t gps_time);	
void (*get_imu_time)(uint32_t imu_time);
void (*get_odo_time)(uint32_t odo_time);
void (*get_gps_data)(double latitude,double longitude,float gps_speed,uint32_t gps_rms);
void (*get_car_data)(int32_t odo_speed);
void (*send)(E_protocol_type e_protocol_type, const void* p_data, uint32_t data_len);
   // uint8_t (*init)(void);
   // uint8_t (*send)(E_protocol_type e_protocol_type, const void* p_data, uint32_t data_len);
   // uint8_t (*parse_protocol)(void);
   // uint8_t (*deinit)(void);
} ST_protocol,*ST_protocol_p;
extern ST_protocol protocol;
extern ST_protocol_p protocol_p;
/* Variables -----------------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

#endif	/* __PROTOCOL_H__ */
