
/**
  ******************************************************************************
  * @file    protocol.c
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
  *
  *
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "protocol.h"
#include "crc.h"
#include "uart.h"
/* Debug Switch Section ------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
ST_protocol_info protocol_info = {0};
ST_protocol_info_p protocol_info_p = &protocol_info;

ST_moving_cmd moving_cmd = {0};
ST_moving_cmd_p moving_cmd_p = &moving_cmd;

ST_ekf_location ekf_location = {0};
ST_ekf_location_p ekf_location_p = &ekf_location;

ST_answer_cmd answer_cmd ={0x01};
ST_answer_cmd_p answer_cmd_p = & answer_cmd;
char buf[200]= {0};

/* Exported function prototypes ----------------------------------------------*/
 static void Get_La_And_Lo_Station(char la,char lo);
 static void Get_Gps_Station(char station);
 static void Get_Gps_Mode(char mode);
 static void Get_Moving_Data(int16_t v,float w,float s_or_angle);
 static void Get_Efk_PRY_Data(double pitch,double roll,double yaw);
 static void Get_Efk_GYO_Data(double gyo_x,double gyo_y,double gyo_z);
 static void Get_Efk_ACC_Data(double acc_x,double acc_y,double acc_z); 
 static void Get_Base_Time(uint32_t base_time); 
 static void Get_GPS_Time(uint32_t gps_time);
 static void Get_Imu_Time(uint32_t imu_time);
 static void Get_Odo_Time(uint32_t odo_time);
 static void Get_Gps_Data(double latitude,double longitude,float gps_speed,uint32_t gps_rms);
 static void Get_Car_Data(int32_t odo_speed);
 static void Set_Protocol_Type(E_protocol_type protocol_type);
 static void Set_Protocol_Moving_Cmd(E_protocool_moving_cmd protocool_moving_cmd);
 static void Set_Gps_Location_State(E_gps_location_state gps_location_state);
 static void Set_Gps_Location_Mode(E_gps_location_mode gps_location_module);
 static void Set_Gps_Latitude(E_gps_latitude gps_latitude);
 static void Set_Gps_Longitude(E_gps_longitude gps_longitude);
 static void Send(E_protocol_type e_protocol_type, const void* p_data, uint32_t data_len);
 /*---------------------------------------------------------------------------*/
 ST_protocol protocol = {
 Get_La_And_Lo_Station,
 Get_Gps_Station,
 Get_Gps_Mode,
 Get_Moving_Data,
 Get_Efk_PRY_Data,
 Get_Efk_GYO_Data,
 Get_Efk_ACC_Data, 
 Get_Base_Time,
 Get_GPS_Time,
 Get_Imu_Time,
 Get_Odo_Time,
 Get_Gps_Data,
 Get_Car_Data,
 Send,
};
ST_protocol_p protocol_p = &protocol;

/* Private functions ---------------------------------------------------------*/
 static inline void Get_La_And_Lo_Station(char la,char lo)
 {
   if(la == 'N')
	 Set_Gps_Latitude(E_GPS_N);
	 else if(la == 'S')
	 Set_Gps_Latitude(E_GPS_S);
	 
	 if(lo == 'E')
	 Set_Gps_Longitude(E_GPS_E);
	 else if(la == 'W')
	 Set_Gps_Longitude(E_GPS_W);
 }
  static inline void Get_Gps_Station(char station)
 {
   if(station == 'V')
	 Set_Gps_Location_State(E_GPS_STATE_V);
	 else if(station == 'A')
	 Set_Gps_Location_State(E_GPS_STATE_A);
 }
  static inline void Get_Gps_Mode(char mode)
 {
   if(mode == 'N')
	 Set_Gps_Location_Mode(E_GPS_MODE_N);
	 else if(mode == 'E')
	 Set_Gps_Location_Mode(E_GPS_MODE_E);
	 else if(mode == 'A')
	 Set_Gps_Location_Mode(E_GPS_MODE_A);
	 else if(mode == 'D')
	 Set_Gps_Location_Mode(E_GPS_MODE_D);
	 else if(mode == 'F')
	 Set_Gps_Location_Mode(E_GPS_MODE_F);
	 else if(mode == 'R')
	 Set_Gps_Location_Mode(E_GPS_MODE_R);
 }
 static inline void Get_Moving_Data(int16_t v,float w,float s_or_angle)
 {
   moving_cmd_p->v = v ;
	 moving_cmd_p->w = w ;
	 moving_cmd_p->s_or_angle = s_or_angle ;
 } 
  static inline void Get_Efk_PRY_Data(double pitch,double roll,double yaw)
 {
   ekf_location_p->pitch = pitch ;
	 ekf_location_p->roll = roll ;
	 ekf_location_p->yaw = yaw ;
 } 
 
  static inline void Get_Efk_GYO_Data(double gyo_x,double gyo_y,double gyo_z)
 {
   ekf_location_p->gyo_x = gyo_x ;
	 ekf_location_p->gyo_y = gyo_y ;
	 ekf_location_p->gyo_z = gyo_z ;
 } 
   static inline void Get_Efk_ACC_Data(double acc_x,double acc_y,double acc_z)
 {
   //ekf_location_p->acc_x = acc_x ;
	 ekf_location_p->acc_y = acc_y ;
	 //ekf_location_p->acc_z = acc_z ;
 } 
   static inline void Get_Base_Time(uint32_t base_time)
 {
    ekf_location_p->base_time = base_time;
 }
   static inline void Get_GPS_Time(uint32_t gps_time)
 {
    ekf_location_p->gps_time = gps_time;
 } 
   static inline void Get_Imu_Time(uint32_t imu_time)
 {
    ekf_location_p->imu_time = imu_time;
 } 
   static inline void Get_Odo_Time(uint32_t odo_time)
 {
    ekf_location_p->odo_time = odo_time;
 }
   static inline void Get_Gps_Data(double latitude,double longitude,float gps_speed,uint32_t gps_rms)
 {
    ekf_location_p->latitude = latitude ;
	  ekf_location_p->longitude = longitude ;
	  ekf_location_p->gps_speed = gps_speed ;
	  ekf_location_p->gps_rms = gps_speed ; 
 }
   static inline void Get_Car_Data(int32_t odo_speed)
 {
    ekf_location_p->odo_speed = odo_speed ;
 }
 static inline void Set_Protocol_Type(E_protocol_type protocol_type)
 {
   protocol_info_p->protocol_type = protocol_type ;
 } 
 static inline void Set_Protocol_Moving_Cmd(E_protocool_moving_cmd protocool_moving_cmd)
 {
   moving_cmd_p->moving_cmd = protocool_moving_cmd ;
 } 
 static inline void Set_Gps_Location_State(E_gps_location_state gps_location_state)
 {
   ekf_location_p->e_gps_state = gps_location_state ;
 } 

 static inline void Set_Gps_Location_Mode(E_gps_location_mode gps_location_module)
 {
   ekf_location_p->e_gps_mode = gps_location_module ;
 } 

 static inline void Set_Gps_Latitude(E_gps_latitude gps_latitude)
 {
   ekf_location_p->e_latitude = gps_latitude ;
 } 
 
 static inline void Set_Gps_Longitude(E_gps_longitude gps_longitude)
 {
   ekf_location_p->e_longitude = gps_longitude ;
 } 
/*打包发送*/
static void Send(E_protocol_type e_protocol_type, const void* p_data, uint32_t data_len)
{
	int32_t buf_size = 0;
	memset(buf,0,200);
	/*包头*/
	protocol_info_p->head = PROTOCOL_HEAD;
	memcpy(buf,&protocol_info_p->head,sizeof(protocol_info_p->head));
  buf_size = sizeof(protocol_info_p->head);
	/*包长*/
	protocol_info_p->package_len = data_len+sizeof(protocol_info_p->head)\
		                                     +sizeof(protocol_info_p->protocol_type)\
																				 +sizeof(protocol_info_p->crc)\
																				 +sizeof(protocol_info_p->is_answer)\
																				 +sizeof(protocol_info_p->tail)\
																				 +sizeof(protocol_info_p->package_len);	
	memcpy(buf+buf_size,&protocol_info_p->package_len,sizeof(protocol_info_p->package_len));
  buf_size += sizeof(protocol_info_p->package_len);																				 
	/*包类型*/																		 
	protocol_info_p->protocol_type = e_protocol_type;																				 
	memcpy(buf+buf_size,&protocol_info_p->protocol_type,sizeof(protocol_info_p->protocol_type));
  buf_size += sizeof(protocol_info_p->protocol_type);																			 
	/*数据包*/													 
	protocol_info_p->p_data = (uint8_t*)p_data;
	memcpy(buf+buf_size,protocol_info_p->p_data,data_len);
	buf_size += data_len;
	/*应答位*/
	protocol_info_p->is_answer = 0;
  memcpy(buf+buf_size,&protocol_info_p->is_answer,sizeof(protocol_info_p->is_answer));	
  buf_size += sizeof(protocol_info_p->is_answer);																				 
  /*CRC*/														 
	protocol_info_p->crc = crc8_chk_value((unsigned char *)(buf+sizeof(protocol_info_p->head)),buf_size-sizeof(protocol_info_p->head));
	memcpy(buf+buf_size,&protocol_info_p->crc,sizeof(protocol_info_p->crc));
	buf_size += sizeof(protocol_info_p->crc);
	/*包尾*/																		 
	protocol_info_p->tail = PROTOCOL_TAIL;
	memcpy(buf+buf_size,&protocol_info_p->tail,sizeof(protocol_info_p->tail));
	buf_size += sizeof(protocol_info_p->tail);

	HAL_UART_Transmit_DMA(&huart3,(uint8_t *)buf, buf_size); 
}

