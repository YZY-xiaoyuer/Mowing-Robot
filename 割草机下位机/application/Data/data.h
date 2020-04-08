#ifndef __DATA_H__
#define __DATA_H__

/*===========头文件==============*/

#include "stdint.h"

//---1字节对齐
//#pragma pack(1)//--解析GPS数据后不要用1字节对齐来存放数据，
//--因为GPS数据长度不固定，在运算的时候会导致内存错误

typedef struct _gps_data{
char     comma1;    //--G  gps 包头
char     comma2;   //--逗号
uint32_t time; 
char     comma3;   
float UTC;
char     comma12; 
char     gps_state;//--gps状态
char     comma4;   
double    latitude;//--纬度
char     comma5;   
char     latitude_state;//--纬度符号
char     comma6;   
double    longitude;//--经度
char     comma7;
char     longitude_state;//--经度符号
char     comma8; 
float    speed;//--速度
char     comma9; 
char     mode;
char     comma10; 
uint32_t  rms;//--定位精度
char   comma11;
}ST_gps_data,*ST_gps_data_p;
extern ST_gps_data gps_data_center;
extern ST_gps_data_p  gps_data_center_p; 

typedef struct _imu_data{
 char  comma1;//--I
 uint32_t time;
	char comma2;
 float pitch;
char  comma3;
 float roll;
char  comma4;
 float yaw;
	char  comma5;
 float acc_x;
	char  comma6;
 float acc_y;
	char  comma7;
 float acc_z;
	char  comma8;
 float gyo_x;
	char  comma9;
 float gyo_y;
	char  comma10;
 float gyo_z;
char  comma11;
}ST_imu_data,*ST_imu_data_p;
extern ST_imu_data  imu_data_center;
extern ST_imu_data_p imu_data_center_p;

typedef struct _car_data{
char  comma1;
uint32_t time;
char comma2;
int32_t speed;
char  comma3;
uint32_t distance;
char  comma4;
}ST_car_data,*ST_car_data_p;
extern ST_car_data  car_data_center;
extern ST_car_data_p  car_data_center_p;



extern uint8_t Car_time_updata_en;
extern void Data_Center_Iint(void);
extern void Gps_Get_Time(uint32_t t);
extern void Imu_Get_Time(uint32_t t);
extern void Car_Get_Time(uint32_t t);
extern void Data_center_Updata_To_Send(void);
extern void Get_Crc(unsigned char crc);
#endif
