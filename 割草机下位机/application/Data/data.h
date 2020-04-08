#ifndef __DATA_H__
#define __DATA_H__

/*===========ͷ�ļ�==============*/

#include "stdint.h"

//---1�ֽڶ���
//#pragma pack(1)//--����GPS���ݺ�Ҫ��1�ֽڶ�����������ݣ�
//--��ΪGPS���ݳ��Ȳ��̶����������ʱ��ᵼ���ڴ����

typedef struct _gps_data{
char     comma1;    //--G  gps ��ͷ
char     comma2;   //--����
uint32_t time; 
char     comma3;   
float UTC;
char     comma12; 
char     gps_state;//--gps״̬
char     comma4;   
double    latitude;//--γ��
char     comma5;   
char     latitude_state;//--γ�ȷ���
char     comma6;   
double    longitude;//--����
char     comma7;
char     longitude_state;//--���ȷ���
char     comma8; 
float    speed;//--�ٶ�
char     comma9; 
char     mode;
char     comma10; 
uint32_t  rms;//--��λ����
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
