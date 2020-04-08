#include "data.h"

void Data_Center_Iint(void);
void Gps_Get_Time(uint32_t t);
void Imu_Get_Time(uint32_t t);
void Car_Get_Time(uint32_t t);
void Data_center_Updata_To_Send(void);
void Get_Crc(unsigned char crc);
ST_gps_data gps_data_center = {0};
ST_gps_data_p  gps_data_center_p = &gps_data_center; 

ST_imu_data  imu_data_center = {0};
ST_imu_data_p imu_data_center_p = &imu_data_center;

ST_car_data  car_data_center = {0};
ST_car_data_p  car_data_center_p = &car_data_center;


uint8_t Car_time_updata_en = 0;

void Data_Center_Iint(void)
{
gps_data_center_p->comma1 = 'G';
gps_data_center_p->comma2 =  ','; 
gps_data_center_p->comma3 =  ',';
gps_data_center_p->comma4 =  ',';
gps_data_center_p->comma5 =  ',';
gps_data_center_p->comma6 =  ',';
gps_data_center_p->comma7 =  ',';
gps_data_center_p->comma8 =  ',';
gps_data_center_p->comma9 =  ',';
gps_data_center_p->comma10 =  ',';
gps_data_center_p->comma11 =  ',';
gps_data_center_p->comma12 =  ',';
//--字符数据需要赋值，不能写0，否则导致数据发送失败
gps_data_center_p->gps_state = 'V';
gps_data_center_p->latitude_state = 'N';
gps_data_center_p->longitude_state = 'E';
gps_data_center_p->mode = 'N';
	
imu_data_center_p->comma1 = 'I';
imu_data_center_p->comma2 = ',';
imu_data_center_p->comma3 = ',';
imu_data_center_p->comma4 = ',';
imu_data_center_p->comma5 = ',';
imu_data_center_p->comma6 = ',';
imu_data_center_p->comma7 = ',';
imu_data_center_p->comma8 = ',';
imu_data_center_p->comma9 = ',';
imu_data_center_p->comma10 = ',';
imu_data_center_p->comma11 = ',';

car_data_center_p->comma1 = 'C';
car_data_center_p->comma2 = ',';
car_data_center_p->comma3 = ',';
car_data_center_p->comma4 = ',';
}



void Gps_Get_Time(uint32_t t)
{
gps_data_center_p->time = t;
}

void Imu_Get_Time(uint32_t t)
{
imu_data_center_p->time = t;
}

void Car_Get_Time(uint32_t t)
{
car_data_center_p->time = t;
}
