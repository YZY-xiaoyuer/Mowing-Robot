#include "gps.h"
#include "string.h"
#include "stdio.h"
#include "uart.h"
#include "stdlib.h"
#include "data.h"
//-----------------------------------------------
static int8_t Gps_GNRMC_Analyse(char *buff);
//--经纬度转化为只有度的形式
//static int8_t Latitude_Conversion(void);
//static int8_t Longitude_Conversion(void);
/*
//--纬度转换
static int8_t Latitude_Conversion(int32_t *d,int32_t *f,int32_t *m);
//--经度转换
static int8_t Longitude_Conversion(int32_t *d,int32_t *f,int32_t *m);
*/
//-----------------------------------------------
//--保存确定值
/*
static ST_gps_gnrmc gps_gnrmc={0};
static ST_gps_gnrmc_p gps_gnrmc_p = &gps_gnrmc;

static ST_gps_gngga gps_gngga={0};
static ST_gps_gngga_p gps_gngga_p = &gps_gngga;

static ST_gps_gngst gps_gngst={0};
static ST_gps_gngst_p gps_gngst_p = &gps_gngst;
*/

//-----------------------------------------------
ST_gps_handle gps_handle = {
Gps_GNRMC_Analyse,
};
ST_gps_handle_p gps_handle_p = &gps_handle;
//-----------------------------------------------

static int8_t Gps_GNRMC_Analyse(char *buff)
{

char *ptr = NULL;
	if(buff == NULL) return -1;
	if(strlen(buff)<10)return -1;
	/*如果buff字符串中包含"$GNRMC"，则将$GNRMC的地址赋值给ptr*/
	if(NULL == (ptr = strstr(buff,"$GNRMC")))return -1;
	/*从字符串中分离出对应格式的数据*/
	uint32_t a,b;

	//-         $GNRMC,,200919,,,A,V*17
	sscanf(ptr,"$GNRMC,%f,%c,%lf,%c,%lf,%c,%f,,%d,,,%c,",
	           &(gps_data_center_p->UTC),
	//--不能用float类型接收，会导致小数位不准确，使用double类型可以
	           &(gps_data_center_p->gps_state),//--gps状态位-      %c
	           &(gps_data_center_p->latitude),//--纬度 -           %f
	           &(gps_data_center_p->latitude_state),//--南纬/北纬- %c
	           &(gps_data_center_p->longitude),//--经度-           %f
	           &(gps_data_center_p->longitude_state),//--东经/西经 %c
	           &(gps_data_center_p->speed),//--速度                %f
	           &b,
	           &(gps_data_center_p->mode) //--模式指示             %c
	);
	/*
	//--经纬度转化为只有度的形式
	//Latitude_Conversion();
	//Longitude_Conversion();
		if(NULL == (ptr = strstr(buff,"$GNGGA")))
		return -1;
	//$GNGGA,033642.00,2237.44292,N,11402.39192,E,1,08,0.97,100.7,M,-2.5,M,,*53
  float temp1,temp2,temp3;
	char temp4,temp5;
		sscanf(ptr,"$GNGGA,%d.00,%f,%c,%f,%c,%d,%d,%f,%f,",
		         &a,
		         &temp1,
		         &temp4,
		         &temp2,
		         &temp5,
		         &b,
	           &c,//--卫星数量
	           &temp3,//--水平精度因子
		         &(gps_data_center_p->alt)//--海拔高度
	);	
*/
		if(NULL == (ptr = strstr(buff,"$GNGST")))return -1;		
   //---$GNGST,060644.80,148697,,,,1.1,1.0,2.3*44
				sscanf(ptr,"$GNGST,%d.00,%d,",
		         &a,
		         &gps_data_center_p->rms//--方差
	          );	

   return 0;
}
/*
//--经纬度转化为只有度的形式
static int8_t Latitude_Conversion(void)
{
 uint32_t latitude_temp = gps_data_center_p->latitude;
	latitude_temp = latitude_temp /100;
	double latitude_temp1 = (gps_data_center_p->latitude/100 - latitude_temp)/60;
	gps_data_center_p->latitude = (double)latitude_temp + latitude_temp1;
	return 0;
}
static int8_t Longitude_Conversion(void)
{
 uint32_t longitude_temp = gps_data_center_p->longitude;
	longitude_temp = longitude_temp /100;
	double longitude_temp1 = (gps_data_center_p->longitude/100 - longitude_temp)/60;
	gps_data_center_p->longitude = (double)longitude_temp + longitude_temp1;
	return 0;
}
*/
/*
//--纬度转换
static int8_t Latitude_Conversion(int32_t *d,int32_t *f,int32_t *m)
{
 *d = (int32_t)gps_gnrmc_p->latitude/100;
 *f = (int32_t)(gps_gnrmc_p->latitude-*d*100);
 *m = (int32_t)(((gps_gnrmc_p->latitude-*d*100)-*f)*60);
	return 0;
}
//--经度转换
static int8_t Longitude_Conversion(int32_t *d,int32_t *f,int32_t *m)
{
 *d = (int32_t)gps_gnrmc_p->longitude/100;
 *f = (int32_t)(gps_gnrmc_p->longitude-*d*100);
 *m = (int32_t)(((gps_gnrmc_p->longitude-*d*100)-*f)*60);
 return 0;
}
*/

