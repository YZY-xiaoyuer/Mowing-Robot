#ifndef __GPS_H__
#define __GPS_H__

//---------------------------------------------
#include "stdint.h"
#include "time.h"
//---------------------------------------------
/*
$GNRMC,
092846.400, // UTC时间，hhmmss.sss(时分秒.毫秒)格式
A, // 定位状态，A=有效定位，V=无效定位
3029.7317,N, // 纬度
10404.1784,E, // 经度
000.0, // 地面速率
183.8, // 地面航向
070417, // UTC日期
, // 磁俯角
, // 磁方向角
A*73 // 模式指示
*/
//------------------------------------------
typedef struct _GNRMC{
//---GPS数据
char     gps_state;//--gps状态位
double    latitude;//--纬度
char     latitude_state;//--南纬/北纬
double    longitude;//--经度
char     longitude_state;//--东经/西经
float    speed;//--速度
char     mode;//--模式指示	
}ST_gps_gnrmc,*ST_gps_gnrmc_p;
/*
$GNGGA,<1>,<2>,<3>,<4>,<5>,<6>,<7>,<8>,<9>,M,<10>,M,<11>,<12>*xx<CR><LF>
<7> 使用卫星数量，从00到12(第一个零也将传送)
<8> HDOP-水平精度因子，0.5到99.9，一般认为HDOP越小，质量越好。
*/
typedef struct _GNGGA{
uint32_t  number_of_satellites;//--卫星数量
float    HDOP;//--水平精度因子
}ST_gps_gngga,*ST_gps_gngga_p;

typedef struct _GNGST{
uint32_t  RMS;//--方差
}ST_gps_gngst,*ST_gps_gngst_p;
//------------------------------------------

typedef struct _GPS
{
int8_t (*gps_gnrmc_analyse)(char *buff);
}ST_gps_handle,*ST_gps_handle_p;
extern ST_gps_handle gps_handle;
extern ST_gps_handle_p gps_handle_p;
//------------------------------------------

#endif
