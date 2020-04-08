
#include "time.h"

//-------------------------------------
static int8_t Ms_Add(void);
static uint32_t Read_Time(void);
static uint32_t Time_To_Float(void);
//----------------------------------------------
ST_time st_time ={
	0,
	0,
	0,
	0
};
ST_time_p st_time_p = &st_time;
//----------------------------------------------

ST_time_handle time_handle={
  Ms_Add,
	Read_Time,
	Time_To_Float
};
ST_time_handle_p time_handle_p = &time_handle;


//----------------------------------------------
static int8_t Ms_Add(void)
{
	st_time_p->ms++;
	if(st_time_p->ms >= 1000)
	{
	st_time_p->ms = 0;
	st_time_p->s ++;
	}
	if(st_time_p->s >=60)
	{
		st_time_p->s = 0;
		st_time_p->m ++;
	}
	if(st_time_p->m >=60)
	{
	 st_time_p->m =0;
	 st_time_p->h ++;
	}
	if(st_time_p->h>=24)
	{
	st_time_p->h=0;
	}
	return 0;
}

static uint32_t Read_Time(void)
{
  uint32_t time =Time_To_Float();
	return time;
}
//--hhmmss.ssss
static uint32_t Time_To_Float(void)
{
uint32_t temp =0;
 temp += st_time_p->h*10000000;
 temp += st_time_p->m*100000;
 temp += st_time_p->s*1000;
 temp += st_time_p->ms;
// float t = temp/(float)1000;	
 return temp;
}
