#ifndef __TIME_H__
#define __TIME_H__
#include "stdint.h"

typedef struct _TIME{
  uint16_t ms;
	uint8_t s;
  uint8_t m;
  uint8_t h;
}ST_time,*ST_time_p; 

//------------------------------------------
typedef struct _TIME_FUNCTION{
int8_t (*ms_add)(void);
uint32_t (*read_time)(void);
uint32_t (*time_to_float)(void);
}ST_time_handle,*ST_time_handle_p;
extern ST_time_handle time_handle;
extern ST_time_handle_p time_handle_p;
//------------------------------------------
#endif
