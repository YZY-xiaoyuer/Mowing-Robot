#ifndef __QUEUE_H__
#define __QUEUE_H__	
#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
//--运动控制数据队列
typedef  struct __moving_cmd_queue{
uint8_t package_id;
uint8_t moving_cmd;
int16_t v;
float w;
float s_or_angle;
struct  __moving_cmd_queue * next;
}ST_moving_cmd_queue,*ST_moving_cmd_queue_p;


typedef struct __queue_handle{
void (*queue_init)(void);
ST_moving_cmd_queue_p (*creat_moving_cmd_node)(uint8_t package_id,uint8_t moving_cmd,int16_t v,float w,float s_or_angle);
void (*insert_to_moving_cmd_list)(ST_moving_cmd_queue_p H);
uint8_t (*get_node_from_moving_list_head)(uint8_t *moving_cmd_p,int16_t *v_p,float *w_p,float *s_or_angle_p);
void (*free_moving_cmd_list)(void);
void (*free_all_head_node)(void);
uint8_t (*list_is_null)(void);
uint8_t (*get_last_package_id)(void);
uint8_t (*get_queue_head_cmd_type)(void);
void (*update_remote_cmd)(uint8_t moving_cmd,int16_t v_p,float w_p);
uint8_t (*get_remote_cmd)(uint8_t *moving_cmd_p,int16_t *v_p,float *w_p);
 /*更新轨迹跟踪命令，只有一个节点的队列*/
void (*update_track_tracking_cmd)(uint8_t moving_cmd,uint8_t id,int16_t v_p,float w_p,float s_or_angle_p);
/*从队列链表头部取走数据*/
uint8_t (*get_track_tracking_cmd)(uint8_t *moving_cmd_p,int16_t *v_p,float *w_p,float *s_or_angle_p);
}ST_queue_handle,*ST_queue_handle_p;

extern ST_queue_handle queue_handle;
extern ST_queue_handle_p queue_handle_p;



#endif
