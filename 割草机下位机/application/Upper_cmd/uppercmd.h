#ifndef __UPPERCMD_H__
#define __UPPERCMD_H__
#include <stdint.h>

typedef enum _Package_Type{
E_UPPER_NULL_CMD ,//--灭有命令
E_UPPER_MOVING_CMD,//--运动控制
E_UPPER_HEADING_SYCN,//--航向同步包
E_UPPER_POSITION_SYCN,//--位置同步包
E_UPPER_LOCATION_DATA,//--实时定位包
E_UPPER_MACHINE_STATE,//--及其状态包
E_UPPER_ANSWER_CMD,//--回应包
}E_upper_package_type,*E_upper_package_type_p;
extern E_upper_package_type_p upper_package_type_p;
typedef enum __CMD_Type{
E_UPPER_CMD_STOP,
E_UPPER_CMD_BRAKE,
E_UPPER_CMD_MOVING_LINE,
E_UPPER_CMD_TURN,
E_REMOTE_CMD_STOP,
E_REMOTE_CMD_BRAKE,
E_REMOTE_CMD_MOVING_LINE,
E_REMOTE_CMD_TURN,
}E_upper_cmd_type,*E_upper_cmd_type_p;
extern E_upper_cmd_type_p upper_cmd_type_p;
typedef struct _Upper_Cmd_Flag{
uint8_t IS_REPLY_OK;
uint8_t IS_UPDATE_CMD;
uint8_t IS_REMOTE_CMD;
}ST_upper_flag,*ST_upper_flag_p;
extern ST_upper_flag_p upper_flag_p;

typedef struct _upper{
void (*set_package_type)(E_upper_package_type package_type);
E_upper_package_type (*get_package_type)(void);
void (*set_cmd_type)(E_upper_cmd_type cmd_type);
E_upper_cmd_type (*get_cmd_type)(void);
void (*set_reply_flag)(uint8_t flag);
uint8_t (*get_reply_flag)(void);
void (*set_update_cmd_flag)(uint8_t flag);
uint8_t (*get_update_cmd_flag)(void);
void (*set_remote_flag)(uint8_t flag);
uint8_t (*get_remote_flag)(void);
}ST_upper_cmd_header,*ST_upper_cmd_header_p;
extern ST_upper_cmd_header_p upper_cmd_header_p;
#endif
