#include "uppercmd.h"

E_upper_package_type upper_package_type={0};
E_upper_package_type_p upper_package_type_p = &upper_package_type;
E_upper_cmd_type upper_cmd_type = {0};
E_upper_cmd_type_p upper_cmd_type_p = &upper_cmd_type;
ST_upper_flag upper_flag = {0};
ST_upper_flag_p upper_flag_p = &upper_flag;
static void Set_Package_Type(E_upper_package_type package_type);
static E_upper_package_type Get_Package_Type(void);
static void Set_Cmd_Type(E_upper_cmd_type cmd_type);
static E_upper_cmd_type Get_Cmd_Type(void);
static void Set_Reply_Flag(uint8_t flag);
static uint8_t Get_Reply_flag(void);
static void Set_Update_Cmd_Flag(uint8_t flag);
static uint8_t Get_Update_Cmd_flag(void);
static void Set_Remote_flag(uint8_t flag);
static uint8_t Get_Remote_flag(void);
ST_upper_cmd_header upper_cmd_header = {
Set_Package_Type,
Get_Package_Type,
Set_Cmd_Type,
Get_Cmd_Type,
Set_Reply_Flag,
Get_Reply_flag,
Set_Update_Cmd_Flag,
Get_Update_Cmd_flag,
Set_Remote_flag,
Get_Remote_flag,	
};
ST_upper_cmd_header_p upper_cmd_header_p = &upper_cmd_header;
static inline void Set_Package_Type(E_upper_package_type package_type)
{
 upper_package_type = package_type;
}
static inline E_upper_package_type Get_Package_Type(void){
	return upper_package_type;
}
static void Set_Cmd_Type(E_upper_cmd_type cmd_type)
{
 upper_cmd_type = cmd_type;
}
static inline E_upper_cmd_type Get_Cmd_Type(void){
	return upper_cmd_type;
}
static inline void Set_Reply_Flag(uint8_t flag){
 upper_flag_p->IS_REPLY_OK = flag;
}
static inline uint8_t Get_Reply_flag(void){
return upper_flag_p->IS_REPLY_OK;
}
static inline void Set_Update_Cmd_Flag(uint8_t flag){
 upper_flag_p->IS_UPDATE_CMD = flag;
}
static inline uint8_t Get_Update_Cmd_flag(void){
return upper_flag_p->IS_UPDATE_CMD;
}

static inline void Set_Remote_flag(uint8_t flag){
upper_flag_p->IS_REMOTE_CMD = flag;
}
static inline uint8_t Get_Remote_flag(void){
return upper_flag_p->IS_REMOTE_CMD;
}