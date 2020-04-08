#ifndef __USERCMD_H
#define __USERCMD_H

#include "usart.h"
#include "shell.h"

#define  DISPLAY_GPS  (1) //--显示GPS数据
#define DISPLAY_IMU  (2) //--显示IMU数据
#define DISPLAY_ALL  (3) //--显示所有数据
#define DISPLAY_END  (4) //--显示所有数据
#define DISPLAY_CAR_DATA (13) //--小车数据
#define DISPLAY_NO_ANALYZE (14)//--显示GPS原始数据
#define DISPLAY_TWO  (15) //--同时显示GPS原始数据和解析后的数据
#define DISPLAY_CA_BEGIN  (17) //--坐标同步
#define DISPLAY_CA  (18) //--坐标同步
#define DISPLAY_CA_END  (19) //--坐标同步
#define DISPLAY_HA_BEGIN  (20)//--航向同步
#define DISPLAY_HA  (21)//--航向同步
#define DISPLAY_HA_END  (22)//--航向同步
#define DISPLAY_RL_BEGIN   (23) //--实时定位
#define DISPLAY_RL   (24) //--实时定位
#define DISPLAY_RL_END   (25) //--实时定位
extern uint8_t DISPLAY_STATION;//--显示状态

#define REMOTE_LINE_W  (5) //--遥控直线行走
#define REMOTE_LINE_S  (6) //--遥控直线行走
#define REMOTE_TURN_A  (7) //--遥控转弯
#define REMOTE_TURN_D  (8) //--遥控转弯
#define SET_CMD_LINE  (9) //--设置命令直线行走
#define SET_CMD_TUEN  (10) //--设置命令转弯
#define CMD_CAR_STOP  (11) //--小车停止
#define CMD_CAR_BRAKE  (12) //--小车停止
#define CMD_CAR_STOP_TO_DISPLAY (16)//--停止的时候显示
#define CMD_TURN_8  (26)//--画8字
extern uint8_t CAR_STATION; //--小车状态

uint8_t Cmd_Reboot(char * p0, char * p1, char * p2, char * p3);
uint8_t Cmd_Power_Off(char * p0, char * p1, char * p2, char * p3);
uint8_t Cmd_Remote(char * p0, char * p1, char * p2, char * p3);
uint8_t Cmd_Display(char * p0, char * p1, char * p2, char * p3);
uint8_t Cmd_Set_Pid(char * p0, char * p1, char * p2, char * p3);
uint8_t Cmd_Turn_Test(char * p0, char * p1, char * p2, char * p3);


extern uint8_t cmd_remote_rcv;


#endif

