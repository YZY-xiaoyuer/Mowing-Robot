#ifndef __USERCMD_H
#define __USERCMD_H

#include "usart.h"
#include "shell.h"

#define  DISPLAY_GPS  (1) //--��ʾGPS����
#define DISPLAY_IMU  (2) //--��ʾIMU����
#define DISPLAY_ALL  (3) //--��ʾ��������
#define DISPLAY_END  (4) //--��ʾ��������
#define DISPLAY_CAR_DATA (13) //--С������
#define DISPLAY_NO_ANALYZE (14)//--��ʾGPSԭʼ����
#define DISPLAY_TWO  (15) //--ͬʱ��ʾGPSԭʼ���ݺͽ����������
#define DISPLAY_CA_BEGIN  (17) //--����ͬ��
#define DISPLAY_CA  (18) //--����ͬ��
#define DISPLAY_CA_END  (19) //--����ͬ��
#define DISPLAY_HA_BEGIN  (20)//--����ͬ��
#define DISPLAY_HA  (21)//--����ͬ��
#define DISPLAY_HA_END  (22)//--����ͬ��
#define DISPLAY_RL_BEGIN   (23) //--ʵʱ��λ
#define DISPLAY_RL   (24) //--ʵʱ��λ
#define DISPLAY_RL_END   (25) //--ʵʱ��λ
extern uint8_t DISPLAY_STATION;//--��ʾ״̬

#define REMOTE_LINE_W  (5) //--ң��ֱ������
#define REMOTE_LINE_S  (6) //--ң��ֱ������
#define REMOTE_TURN_A  (7) //--ң��ת��
#define REMOTE_TURN_D  (8) //--ң��ת��
#define SET_CMD_LINE  (9) //--��������ֱ������
#define SET_CMD_TUEN  (10) //--��������ת��
#define CMD_CAR_STOP  (11) //--С��ֹͣ
#define CMD_CAR_BRAKE  (12) //--С��ֹͣ
#define CMD_CAR_STOP_TO_DISPLAY (16)//--ֹͣ��ʱ����ʾ
#define CMD_TURN_8  (26)//--��8��
extern uint8_t CAR_STATION; //--С��״̬

uint8_t Cmd_Reboot(char * p0, char * p1, char * p2, char * p3);
uint8_t Cmd_Power_Off(char * p0, char * p1, char * p2, char * p3);
uint8_t Cmd_Remote(char * p0, char * p1, char * p2, char * p3);
uint8_t Cmd_Display(char * p0, char * p1, char * p2, char * p3);
uint8_t Cmd_Set_Pid(char * p0, char * p1, char * p2, char * p3);
uint8_t Cmd_Turn_Test(char * p0, char * p1, char * p2, char * p3);


extern uint8_t cmd_remote_rcv;


#endif

