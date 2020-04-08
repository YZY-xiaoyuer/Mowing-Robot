#include "usercmd.h"
#include "shell.h"
#include "gpio.h"
#include <cstdlib>
#include <cstdio>
#include <stdlib.h>
#include "car.h"
#include "pid.h"

uint8_t cmd_remote_rcv =0;


uint8_t DISPLAY_STATION = 0;//--显示状态
uint8_t CAR_STATION = 0; //--小车状态
uint8_t Cmd_Reboot(char * p0, char * p1, char * p2, char * p3)
{
	 st_usart_shell.output((uint8_t*)"\n\r\n正在重启\r\n", strlen("\n\r\n正在重启\r\n"));
	 HAL_Delay(500);
    NVIC_SystemReset();
    return 0;
}

uint8_t Cmd_Power_Off(char * p0, char * p1, char * p2, char * p3)
{
	 st_usart_shell.output((uint8_t*)"\n\r\n警告：即将断电\r\n", strlen("\n\r\n警告：即将断电\r\n"));
	  HAL_Delay(1000);
    HAL_GPIO_WritePin(POWER_12V_HOLD_ON_GPIO_Port, POWER_12V_HOLD_ON_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(POWER_28V_HOLD_ON_GPIO_Port, POWER_28V_HOLD_ON_Pin, GPIO_PIN_RESET);
    return 0;
}

uint8_t Cmd_Remote(char * p0, char * p1, char * p2, char * p3)
{
	
	static int count =0;
    switch (*p0) {
        case 'h':
            st_usart_shell.output((uint8_t*)" s：开始遥控，esc：退出遥控\r\n>>", strlen(" s：开始遥控，esc：退出遥控\r\n>>"));
            break;

        case 's':
            st_usart_shell.set_type(SHELL_REMOTE);
            st_usart_shell.output((uint8_t*)"\r\n                             键盘 :w s a d q\r\n", strlen("\r\n                             键盘 :w s a d q\r\n"));

            while(1) {
                HAL_UART_Receive_IT(&huart1, &cmd_remote_rcv, 1);
               	osDelay(20);

                if(cmd_remote_rcv == KEY_ESC) {
                    st_usart_shell.set_type(SHELL_CONSOLE);
                    st_usart_shell.output((uint8_t *)"\r\n->退出\r\n>>", strlen("\r\n->退出\r\n>>"));
                    HAL_UART_Receive_IT(&huart1, p_usart_shell_rcv_buf, 1);
                   	CAR_STATION = CMD_CAR_STOP;//--停止
									
									break;
                }
               switch (cmd_remote_rcv) {
                    case 'w':
											   count = 0;
										     CAR_STATION = REMOTE_LINE_W;//--遥控直线行走
                        break;
                    case 's':
											count = 0;
										    CAR_STATION = REMOTE_LINE_S;//--遥控直线行走
                        break;
                    case 'a':
											  count = 0;
										      CAR_STATION = REMOTE_TURN_A;//--遥控转弯
                        break;
                    case 'd':
											count = 0;
										      CAR_STATION = REMOTE_TURN_D;//--遥控转弯
                        break;
										case 'q':
											count = 0;
											   CAR_STATION = CMD_CAR_STOP;//--停止
									  break;
                    default :												
										  count ++;
										  if(count >= 25)
											 {
                   	   CAR_STATION = CMD_CAR_STOP;//--停止
											}
											
                      break;
									
                }
								 
					 cmd_remote_rcv = 0;
            }

            break;
    }
   
    return 0;
		
}

uint8_t Cmd_Display(char * p0, char * p1, char * p2, char * p3)
{
 switch (*p0) {
        case 'h':
					st_usart_shell.output((uint8_t*)"\r\n g:显示GPS数据\r\ni：显示IMU数据\r\nc:显示小车数据\r\na :显示所有数据\r\nn:显示GPS原始数据\r\nt:显示GPS原始和解析后的数据\r\ne:停止所有显示\r\nC：CA停止\r\nH:HA停止\r\nR：RL停止\r\n>>",
            strlen("\r\n g:显示GPS数据\r\ni：显示IMU数据\r\nc:显示小车数据\r\na :显示所有数据\r\nn:显示GPS原始数据\r\nt:显示GPS原始和解析后的数据\r\ne:停止所有显示\r\nC：CA停止\r\nH:HA停止\r\nR：RL停止\r\n>>"));
            break;
        case 'g':
               DISPLAY_STATION = DISPLAY_GPS;//--显示状态
            break;
        case 'i':				
				DISPLAY_STATION = DISPLAY_IMU;//--显示状态
            break;
        case 'a':			
				   DISPLAY_STATION = DISPLAY_ALL;//--显示状态
            break;
				case 'c':
					  DISPLAY_STATION = DISPLAY_CAR_DATA;
				 break;
//        case 'e':
//					if(DISPLAY_STATION == DISPLAY_CA)
//						 DISPLAY_STATION = DISPLAY_CA_END;
//					else if(DISPLAY_STATION == DISPLAY_HA)
//						 DISPLAY_STATION = DISPLAY_HA_END;
//					else if(DISPLAY_STATION == DISPLAY_RL)
//						 DISPLAY_STATION = DISPLAY_RL_END;
//					else
//				   DISPLAY_STATION = DISPLAY_END;//--显示状态			
//            break;
		      case 'e':
				   DISPLAY_STATION = DISPLAY_END;//--显示状态			
           break;
				case 'C':
				   DISPLAY_STATION = DISPLAY_CA_END;//--显示状态			
           break;
				case 'H':
				   DISPLAY_STATION = DISPLAY_HA_END;//--显示状态			
           break;
			 case 'R':
				   DISPLAY_STATION = DISPLAY_RL_END;//--显示状态			
           break;
        case 'n':
				 DISPLAY_STATION = DISPLAY_NO_ANALYZE;//--显示状态			
            break;
        case 't':
				 DISPLAY_STATION = DISPLAY_TWO;//--显示状态			
            break;
    }

    return 0;
}

uint8_t Cmd_Set_Pid(char * p0, char * p1, char * p2, char * p3)
{
    switch (*p0) {
        case 'h':
            st_usart_shell.output((uint8_t*)"\r\n 设置PID参数. 例子: pid l 2 1 1 .表示设置PID参数为 kp 2,ki 1,kd 1\r\n>>",
            strlen("\r\n 设置PID参数. 例子: pid l 2 1 1 .表示设置PID参数为 kp 2,ki 1,kd 1\r\n>>"));            break;

        case 'l': {
                int temp1_kp = 0;
                int temp2_ki = 0;
                int temp3_kd = 0;
                sscanf(p1, "%d", &temp1_kp);
                sscanf(p2, "%d", &temp2_ki);
                sscanf(p3, "%d", &temp3_kd);
                car_pid_handle_p->car_walk_distance_pid(temp1_kp,temp2_ki,temp3_kd);
					   char buf[30];		
			
	        sprintf(buf,"\r\nKP:%d  KI:%d   KD:%d\r\n",temp1_kp,temp2_ki,temp3_kd);
	        st_usart_shell.output((uint8_t *)buf,strlen(buf));
                break;
            }
    }

    return 0;
}
uint8_t Cmd_Turn_Test(char * p0, char * p1, char * p2, char * p3)
{ 

	switch (*p0) {
		    case 'h':
					st_usart_shell.output((uint8_t*)"\r\nw/s 前进/后退:w 200 1000(w v s)\r\na/d 左转右转：a 0 1 90(a v w A[角度])\r\nq 停车\r\nr:坐标同步（停止并发送5秒数据)\r\nc:航向同步（走一米）\r\nt:实时定位(4 e 停止输出)\r\n8 (8 v w q):走8字\r\n>>",
            strlen("\r\nw/s 前进/后退:w 200 1000(w v s)\r\na/d 左转右转：a 0 1 90(a v w A[角度])\r\nq 停车\r\nr:坐标同步（停止并发送5秒数据)\r\nc:航向同步（走一米）\r\nt:实时定位(4 e 停止输出)\r\n8 (8 v w q):走8字\r\n>>"));            
				break;
		    case 'q': 
				       CAR_STATION = CMD_CAR_STOP;//--停止
					break;

				case 'w':{//--前进

					      int temp1 = 0;//--0.2m/s
                int temp2 = 0;//--1m
               // int temp3 = 0;
                sscanf(p1, "%d", &temp1);
                sscanf(p2, "%d", &temp2);
               // sscanf(p3, "%d", &temp3_kd);
					   car_handle_p->car_walk_straight_line_updata(temp1,temp2);
					  CAR_STATION = SET_CMD_LINE;//--设置命令直线行走
						break;			
				}
				case 's':{//--后退
				//	printf("=======2=========\r\n");
					      int temp1 = 0;
                int temp2 = 0;
               // int temp3 = 0;
                sscanf(p1, "%d", &temp1);
                sscanf(p2, "%d", &temp2);
               // sscanf(p3, "%d", &temp3_kd);
					      car_handle_p->car_walk_straight_line_updata(-temp1,temp2);
					  CAR_STATION = SET_CMD_LINE;//--设置命令直线行走
						break;			
				}
				case 'a':{//--左转
					//printf("=======3=========\r\n");
					      int temp1 = 0;
                float temp2 = 0;
                float temp3 = 0;
                sscanf(p1, "%d", &temp1);
                sscanf(p2, "%f", &temp2);
                sscanf(p3, "%f", &temp3);
					   car_handle_p->car_turn_updata(temp1,temp2,temp3);
					 CAR_STATION = SET_CMD_TUEN;//--设置命令转弯
						break;			
				}
				case 'd':{//--右转
					//printf("=======4=========\r\n");
					      int temp1 = 0;
                float temp2 = 0;
                float temp3 = 0;
                sscanf(p1, "%d", &temp1);
                sscanf(p2, "%f", &temp2);
                sscanf(p3, "%f", &temp3);
					   car_handle_p->car_turn_updata(temp1,-temp2,temp3);
					  CAR_STATION = SET_CMD_TUEN;//--设置命令转弯
						break;			
				}
				case '8':{
					  		int temp1 = 0;
                float temp2 = 0;
                int temp3 = 0;
                sscanf(p1, "%d", &temp1);//--线速度
                sscanf(p2, "%f", &temp2);//--角度
                sscanf(p3, "%d", &temp3);//--圈数
					  car_handle_p->car_turn_8_updata(temp1,temp2,temp3);
				    CAR_STATION = CMD_TURN_8;
					break;}
			case 'r': //--坐标同步
                DISPLAY_STATION= DISPLAY_CA_BEGIN;//--停止发送5秒数据
                break;			
        case 'c': //--航向同步
                 DISPLAY_STATION = DISPLAY_HA_BEGIN;//--走一米
				         car_handle_p->car_walk_straight_line_updata(300,1000);
				         CAR_STATION = SET_CMD_LINE;//--设置命令直线行走
                break;	
        case 't': //--实时定位
                DISPLAY_STATION = DISPLAY_RL_BEGIN;//--遥控
                break;	
    }

    return 0;
}
