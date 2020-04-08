#include "usercmd.h"
#include "shell.h"
#include "gpio.h"
#include <cstdlib>
#include <cstdio>
#include <stdlib.h>
#include "car.h"
#include "pid.h"

uint8_t cmd_remote_rcv =0;


uint8_t DISPLAY_STATION = 0;//--��ʾ״̬
uint8_t CAR_STATION = 0; //--С��״̬
uint8_t Cmd_Reboot(char * p0, char * p1, char * p2, char * p3)
{
	 st_usart_shell.output((uint8_t*)"\n\r\n��������\r\n", strlen("\n\r\n��������\r\n"));
	 HAL_Delay(500);
    NVIC_SystemReset();
    return 0;
}

uint8_t Cmd_Power_Off(char * p0, char * p1, char * p2, char * p3)
{
	 st_usart_shell.output((uint8_t*)"\n\r\n���棺�����ϵ�\r\n", strlen("\n\r\n���棺�����ϵ�\r\n"));
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
            st_usart_shell.output((uint8_t*)" s����ʼң�أ�esc���˳�ң��\r\n>>", strlen(" s����ʼң�أ�esc���˳�ң��\r\n>>"));
            break;

        case 's':
            st_usart_shell.set_type(SHELL_REMOTE);
            st_usart_shell.output((uint8_t*)"\r\n                             ���� :w s a d q\r\n", strlen("\r\n                             ���� :w s a d q\r\n"));

            while(1) {
                HAL_UART_Receive_IT(&huart1, &cmd_remote_rcv, 1);
               	osDelay(20);

                if(cmd_remote_rcv == KEY_ESC) {
                    st_usart_shell.set_type(SHELL_CONSOLE);
                    st_usart_shell.output((uint8_t *)"\r\n->�˳�\r\n>>", strlen("\r\n->�˳�\r\n>>"));
                    HAL_UART_Receive_IT(&huart1, p_usart_shell_rcv_buf, 1);
                   	CAR_STATION = CMD_CAR_STOP;//--ֹͣ
									
									break;
                }
               switch (cmd_remote_rcv) {
                    case 'w':
											   count = 0;
										     CAR_STATION = REMOTE_LINE_W;//--ң��ֱ������
                        break;
                    case 's':
											count = 0;
										    CAR_STATION = REMOTE_LINE_S;//--ң��ֱ������
                        break;
                    case 'a':
											  count = 0;
										      CAR_STATION = REMOTE_TURN_A;//--ң��ת��
                        break;
                    case 'd':
											count = 0;
										      CAR_STATION = REMOTE_TURN_D;//--ң��ת��
                        break;
										case 'q':
											count = 0;
											   CAR_STATION = CMD_CAR_STOP;//--ֹͣ
									  break;
                    default :												
										  count ++;
										  if(count >= 25)
											 {
                   	   CAR_STATION = CMD_CAR_STOP;//--ֹͣ
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
					st_usart_shell.output((uint8_t*)"\r\n g:��ʾGPS����\r\ni����ʾIMU����\r\nc:��ʾС������\r\na :��ʾ��������\r\nn:��ʾGPSԭʼ����\r\nt:��ʾGPSԭʼ�ͽ����������\r\ne:ֹͣ������ʾ\r\nC��CAֹͣ\r\nH:HAֹͣ\r\nR��RLֹͣ\r\n>>",
            strlen("\r\n g:��ʾGPS����\r\ni����ʾIMU����\r\nc:��ʾС������\r\na :��ʾ��������\r\nn:��ʾGPSԭʼ����\r\nt:��ʾGPSԭʼ�ͽ����������\r\ne:ֹͣ������ʾ\r\nC��CAֹͣ\r\nH:HAֹͣ\r\nR��RLֹͣ\r\n>>"));
            break;
        case 'g':
               DISPLAY_STATION = DISPLAY_GPS;//--��ʾ״̬
            break;
        case 'i':				
				DISPLAY_STATION = DISPLAY_IMU;//--��ʾ״̬
            break;
        case 'a':			
				   DISPLAY_STATION = DISPLAY_ALL;//--��ʾ״̬
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
//				   DISPLAY_STATION = DISPLAY_END;//--��ʾ״̬			
//            break;
		      case 'e':
				   DISPLAY_STATION = DISPLAY_END;//--��ʾ״̬			
           break;
				case 'C':
				   DISPLAY_STATION = DISPLAY_CA_END;//--��ʾ״̬			
           break;
				case 'H':
				   DISPLAY_STATION = DISPLAY_HA_END;//--��ʾ״̬			
           break;
			 case 'R':
				   DISPLAY_STATION = DISPLAY_RL_END;//--��ʾ״̬			
           break;
        case 'n':
				 DISPLAY_STATION = DISPLAY_NO_ANALYZE;//--��ʾ״̬			
            break;
        case 't':
				 DISPLAY_STATION = DISPLAY_TWO;//--��ʾ״̬			
            break;
    }

    return 0;
}

uint8_t Cmd_Set_Pid(char * p0, char * p1, char * p2, char * p3)
{
    switch (*p0) {
        case 'h':
            st_usart_shell.output((uint8_t*)"\r\n ����PID����. ����: pid l 2 1 1 .��ʾ����PID����Ϊ kp 2,ki 1,kd 1\r\n>>",
            strlen("\r\n ����PID����. ����: pid l 2 1 1 .��ʾ����PID����Ϊ kp 2,ki 1,kd 1\r\n>>"));            break;

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
					st_usart_shell.output((uint8_t*)"\r\nw/s ǰ��/����:w 200 1000(w v s)\r\na/d ��ת��ת��a 0 1 90(a v w A[�Ƕ�])\r\nq ͣ��\r\nr:����ͬ����ֹͣ������5������)\r\nc:����ͬ������һ�ף�\r\nt:ʵʱ��λ(4 e ֹͣ���)\r\n8 (8 v w q):��8��\r\n>>",
            strlen("\r\nw/s ǰ��/����:w 200 1000(w v s)\r\na/d ��ת��ת��a 0 1 90(a v w A[�Ƕ�])\r\nq ͣ��\r\nr:����ͬ����ֹͣ������5������)\r\nc:����ͬ������һ�ף�\r\nt:ʵʱ��λ(4 e ֹͣ���)\r\n8 (8 v w q):��8��\r\n>>"));            
				break;
		    case 'q': 
				       CAR_STATION = CMD_CAR_STOP;//--ֹͣ
					break;

				case 'w':{//--ǰ��

					      int temp1 = 0;//--0.2m/s
                int temp2 = 0;//--1m
               // int temp3 = 0;
                sscanf(p1, "%d", &temp1);
                sscanf(p2, "%d", &temp2);
               // sscanf(p3, "%d", &temp3_kd);
					   car_handle_p->car_walk_straight_line_updata(temp1,temp2);
					  CAR_STATION = SET_CMD_LINE;//--��������ֱ������
						break;			
				}
				case 's':{//--����
				//	printf("=======2=========\r\n");
					      int temp1 = 0;
                int temp2 = 0;
               // int temp3 = 0;
                sscanf(p1, "%d", &temp1);
                sscanf(p2, "%d", &temp2);
               // sscanf(p3, "%d", &temp3_kd);
					      car_handle_p->car_walk_straight_line_updata(-temp1,temp2);
					  CAR_STATION = SET_CMD_LINE;//--��������ֱ������
						break;			
				}
				case 'a':{//--��ת
					//printf("=======3=========\r\n");
					      int temp1 = 0;
                float temp2 = 0;
                float temp3 = 0;
                sscanf(p1, "%d", &temp1);
                sscanf(p2, "%f", &temp2);
                sscanf(p3, "%f", &temp3);
					   car_handle_p->car_turn_updata(temp1,temp2,temp3);
					 CAR_STATION = SET_CMD_TUEN;//--��������ת��
						break;			
				}
				case 'd':{//--��ת
					//printf("=======4=========\r\n");
					      int temp1 = 0;
                float temp2 = 0;
                float temp3 = 0;
                sscanf(p1, "%d", &temp1);
                sscanf(p2, "%f", &temp2);
                sscanf(p3, "%f", &temp3);
					   car_handle_p->car_turn_updata(temp1,-temp2,temp3);
					  CAR_STATION = SET_CMD_TUEN;//--��������ת��
						break;			
				}
				case '8':{
					  		int temp1 = 0;
                float temp2 = 0;
                int temp3 = 0;
                sscanf(p1, "%d", &temp1);//--���ٶ�
                sscanf(p2, "%f", &temp2);//--�Ƕ�
                sscanf(p3, "%d", &temp3);//--Ȧ��
					  car_handle_p->car_turn_8_updata(temp1,temp2,temp3);
				    CAR_STATION = CMD_TURN_8;
					break;}
			case 'r': //--����ͬ��
                DISPLAY_STATION= DISPLAY_CA_BEGIN;//--ֹͣ����5������
                break;			
        case 'c': //--����ͬ��
                 DISPLAY_STATION = DISPLAY_HA_BEGIN;//--��һ��
				         car_handle_p->car_walk_straight_line_updata(300,1000);
				         CAR_STATION = SET_CMD_LINE;//--��������ֱ������
                break;	
        case 't': //--ʵʱ��λ
                DISPLAY_STATION = DISPLAY_RL_BEGIN;//--ң��
                break;	
    }

    return 0;
}
