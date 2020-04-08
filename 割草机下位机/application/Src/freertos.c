/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
//#include "iwdg.h"
#include "shell.h"
#include "car.h"
#include "stdio.h"
#include "usercmd.h"
#include "bno005.h"
#include "time.h"
#include "gps.h"
#include "uart.h"
#include "crc.h"
#include "shell.h"
#include "stdlib.h"
#include "uart3_rcv_queue.h"
#include "protocol.h"
#include "uppercmd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId Moving_Control_TaskHandle;
osThreadId Power_TaskHandle;
osThreadId Shell_TaskHandle;
osThreadId Data_Updata_TaskHandle;
osThreadId Analyse_Gps_TaskHandle;
osThreadId UART3_RCV_TaskHandle;
osTimerId Timer_10msHandle;
osTimerId Timer_100msHandle;
osSemaphoreId Moving_Control_SemHandle;
osSemaphoreId Usart_Shell_SemHandle;
osSemaphoreId Data_Updata_SemHandle;
osSemaphoreId Analyse_Gps_SemHandle;
osSemaphoreId Uart3_Rcv_SemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Start_Moving_Control_Task(void const * argument);
void Start_Power_Task(void const * argument);
void Start_Shell_Task(void const * argument);
void Start_Data_Updata_Task(void const * argument);
void Start_Analyse_Gps_Task(void const * argument);
void Start_UART3_RCV_Task(void const * argument);
void Timer_10ms_Callback(void const * argument);
void Timer_100ms_Callback(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{
}

__weak unsigned long getRunTimeCounterValue(void)
{
    return 0;
}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of Moving_Control_Sem */
  osSemaphoreDef(Moving_Control_Sem);
  Moving_Control_SemHandle = osSemaphoreCreate(osSemaphore(Moving_Control_Sem), 1);

  /* definition and creation of Usart_Shell_Sem */
  osSemaphoreDef(Usart_Shell_Sem);
  Usart_Shell_SemHandle = osSemaphoreCreate(osSemaphore(Usart_Shell_Sem), 1);

  /* definition and creation of Data_Updata_Sem */
  osSemaphoreDef(Data_Updata_Sem);
  Data_Updata_SemHandle = osSemaphoreCreate(osSemaphore(Data_Updata_Sem), 1);

  /* definition and creation of Analyse_Gps_Sem */
  osSemaphoreDef(Analyse_Gps_Sem);
  Analyse_Gps_SemHandle = osSemaphoreCreate(osSemaphore(Analyse_Gps_Sem), 1);

  /* definition and creation of Uart3_Rcv_Sem */
  osSemaphoreDef(Uart3_Rcv_Sem);
  Uart3_Rcv_SemHandle = osSemaphoreCreate(osSemaphore(Uart3_Rcv_Sem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of Timer_10ms */
  osTimerDef(Timer_10ms, Timer_10ms_Callback);
  Timer_10msHandle = osTimerCreate(osTimer(Timer_10ms), osTimerPeriodic, NULL);

  /* definition and creation of Timer_100ms */
  osTimerDef(Timer_100ms, Timer_100ms_Callback);
  Timer_100msHandle = osTimerCreate(osTimer(Timer_100ms), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Moving_Control_Task */
  osThreadDef(Moving_Control_Task, Start_Moving_Control_Task, osPriorityHigh, 0, 512);
  Moving_Control_TaskHandle = osThreadCreate(osThread(Moving_Control_Task), NULL);

  /* definition and creation of Power_Task */
  osThreadDef(Power_Task, Start_Power_Task, osPriorityRealtime, 0, 64);
  Power_TaskHandle = osThreadCreate(osThread(Power_Task), NULL);

  /* definition and creation of Shell_Task */
  osThreadDef(Shell_Task, Start_Shell_Task, osPriorityAboveNormal, 0, 256);
  Shell_TaskHandle = osThreadCreate(osThread(Shell_Task), NULL);

  /* definition and creation of Data_Updata_Task */
  osThreadDef(Data_Updata_Task, Start_Data_Updata_Task, osPriorityRealtime, 0, 512);
  Data_Updata_TaskHandle = osThreadCreate(osThread(Data_Updata_Task), NULL);

  /* definition and creation of Analyse_Gps_Task */
  osThreadDef(Analyse_Gps_Task, Start_Analyse_Gps_Task, osPriorityHigh, 0, 256);
  Analyse_Gps_TaskHandle = osThreadCreate(osThread(Analyse_Gps_Task), NULL);

  /* definition and creation of UART3_RCV_Task */
  osThreadDef(UART3_RCV_Task, Start_UART3_RCV_Task, osPriorityHigh, 0, 64);
  UART3_RCV_TaskHandle = osThreadCreate(osThread(UART3_RCV_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Start_Moving_Control_Task */
/**
  * @brief  Function implementing the Moving_Control_ thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_Moving_Control_Task */
void Start_Moving_Control_Task(void const * argument)
{
    
    
    
    
    
    
    

  /* USER CODE BEGIN Start_Moving_Control_Task */
    car_handle_p->car_init();
     queue_handle_p->queue_init();
	  upper_cmd_header_p->set_update_cmd_flag(1);//--默认更新运动控制命令
		static uint8_t moving_cmd_temp;
	  static int16_t v_temp;
	   float w_temp,s_or_angle_temp;
		osTimerStart(Timer_100msHandle, 100);
		// MX_IWDG_Init();
    /* Infinite loop */

    for(;;) {
        osSemaphoreWait(Moving_Control_SemHandle, osWaitForever);
       /*速度更新*/
//==================================================================
      car_handle_p->car_speed_updata();	
			right_wheel_handle_p->count_wheel_walk_distance();
			left_wheel_handle_p->count_wheel_walk_distance();
			//------------------------------------------------------
			uint32_t time_temp = HAL_GetTick();		//--获取时间
			Car_Get_Time(time_temp);
		  car_data_center_p->speed = car_handle_p->read_speed();//--向数据中心更新速度
			car_data_center_p->distance = car_handle_p->read_car_distance();//--向数据中心更新总距离
//==================================================================
		/*获取上位机数据*/
		if(upper_cmd_header_p->get_remote_flag() != 1)//--非遥控
		{
			if(upper_cmd_header_p->get_update_cmd_flag())//--判断是否更新命令
			{
			
				upper_cmd_header_p->set_update_cmd_flag(0);//--吧更新命令标志设为0
			queue_handle_p->get_track_tracking_cmd(&moving_cmd_temp,&v_temp,&w_temp,&s_or_angle_temp);//--
      upper_cmd_header_p->set_cmd_type((E_upper_cmd_type)moving_cmd_temp);//--设置命令类型
			//--执行相应的命令
						 if(moving_cmd_temp == (uint8_t)E_UPPER_CMD_MOVING_LINE)car_handle_p->car_walk_straight_line_updata(v_temp,(int)s_or_angle_temp);						 
						 if(moving_cmd_temp == (uint8_t)E_UPPER_CMD_TURN)car_handle_p->car_turn_updata(v_temp,w_temp,s_or_angle_temp);

			}
			/*
				if(upper_cmd_header_p->get_update_cmd_flag())//--判断是否更新命令
				{//--如果更新命令标志是1，进行更新
					upper_cmd_header_p->set_update_cmd_flag(0);//--吧更新命令标志设为0，链表内已有的数据在动作函数内部更新
					if(queue_handle_p->list_is_null() ==0)//--如果链表不为空，说明有数据
						{
						 queue_handle_p->get_node_from_moving_list_head(&moving_cmd_temp,&v_temp,&w_temp,&s_or_angle_temp);//--从链表头获取数据
						 upper_cmd_header_p->set_cmd_type((E_upper_cmd_type)moving_cmd_temp);//--设置命令类型
							//--执行相应的命令
						 if(moving_cmd_temp == (uint8_t)E_UPPER_CMD_MOVING_LINE)car_handle_p->car_walk_straight_line_updata(v_temp,(int)s_or_angle_temp);		
						 if(moving_cmd_temp == (uint8_t)E_UPPER_CMD_TURN)car_handle_p->car_turn_updata(v_temp,w_temp,s_or_angle_temp);
					 }
						//--如果链表为空，设置为更新命令状态，等待新的命令到来
					else upper_cmd_header_p->set_update_cmd_flag(1);
			 }*/
   }
		else{//--遥控
		    queue_handle_p->get_remote_cmd(&moving_cmd_temp,&v_temp,&w_temp);
			  upper_cmd_header_p->set_cmd_type((E_upper_cmd_type)moving_cmd_temp);
			  if(moving_cmd_temp == (uint8_t)E_REMOTE_CMD_MOVING_LINE)car_handle_p->car_walk_straight_line_updata(v_temp,0);		
				if(moving_cmd_temp == (uint8_t)E_REMOTE_CMD_TURN)car_handle_p->car_turn_updata(v_temp,w_temp,0);
		}
//==================================================================
			    switch(upper_cmd_header_p->get_cmd_type())
					     {
						   case E_UPPER_CMD_STOP:
								     car_handle_p->car_stop();					 
								 break;
							 case E_UPPER_CMD_BRAKE:

								     car_handle_p->car_brake();	
								 break;
							 case E_UPPER_CMD_MOVING_LINE:
								    car_handle_p->car_walk_straight_line();//--走直线	
								 break;
							 case E_UPPER_CMD_TURN:
								    car_handle_p->car_turn();//--转弯						 
								 break;
							 case E_REMOTE_CMD_STOP:
								     car_handle_p->car_stop();					 
								 break;
							 case E_REMOTE_CMD_BRAKE:
								     car_handle_p->car_brake();	
								 break;
							 case E_REMOTE_CMD_MOVING_LINE:
								    car_handle_p->car_remote_line();//--走直线	
								 break;
							 case E_REMOTE_CMD_TURN:
								    car_handle_p->car_remote_turn();//--转弯						 
								 break;
							 default:
								 printf("5\r\n");
					          car_handle_p->car_stop();
							 }
//==================================================================
	 }

  /* USER CODE END Start_Moving_Control_Task */
}

/* USER CODE BEGIN Header_Start_Power_Task */
/**
* @brief Function implementing the Power_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Power_Task */
void Start_Power_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Power_Task */
st_usart_shell.reg_cmd("测试",Cmd_Turn_Test,"测试机器");
    /* Infinite loop */
    for(;;) {
			osDelay(10);  
		}

  /* USER CODE END Start_Power_Task */
}

/* USER CODE BEGIN Header_Start_Shell_Task */
/**
* @brief Function implementing the Shell_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Shell_Task */
void Start_Shell_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Shell_Task */
    st_usart_shell.init();

    /* Infinite loop */
    for(;;) {
        osSemaphoreWait(Usart_Shell_SemHandle, osWaitForever);
        st_usart_shell.rcv(1);
    }

  /* USER CODE END Start_Shell_Task */
}

/* USER CODE BEGIN Header_Start_Data_Updata_Task */
/**
* @brief Function implementing the Data_Updata_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Data_Updata_Task */
void Start_Data_Updata_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Data_Updata_Task */

//----------------------------------------------------------------------------------------------------------------------------
while((bon055_handle_p->bno005_err_display(bon055_handle_p->bno055_init(1))!=NO_ERROR))
 {
   HAL_I2C_DeInit(&hi2c1);     //--重启I2C
   HAL_I2C_Init(&hi2c1);         
   osDelay(10);
 }	
//----------------------------------------------------------------------------------------------------------------------------
  Data_Center_Iint(); //--数据中心初始化
//----------------------------------------------------------------------------------------------------------------------------
  uint32_t time_temp = 0;
//----------------------------------------------------------------------------------------------------------------------------
	osTimerStart(Timer_10msHandle, 20);

 
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreWait(Data_Updata_SemHandle, osWaitForever);
		//------------------------------------------------------	
		time_temp = HAL_GetTick();		//--获取时间
		//--------------------------------------------------------
		//--向数据中心更新imu数据
		uint8_t err = bon055_handle_p->bno055_get_data(&imu_data_center_p->pitch,&imu_data_center_p->roll,&imu_data_center_p->yaw);
    err += bon055_handle_p->bno055_get_acc_data(&imu_data_center_p->acc_x,&imu_data_center_p->acc_y,&imu_data_center_p->acc_z);
		err += bon055_handle_p->bno055_get_gyo_data(&imu_data_center_p->gyo_x,&imu_data_center_p->gyo_y,&imu_data_center_p->gyo_z);
		if(err)
		{
		  HAL_I2C_DeInit(&hi2c1);     //--重启I2C
      HAL_I2C_Init(&hi2c1);  
		}
    else
	  {
	    Imu_Get_Time(time_temp);//--IMU更新时间
	  }
	  //------------------------------------------------------------
		if(upper_cmd_header_p->get_reply_flag() == 1)//--发送回复OK
		  {
			 protocol_p->send(E_PROTOCOL_ANSWER_CMD,answer_cmd_p,sizeof(uint8_t));
			 upper_cmd_header_p->set_reply_flag(0);
			}
     else
		 {
			 //-----------------------从数据中心获取数据给串口打包-------------------------------------
					protocol_p->get_base_time(time_temp); 
					protocol_p->get_imu_time(imu_data_center_p->time);
					protocol_p->get_efk_pry_data(imu_data_center_p->pitch,imu_data_center_p->roll,imu_data_center_p->yaw);
					protocol_p->get_efk_acc_data(imu_data_center_p->acc_x,imu_data_center_p->acc_y,imu_data_center_p->acc_z);
					protocol_p->get_efk_gyo_data(imu_data_center_p->gyo_x,imu_data_center_p->gyo_y,imu_data_center_p->gyo_z);
					protocol_p->get_gps_time(gps_data_center_p->time);
					protocol_p->get_gps_station(gps_data_center_p->gps_state);
					protocol_p->get_gps_data(gps_data_center_p->latitude,gps_data_center_p->longitude,gps_data_center_p->speed,gps_data_center_p->rms);
					protocol_p->get_la_and_lo_station(gps_data_center_p->latitude_state,gps_data_center_p->longitude_state);
					protocol_p->get_gps_mode(gps_data_center_p->mode);
					protocol_p->get_odo_time(car_data_center_p->time);
					protocol_p->get_car_data(car_data_center_p->speed);
					//------------------------------------------------------------
			  switch(upper_cmd_header_p->get_package_type())
               {
								 case E_UPPER_MOVING_CMD:
								 break;
								 case E_UPPER_HEADING_SYCN:
									      protocol_p->send(E_PROTOCOL_HEADING_SYCN,ekf_location_p,sizeof(ekf_location));
								 break;
								 case E_UPPER_POSITION_SYCN:
									      protocol_p->send(E_PROTOCOL_POSITION_SYCN,ekf_location_p,sizeof(ekf_location));
								 break;
								 case E_UPPER_LOCATION_DATA:
									      protocol_p->send(E_PROTOCOL_LOCATION_DATA,ekf_location_p,sizeof(ekf_location));
								        printf("3\r\n");
								 break;       
								 case E_UPPER_MACHINE_STATE:
									       protocol_p->send(E_PROTOCOL_MACHINE_STATE,ekf_location_p,sizeof(ekf_location));
								 break;
								 case E_UPPER_ANSWER_CMD:
									   /*回应结束标志*/
								     /*设置包类型，停止发送*/
								      upper_cmd_header_p->set_package_type(E_UPPER_NULL_CMD);		
								 break;
								 case E_UPPER_NULL_CMD:
								 break;	 
	           }
			 

		      }
	   }
  
  /* USER CODE END Start_Data_Updata_Task */
}

/* USER CODE BEGIN Header_Start_Analyse_Gps_Task */
/**
* @brief Function implementing the Analyse_Gps_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Analyse_Gps_Task */
void Start_Analyse_Gps_Task(void const * argument)
{
  /* USER CODE BEGIN Start_Analyse_Gps_Task */
	 uart_handle_p->uart_init();//--GPS 串口空闲中断初始化  
    uint32_t time_temp = 0;	
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreWait(Analyse_Gps_SemHandle, osWaitForever);
	time_temp = HAL_GetTick();		//--获取时间
	if(DISPLAY_STATION ==DISPLAY_NO_ANALYZE )
		uart_handle_p->uart_send_test();		
	else uart_handle_p->uart_rcv_gps(time_temp);//--如果接收到数据就解析数据并更新时间

  }
  /* USER CODE END Start_Analyse_Gps_Task */
}

/* USER CODE BEGIN Header_Start_UART3_RCV_Task */
/**
* @brief Function implementing the UART3_RCV_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_UART3_RCV_Task */
void Start_UART3_RCV_Task(void const * argument)
{
  /* USER CODE BEGIN Start_UART3_RCV_Task */
	
	  uart_handle_p->uart3_init();//--GPS 串口空闲中断初始化  
	 

  /* Infinite loop */
  for(;;)
  {
    osSemaphoreWait(Uart3_Rcv_SemHandle, osWaitForever);
    uart_handle_p->uart3_rcv_date();//--接收上位机数据

  }
  /* USER CODE END Start_UART3_RCV_Task */
}

/* Timer_10ms_Callback function */
void Timer_10ms_Callback(void const * argument)
{
  /* USER CODE BEGIN Timer_10ms_Callback */
	osSemaphoreRelease(Data_Updata_SemHandle);
  /* USER CODE END Timer_10ms_Callback */
}

/* Timer_100ms_Callback function */
void Timer_100ms_Callback(void const * argument)
{
  /* USER CODE BEGIN Timer_100ms_Callback */
    osSemaphoreRelease(Moving_Control_SemHandle);
   //HAL_IWDG_Refresh(&hiwdg);
  /* USER CODE END Timer_100ms_Callback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
