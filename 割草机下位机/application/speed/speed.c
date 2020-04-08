/*
  ******************************************************************************
  * @file    speed.c
  * @author  xiaoyuer
  * @version V0.0.1
  * @date    2019/7/16
  * @brief   伽利略机器人电机T法测速
  ******************************************************************************        
*/

#include "speed.h"

/*转一圈脉冲数值计算*/
uint32_t  Counting_Difference(ST_speed_variable_p speed_variable_p);
/*速度更新，mm/s*/
uint8_t Refresh_Speed(ST_speed_variable_p speed_variable_p,
                      ST_speed_ctrl_p speed_ctrl_p);
/*读取速度*/
int32_t Read_Speed(ST_speed_variable_p speed_variable_p);
/*速度初始化*/
uint8_t Speed_Init(ST_speed_ctrl_p speed_ctrl_p);
/*速度反初始化*/
uint8_t Speed_Deinit(ST_speed_ctrl_p speed_ctrl_p);
/*获取脉冲计数值*/
uint32_t Get_Motor_Cnt(ST_speed_variable_p speed_variable_p);
/*清零脉冲计数值*/
uint8_t Motor_Cnt_Reset(ST_speed_variable_p speed_variable_p);
//--左电机
/*转一圈脉冲数值计算*/
uint32_t  Left_Motor_Counting_Difference(void);
/*速度更新，mm/s*/
uint8_t Left_Motor_Refresh_Speed(void);
/*读取速度*/
int32_t Left_Motor_Read_Speed(void);
/*速度初始化*/
uint8_t Left_Motor_Speed_Init(void);
/*速度反初始化*/
uint8_t Left_Motor_Speed_Deinit(void);
/*获取脉冲计数值*/
uint32_t Left_Motor_Get_Motor_Cnt(void);
/*清零脉冲计数值*/
uint8_t Left_Motor_Motor_Cnt_Reset(void);
//--右电机
/*转一圈脉冲数值计算*/
uint32_t  Right_Motor_Counting_Difference(void);
/*速度更新，mm/s*/
uint8_t Right_Motor_Refresh_Speed(void);
/*读取速度*/
int32_t Right_Motor_Read_Speed(void);
/*速度初始化*/
uint8_t Right_Motor_Speed_Init(void);
/*速度反初始化*/
uint8_t Right_Motor_Speed_Deinit(void);
/*获取脉冲计数值*/
uint32_t Right_Motor_Get_Motor_Cnt(void);
/*清零脉冲计数值*/
uint8_t Right_Motor_Motor_Cnt_Reset(void);
/*=============================================*/
ST_speed_variable right_speed_variable={
	1,
	0,
	0,
	0,
	0
};
ST_speed_variable_p right_speed_variable_p = & right_speed_variable;
ST_speed_variable left_speed_variable={
	1,
	0,
	0,
	0,
  0
};
ST_speed_variable_p left_speed_variable_p = &left_speed_variable;
/*=================================================*/
ST_speed_ctrl right_speed_ctrl = {
 EXTI15_10_IRQn, /*外部中断端口号*/
 &htim5  /*计数器*/
};
ST_speed_ctrl_p right_speed_ctrl_p = &right_speed_ctrl;

ST_speed_ctrl left_speed_ctrl = {
 EXTI2_IRQn,/*外部中断端口号*/
 &htim5  /*计数器*/
};
ST_speed_ctrl_p left_speed_ctrl_p = &left_speed_ctrl;
/*============================================*/
uint32_t r_motor[SPEED_SIZE] = {0};
uint32_t *r_motor_begin_p = r_motor;
uint32_t *r_motor_end_p = &r_motor[SPEED_SIZE-1];
uint32_t l_motor[SPEED_SIZE] = {0};
uint32_t *l_motor_begin_p = l_motor;
uint32_t *l_motor_end_p = &l_motor[SPEED_SIZE-1];

/*=============================================*/
//--速度接口函数
ST_speed_function_interface_handle   left_speed_handle = {
Left_Motor_Counting_Difference,
Left_Motor_Refresh_Speed,
Left_Motor_Read_Speed,
Left_Motor_Speed_Init,
Left_Motor_Speed_Deinit,
Left_Motor_Get_Motor_Cnt,
Left_Motor_Motor_Cnt_Reset,
};
ST_speed_function_interface_handle_p left_speed_handle_p = &left_speed_handle;

ST_speed_function_interface_handle   right_speed_handle = {
Right_Motor_Counting_Difference,
Right_Motor_Refresh_Speed,
Right_Motor_Read_Speed,
Right_Motor_Speed_Init,
Right_Motor_Speed_Deinit,
Right_Motor_Get_Motor_Cnt,
Right_Motor_Motor_Cnt_Reset,

};
ST_speed_function_interface_handle_p right_speed_handle_p = &right_speed_handle;
/***************************************************************************
** Function name:       HAL_GPIO_EXTI_Callback
** Descriptions:        外部霍尔脉冲检测中断处理函数
** input parameters:    GPIO_Pin：中断引脚
** Returned value:      无
***************************************************************************/
//--中断处理函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
	if(GPIO_Pin == R_MOTOR_EXTI15_Pin)
      { 
       /*读取TIM5计数值*/
				
       *(r_motor_begin_p++) = __HAL_TIM_GET_COUNTER(&htim5);
			 /*循环数组*/
	      if(r_motor_begin_p == r_motor_end_p){
				    r_motor_begin_p = r_motor;	
			  }
				if(right_speed_variable_p->HAL_CNT == 65535)
				++right_speed_variable_p->overflow_count;
				++right_speed_variable_p->HAL_CNT;/*积累脉冲数，为里程计服务*/
				/*数组最后一位记录脉冲数,只为测量速度使用*/
				++(*r_motor_end_p);/*积累脉冲数*/
      }	
    if (GPIO_Pin == L_MOTOR_EXTI2_Pin)
     {		
       /*读取TIM5计数值*/
       *(l_motor_begin_p++) = __HAL_TIM_GET_COUNTER(&htim5);
		    /*循环数组*/
		    if(l_motor_begin_p == l_motor_end_p){
				   l_motor_begin_p = l_motor;
				 }	
			 	if(left_speed_variable_p->HAL_CNT == 65535)
				++left_speed_variable_p->overflow_count;
				++left_speed_variable_p->HAL_CNT;/*积累脉冲数，为里程计服务*/
				/*数组最后一位记录脉冲数*/
				++(*l_motor_end_p);/*积累脉冲数*/
	   }
 }
/***************************************************************************
** Function name:       Counting_Difference
** Descriptions:        计数器计数差值
** input parameters:    speed_variable_p：电机参数结构体
** Returned value:      motor_cnt1：计数差值
***************************************************************************/
 uint32_t  Counting_Difference(ST_speed_variable_p speed_variable_p)
 { 
	 uint32_t *motor_p = NULL;
	 uint32_t motor_cnt = 0;
	 uint32_t motor_cnt1 = 0;
	 uint16_t read_motor[2];
	 int i = 0;
	 int cnt = 0;
	 //---------------------------------------------------------------------//
	 /*读取电机状态*/
	 if(speed_variable_p == left_speed_variable_p)
	 left_motor_handle_p->get_motor_station(read_motor);
	 if(speed_variable_p == right_speed_variable_p)
	 right_motor_handle_p->get_motor_station(read_motor);
	 //---------------------------------------------------------------------//
	  if(read_motor[1]>=1000)/*如果PWM》1000,启动均值滤波*/
	  {
		 cnt = 9;/*100ms测3次速度*/
			speed_variable_p->smoothing_en = 1;
		}
		else
		{
		cnt = 3;/*100ms测一次速度*/
			speed_variable_p->smoothing_en = 0;
		}
		//---------------------------------------------------------------------//
		if(speed_variable_p == left_speed_variable_p)
		{
			 /*循环存储时，指针多+1，这里-1，才是最近有效数据*/
			 if(l_motor_begin_p == l_motor)/*如果当前指针指向首地址*/
				  motor_p = &l_motor[SPEED_SIZE-2];/*有效数据在尾地址*/
			 else motor_p = l_motor_begin_p-1;/*缓存当前有效地址指针*/
	    for(i=0;i<cnt;i++)
			 {
			 if(motor_p == l_motor) /*如果指针是首地址，指向第8位*/
				 {
				  motor_p = &l_motor[SPEED_SIZE-2];
					 if(*l_motor <= l_motor[SPEED_SIZE-2])/*计数器溢出*/
				  {
						motor_cnt = *l_motor + (65536-l_motor[SPEED_SIZE-2]);
						goto left_motor_end;
					}
				  motor_cnt = *l_motor - *motor_p;/*累加相邻脉冲计数值*/
					goto left_motor_end;
				 }
				 motor_p--;
				 if(*(motor_p+1) <= *motor_p)/*计数器溢出*/
				  {
						motor_cnt = *(motor_p+1) + (65536-*motor_p);
					  goto left_motor_end;
					}	
				motor_cnt = *(motor_p+1) - *motor_p;/*累加相邻脉冲计数值*/
				left_motor_end:
				motor_cnt1+=motor_cnt;
				}
			}
		//-------------------------------------------------------------------------//
	if(speed_variable_p == right_speed_variable_p)
		{
			 /*循环存储时，指针多+1，这里-1，才是最近有效数据*/
			 if(r_motor_begin_p == r_motor)/*如果当前指针指向首地址*/
				  motor_p = &r_motor[SPEED_SIZE-2];/*有效数据在尾地址*/
			 else motor_p = r_motor_begin_p-1;/*缓存当前有效地址指针*/
	    for(i=0;i<cnt;i++)
			 {
			 if(motor_p == r_motor) /*如果指针是首地址，指向第8位*/
				 {
				  motor_p = &r_motor[SPEED_SIZE-2];
					 if(*r_motor <= r_motor[SPEED_SIZE-2])/*计数器溢出*/
				  {
						motor_cnt = *r_motor + (65536-r_motor[SPEED_SIZE-2]);
						goto right_motor_end;
					}
				  motor_cnt = *r_motor - *motor_p;/*累加相邻脉冲计数值*/
					goto right_motor_end;
				 }
				 motor_p--;
				 if(*(motor_p+1) <= *motor_p)/*计数器溢出*/
				  {
						motor_cnt = *(motor_p+1) + (65536-*motor_p);
					  goto right_motor_end;
					}	
				motor_cnt = *(motor_p+1) - *motor_p;/*累加相邻脉冲计数值*/
				right_motor_end:
				motor_cnt1+=motor_cnt;
				}
			}
	return motor_cnt1;
 }

/***************************************************************************
** Function name:       Refresh_Speed
** Descriptions:        更新速度
** input parameters:    speed_variable_p：电机速度参数结构体
**                      speed_ctrl_p：电机速度控制结构体
** Returned value:      0
***************************************************************************/
uint8_t Refresh_Speed(ST_speed_variable_p speed_variable_p,ST_speed_ctrl_p speed_ctrl_p)
{	
   
	uint32_t cnt = 0;
	int32_t speed = 0;
	uint16_t read_motor[2];
	static int8_t r_cnt =0,l_cnt = 0;

	//-----------------------------------------------------
	/*解决电机初期启动速度计算错误问题*/
	/*如果在初始化的时候打开外部中断，在调用该函数之前的这度时间会多次进入中断*/
	/*中断计数值会累加，导致初次计算速度数据错误*/
   if(speed_variable_p->first_electrification_flag == 1)/*如果是初次上电*/
	 {
	   speed_variable_p->first_electrification_flag = 0;
		 HAL_NVIC_EnableIRQ(speed_ctrl_p->external_interrupts);/*打开外部中断*/
	 return 0;
	 }
	 //----------------------------------------------------
	 if(speed_variable_p == left_speed_variable_p)
	 { 	
		 left_motor_handle_p->get_motor_station(read_motor);/*读取电机状态*/
		//--------------------------------------------------- 	 
		 /*如果100ms检测到的脉冲数小于4，本次不更新速度*/
		 /*解决电机低速抖动问题*/
		 if(*l_motor_end_p<4) 
		 { 
			 //---------------------------------------------
			 l_cnt ++; 
			 if(l_cnt >=2)/*连续超过两次都未测得有效数据，速度清零*/
			 {
			l_cnt = 0;
			speed_variable_p->motor_speed = 0;
			 }
			 //---------------------------------------------
			 memset(l_motor,0,SPEED_SIZE-1);//--最后一位不清零
			 l_motor_begin_p = &l_motor[0];
			
			 return 0;
		 }
		 else *l_motor_end_p = 0;
		 //-----------------------------------------------------
		 		l_cnt --;
		 if(l_cnt <=0)l_cnt = 0;
		//----------------------------------------------------- 
	  /*关闭外部中断*/
     HAL_NVIC_DisableIRQ(speed_ctrl_p->external_interrupts); 
		//-------------------------------------------------------------//
		 cnt = Counting_Difference(speed_variable_p);/*获取计数差值*/
		 if(speed_variable_p->smoothing_en == 1)
		 {
		 cnt = cnt /3;
		 }
		/*打开外部中断*/
		 HAL_NVIC_EnableIRQ(speed_ctrl_p->external_interrupts);
		 /*计算速度*/ 
	  //speed = 10000*60/cnt/REDUCTION_RATIO;//--r/min		
		speed = 10000*PI*WHEEL_D/cnt/REDUCTION_RATIO;//--mm/s
		 //--------------------------------------------
		if((read_motor[0]&0x0002)== 0x0002)//--FWD
	  speed_variable_p->motor_speed = -speed;
		else
		speed_variable_p->motor_speed = speed;	
		//--------------------------------------------
		 return 0;
	 }
	 if(speed_variable_p == right_speed_variable_p)
	 {  
		right_motor_handle_p->get_motor_station(read_motor);/*读取电机状态*/
		 /*如果10ms检测到的脉冲数小于2，本次不更新速度*/
		 /*解决电机低速抖动问题*/
		if(*r_motor_end_p<4) 
		 {	
			 //-------------------------------------------------------
			 r_cnt ++; 
			 if(r_cnt >=2)/*连续超过两次都未测得有效数据，速度清零*/
			 {
			r_cnt = 0;
			speed_variable_p->motor_speed = 0;
			 }
				 //---------------------------------------------
			 memset(r_motor,0,SPEED_SIZE-1);//--最后一位不清零
			 r_motor_begin_p = &r_motor[0];
			 return 0;
		 }
     else *r_motor_end_p = 0;
		 //-----------------------------------------------
		 r_cnt --;
		 if(r_cnt <=0)r_cnt = 0;
		//----------------------------------------------------- 
	 /*关闭外部中断*/
    HAL_NVIC_DisableIRQ(speed_ctrl_p->external_interrupts);
		//-------------------------------------------------------------/
		cnt = Counting_Difference(speed_variable_p);/*获取计数差值*/
		 if(speed_variable_p->smoothing_en == 1)
		 {
		 cnt = cnt /3;
		 }
	
		/*打开外部中断*/
	 HAL_NVIC_EnableIRQ(speed_ctrl_p->external_interrupts);
		 /*============计算速度=================*/	
	   //speed = 10000*60/cnt/REDUCTION_RATIO;//--r/min
		 speed = 10000*PI*WHEEL_D/cnt/REDUCTION_RATIO;//--mm/s
		 //----------------------------------------------------------
		if((read_motor[0]&0x0002)== 0x0002)//--FWD
	  speed_variable_p->motor_speed = speed;
		else
		speed_variable_p->motor_speed = -speed;	
		//------------------------------------------------------------
		return 0;
	 }

	*l_motor_end_p = 0;/*什么没做也要清除本次测速脉冲计数值*/
	 return 0;
}
/***************************************************************************
** Function name:       Read_Speed
** Descriptions:        速度读取
** input parameters:    speed_variable_p：电机速度参数结构体
** Returned value:      speed：速度
***************************************************************************/
int32_t Read_Speed(ST_speed_variable_p speed_variable_p)
{
 int32_t speed = 0;
  speed = speed_variable_p->motor_speed;
	return speed;
}
/***************************************************************************
** Function name:       Speed_Init
** Descriptions:        关闭外部中断，开启计数器
**input parameters:     speed_ctrl_p ：电机控制参数结构体
** Returned value:      无
***************************************************************************/
uint8_t Speed_Init(ST_speed_ctrl_p speed_ctrl_p)
{ 
		/*关闭外部中断*/
    HAL_NVIC_DisableIRQ(speed_ctrl_p->external_interrupts);
	  /*计数器清零*/
	  __HAL_TIM_SET_COUNTER(speed_ctrl_p->phtim_Speed_cnt,0);
	  /*开启计数器*/
    HAL_TIM_Base_Start(speed_ctrl_p->phtim_Speed_cnt); 
	 /*打开外部中断*/
	 //HAL_NVIC_EnableIRQ(speed_ctrl_p->external_interrupts);
	return 0;
}
/***************************************************************************
** Function name:       Speed_Deinit
** Descriptions:        关闭外部中断，计数器清零，关闭计数器
**input parameters:     speed_ctrl_p ：电机控制参数结构体
** Returned value:      无
***************************************************************************/
uint8_t Speed_Deinit(ST_speed_ctrl_p speed_ctrl_p)
{ 
		/*关闭外部中断*/
    HAL_NVIC_DisableIRQ(speed_ctrl_p->external_interrupts);
	  /*计数器清零*/
	  __HAL_TIM_SET_COUNTER(speed_ctrl_p->phtim_Speed_cnt,0);
	  /*关闭计数器*/
    HAL_TIM_Base_Stop(&htim5); 
	//===========打开外部中断===============//
	 //HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	 //HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	return 0;
}
/***************************************************************************
** Function name:       Get_Motor_Cnt
** Descriptions:        计算电机脉冲数
** input parameters:    speed_variable_p：电机速度参数结构体
** Returned value:      返回电机脉冲数
***************************************************************************/
uint32_t Get_Motor_Cnt(ST_speed_variable_p speed_variable_p)
{
 uint32_t cnt = 0;
 cnt = (speed_variable_p->overflow_count*65536+speed_variable_p->HAL_CNT);
	return cnt;
}

uint8_t Motor_Cnt_Reset(ST_speed_variable_p speed_variable_p)
{
     speed_variable_p->overflow_count = 0;
		 speed_variable_p->HAL_CNT = 0;
	   return 0;
}

//--左电机
/*转一圈脉冲数值计算*/
uint32_t  Left_Motor_Counting_Difference(void)
{
return Counting_Difference(left_speed_variable_p);
}
/*速度更新，mm/s*/
uint8_t Left_Motor_Refresh_Speed(void)
{
return Refresh_Speed(left_speed_variable_p,left_speed_ctrl_p);
}
/*读取速度*/
int32_t Left_Motor_Read_Speed(void)
{
return Read_Speed(left_speed_variable_p);
}
/*速度初始化*/
uint8_t Left_Motor_Speed_Init(void)
{
return Speed_Init(left_speed_ctrl_p);
}
/*速度反初始化*/
uint8_t Left_Motor_Speed_Deinit(void)
{
return Speed_Deinit(left_speed_ctrl_p);
}
/*获取脉冲计数值*/
uint32_t Left_Motor_Get_Motor_Cnt(void)
{
return Get_Motor_Cnt(left_speed_variable_p);
}
/*清零脉冲计数值*/
uint8_t Left_Motor_Motor_Cnt_Reset(void)
{
return Motor_Cnt_Reset(left_speed_variable_p);
}

//--右电机
/*转一圈脉冲数值计算*/
uint32_t  Right_Motor_Counting_Difference(void)
{
return Counting_Difference(right_speed_variable_p);
}
/*速度更新，mm/s*/
uint8_t Right_Motor_Refresh_Speed(void)
{
return Refresh_Speed(right_speed_variable_p,right_speed_ctrl_p);
}
/*读取速度*/
int32_t Right_Motor_Read_Speed(void)
{
return Read_Speed(right_speed_variable_p);
}
/*速度初始化*/
uint8_t Right_Motor_Speed_Init(void)
{
return Speed_Init(right_speed_ctrl_p);
}
/*速度反初始化*/
uint8_t Right_Motor_Speed_Deinit(void)
{
return Speed_Deinit(right_speed_ctrl_p);
}
/*获取脉冲计数值*/
uint32_t Right_Motor_Get_Motor_Cnt(void)
{
return Get_Motor_Cnt(right_speed_variable_p);
}
/*清零脉冲计数值*/
uint8_t Right_Motor_Motor_Cnt_Reset(void)
{
return Motor_Cnt_Reset(right_speed_variable_p);
}

