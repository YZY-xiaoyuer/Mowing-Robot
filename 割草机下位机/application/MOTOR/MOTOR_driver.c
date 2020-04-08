/**
  ******************************************************************************
  * @file    MOTOR_drive.c
  * @author  小鱼儿飞丫飞
  * @version V0.0.1
  * @date    2019/7/17
  * @brief   伽利略机器人电机驱动
  ******************************************************************************        
*/

#include "MOTOR_driver.h"
#include "stdio.h"

//==============================函数=====================================
/*电机初始化*/
uint8_t Motor_Init(ST_motor_ctrl_p wheel_ctrl_handle_p,
                   ST_motor_ctrl_parameter_p wheel_para_handle_p);
/*电机反初始化*/
uint8_t Motor_Deinit(ST_motor_ctrl_p wheel_ctrl_handle_p,
                     ST_motor_ctrl_parameter_p wheel_para_handle_p);
/*电机启动*/
uint8_t Motor_Start(ST_motor_ctrl_p wheel_ctrl_handle_p,
                    ST_motor_ctrl_parameter_p wheel_para_handle_p, 
                    int32_t pwm);
/*正常停止*/
uint8_t Motor_Stop(ST_motor_ctrl_p wheel_ctrl_handle_p,
                   ST_motor_ctrl_parameter_p wheel_para_handle_p);
/*紧急制动*/
uint8_t Motor_Brake(ST_motor_ctrl_p wheel_ctrl_handle_p,
                    ST_motor_ctrl_parameter_p wheel_para_handle_p);
/*电机状态读取*/
uint8_t  Get_Motor_Station(ST_motor_ctrl_parameter_p wheel_para_handle_p,
                           uint16_t motor_station[2]);

/*电机输出使能*/
uint8_t Motor_Driver_Output_Enable (ST_motor_ctrl_p wheel_ctrl_handle_p,
                                    ST_motor_ctrl_parameter_p wheel_para_handle_p);
/*电机PWM占空比设置*/
uint8_t Motor_Speed_Pwm_Duty_Cycle_Set(ST_motor_ctrl_p wheel_ctrl_handle_p,
                                       ST_motor_ctrl_parameter_p wheel_para_handle_p);
/*电机转向设置*/
uint8_t Motor_Fwd_Rev_Select (ST_motor_ctrl_p wheel_ctrl_handle_p,
	                        ST_motor_ctrl_parameter_p wheel_para_handle_p);
/*电机刹车使能*/
uint8_t Motor_Brake_Enable (ST_motor_ctrl_p wheel_ctrl_handle_p,
	                      ST_motor_ctrl_parameter_p wheel_para_handle_p);
/*电机工作模式设置*/
uint8_t Motor_Driver_Work_Mode (ST_motor_ctrl_p wheel_ctrl_handle_p,
	                          ST_motor_ctrl_parameter_p wheel_para_handle_p);

 //--左电机
/*左电机初始化*/
uint8_t Left_Motor_Init(void);
/*左电机反初始化*/
uint8_t Left_Motor_Deinit(void);
/*左电机启动*/
uint8_t Left_Motor_Start(int32_t pwm);
/*左正常停止*/
uint8_t Left_Motor_Stop(void);
/*左紧急制动*/
uint8_t Left_Motor_Brake(void);
/*获取左电机状态*/
uint8_t  Get_Left_Motor_Station(uint16_t motor_station[2]);
//--右电机
/*右电机初始化*/
uint8_t Right_Motor_Init(void);
/*右电机反初始化*/
uint8_t Right_Motor_Deinit(void);
/*右电机启动*/
uint8_t Right_Motor_Start(int32_t pwm);
/*右正常停止*/
uint8_t Right_Motor_Stop(void);
/*右紧急制动*/
uint8_t Right_Motor_Brake(void);
/*获取右电机状态*/
uint8_t  Get_Right_Motor_Station(uint16_t motor_station[2]);
//--右侧车轮电机控制结构体 
/**
*********************************************************************************

#define R_motor_brake_Pin GPIO_PIN_1
#define R_motor_brake_GPIO_Port GPIOG
#define R_motor_fwd_rev_Pin GPIO_PIN_7
#define R_motor_fwd_rev_GPIO_Port GPIOE
#define R_motor_output_enable_Pin GPIO_PIN_8
#define R_motor_output_enable_GPIO_Port GPIOE
#define R_motor_speed_pwm_Pin GPIO_PIN_14
#define R_motor_speed_pwm_GPIO_Port GPIOE
*********************************************************************************
*/
ST_motor_ctrl right_wheel_ctrl_handle = {

    R_motor_output_enable_GPIO_Port,
    R_motor_output_enable_Pin,
    R_motor_fwd_rev_GPIO_Port,
    R_motor_fwd_rev_Pin,
    R_motor_brake_GPIO_Port,
    R_motor_brake_Pin,
    &htim1,  //--PWM输出
    TIM_CHANNEL_4,
};
ST_motor_ctrl_p right_wheel_ctrl_handle_p = &right_wheel_ctrl_handle;
//-- 右侧车轮电机控制结构体 
/**
*********************************************************************************
#define L_motor_brake_Pin GPIO_PIN_14
#define L_motor_brake_GPIO_Port GPIOF
#define L_motor_fwd_rev_Pin GPIO_PIN_15
#define L_motor_fwd_rev_GPIO_Port GPIOF
#define L_motor_output_enable_Pin GPIO_PIN_0
#define L_motor_output_enable_GPIO_Port GPIOG
#define L_motor_speed_pwm_Pin GPIO_PIN_13
#define L_motor_speed_pwm_GPIO_Port GPIOE
*********************************************************************************
*/
ST_motor_ctrl left_wheel_ctrl_handle = {

    L_motor_output_enable_GPIO_Port,
    L_motor_output_enable_Pin,
    L_motor_fwd_rev_GPIO_Port,
    L_motor_fwd_rev_Pin,
    L_motor_brake_GPIO_Port,
    L_motor_brake_Pin,
    &htim1,  /*PWM输*/
    TIM_CHANNEL_3,
};
ST_motor_ctrl_p left_wheel_ctrl_handle_p = &left_wheel_ctrl_handle;
/*==============================================================================*/
/*右侧车轮电机控制参数结构体*/ 
ST_motor_ctrl_parameter   right_wheel_para_handle={
    output_disable,             /*电机驱动器输出使能标志*/
    motor_fwd,                  /*电机旋转方向标志*/
  	brake_disable,              /*刹车使能标志*/ 
	  work_mode_normal,         /*电机驱动芯片选择*/ 
    right_motor,	                /*电机选择*/
    0                           /*PWM*/
};
ST_motor_ctrl_parameter_p right_wheel_para_handle_p =&right_wheel_para_handle;

/*左侧车轮电机控制参数结构体 */ 
ST_motor_ctrl_parameter left_wheel_para_handle={
    output_disable,             //--电机驱动器输出使能标志
    motor_rev,                  //--电机旋转方向标志
    brake_disable,              //--刹车使能标志 
	  work_mode_normal,         //--电机驱动芯片选择 
    left_motor,	
    0                           //--转速PWM脉宽 
};
ST_motor_ctrl_parameter_p left_wheel_para_handle_p =&left_wheel_para_handle;

/*==============================================================================*/
/*左电机函数接口*/
 ST_motor_handle   left_motor_handle = {
  Left_Motor_Init,
  Left_Motor_Deinit,
  Left_Motor_Start,
  Left_Motor_Stop,
  Left_Motor_Brake,
	Get_Left_Motor_Station
 };
 ST_motor_handle_p left_motor_handle_p = &left_motor_handle;
 /*右电机函数接口*/
 ST_motor_handle   right_motor_handle = {
  Right_Motor_Init,
  Right_Motor_Deinit,
  Right_Motor_Start,
  Right_Motor_Stop,
  Right_Motor_Brake,
	Get_Right_Motor_Station
 };
 ST_motor_handle_p right_motor_handle_p = &right_motor_handle;

/***************************************************************************
** Function name:       Motor_Init
** Descriptions:        电机初始化 
**                      初始化电机方向：默认小车前进方向，注意左右两轮方向相反
**                      初始化电机模式 ：设置电机为正常工作模式
** input parameters:    wheel_ctrl_handle_p：电机控制结构体
**                      wheel_para_handle_p：电机参数结构体
** Returned value:      0
***************************************************************************/
uint8_t Motor_Init(ST_motor_ctrl_p wheel_ctrl_handle_p,
                   ST_motor_ctrl_parameter_p wheel_para_handle_p)
{
  	/*右电机和左电机方向相反*/
	  if(wheel_para_handle_p->motor_switc == right_motor)
	  wheel_para_handle_p->fwd_rev_flag = motor_fwd;
	  if(wheel_para_handle_p->motor_switc == left_motor)
	  wheel_para_handle_p->fwd_rev_flag = motor_rev;
		Motor_Fwd_Rev_Select (wheel_ctrl_handle_p, wheel_para_handle_p);
	 /*电机模式选择*/
		wheel_para_handle_p->motor_driver_work_mode = work_mode_normal;
	 Motor_Driver_Work_Mode (wheel_ctrl_handle_p, wheel_para_handle_p);
	return 0;
}
/***************************************************************************
** Function name:       Motor_Deinit
** Descriptions:        电机反初始化 
**                      设置电机为无输出模式
** input parameters:    wheel_ctrl_handle_p：电机控制结构体
**                      wheel_para_handle_p：电机参数结构体
** Returned value:      0
***************************************************************************/
uint8_t Motor_Deinit(ST_motor_ctrl_p wheel_ctrl_handle_p,
                     ST_motor_ctrl_parameter_p wheel_para_handle_p)
{
  	/*电机模式选择*/
	wheel_para_handle_p->motor_driver_work_mode = work_mode_output_disable;
	Motor_Driver_Work_Mode (wheel_ctrl_handle_p, wheel_para_handle_p);
	return 0;
}
/***************************************************************************
** Function name:       Motor_Start
** Descriptions:        启动电机
**                      根据PWM波正负选择电机旋转方向，注意左右两轮方向是相反的
**                      注意，PWM计算的时候有正有负，进行PWM输出设置的时候只有正数
**                      设置电机模式为正常工作模式
** input parameters:    wheel_ctrl_handle_p：电机控制结构体
**                      wheel_para_handle_p：电机参数结构体
**                      PWM：电机转速PWM波
** Returned value:      0
***************************************************************************/
uint8_t Motor_Start(ST_motor_ctrl_p wheel_ctrl_handle_p,
                    ST_motor_ctrl_parameter_p wheel_para_handle_p, 
                    int32_t pwm)
{
  if(pwm >=0)//--前进
	{
	if(wheel_para_handle_p->motor_switc == right_motor)
	  wheel_para_handle_p->fwd_rev_flag = motor_fwd;
	  if(wheel_para_handle_p->motor_switc == left_motor)
	  wheel_para_handle_p->fwd_rev_flag = motor_rev;
	wheel_para_handle_p->pwm_value = pwm;
	}

  if(pwm <0)//--后退
	{
		if(wheel_para_handle_p->motor_switc == right_motor)
	  wheel_para_handle_p->fwd_rev_flag = motor_rev;
	  if(wheel_para_handle_p->motor_switc == left_motor)
	  wheel_para_handle_p->fwd_rev_flag = motor_fwd;
		wheel_para_handle_p->pwm_value = -pwm;
	}		
	
	 Motor_Fwd_Rev_Select (wheel_ctrl_handle_p,wheel_para_handle_p);
	  /*占空比设置*/
	 Motor_Speed_Pwm_Duty_Cycle_Set(wheel_ctrl_handle_p,wheel_para_handle_p);
	 /*电机模式选择*/
		wheel_para_handle_p->motor_driver_work_mode = work_mode_normal;
	 Motor_Driver_Work_Mode (wheel_ctrl_handle_p, wheel_para_handle_p);
 
	return 0;
}
/***************************************************************************
** Function name:       Motor_Stop
** Descriptions:        电机正常停止，注意电机在此模式下会因为惯性进行滑行
**                      修改电机工作模式
**                      注意 一定要停止PWM输出，而不是把PWM设置为0，否则会导致电机
**                      从新启动的时候强烈抖动。
** input parameters:    wheel_ctrl_handle_p：电机控制结构体
**                      wheel_para_handle_p：电机参数结构体
**                      PWM：电机转速PWM波
** Returned value:      0
***************************************************************************/
uint8_t Motor_Stop(ST_motor_ctrl_p wheel_ctrl_handle_p,
                   ST_motor_ctrl_parameter_p wheel_para_handle_p)
{
   /*电机模式选择*/
	 wheel_para_handle_p->motor_driver_work_mode = work_mode_output_disable;
	 Motor_Driver_Work_Mode (wheel_ctrl_handle_p, wheel_para_handle_p); 
	/*关闭PWM*/
	  HAL_TIM_PWM_Stop( wheel_ctrl_handle_p->phtim_Speed_Adjust_PWM,  
	                   wheel_ctrl_handle_p->Channel_Speed_Adjust_PWM); 
	return 0;
}
/***************************************************************************
** Function name:       Motor_Brake
** Descriptions:        电机紧急制动，注意电机在此模式下会抱死
**                      修改电机工作模式
**                      注意 一定要停止PWM输出，而不是把PWM设置为0，否则会导致电机
**                      从新启动的时候强烈抖动。
** input parameters:    wheel_ctrl_handle_p：电机控制结构体
**                      wheel_para_handle_p：电机参数结构体
**                      PWM：电机转速PWM波
** Returned value:      0
***************************************************************************/
/*紧急制动*/
uint8_t Motor_Brake(ST_motor_ctrl_p wheel_ctrl_handle_p,
                   ST_motor_ctrl_parameter_p wheel_para_handle_p)
{
   /*电机模式选择*/
	 wheel_para_handle_p->motor_driver_work_mode = work_mode_brake_hard;
	 Motor_Driver_Work_Mode (wheel_ctrl_handle_p, wheel_para_handle_p); 
	/*关闭PWM*/
	  HAL_TIM_PWM_Stop( wheel_ctrl_handle_p->phtim_Speed_Adjust_PWM,  
	                   wheel_ctrl_handle_p->Channel_Speed_Adjust_PWM); 
	return 0;
}

/***************************************************************************
** Function name:       Motor_Driver_Output_Enable
** Descriptions:        电机驱动器输出使能控制  
** input parameters:    wheel_ctrl_handle_p：电机控制结构体
**                      wheel_para_handle_p：电机参数结构体
** Returned value:      0
***************************************************************************/
uint8_t Motor_Driver_Output_Enable (ST_motor_ctrl_p wheel_ctrl_handle_p,
                                    ST_motor_ctrl_parameter_p wheel_para_handle_p)
{

 HAL_GPIO_WritePin(wheel_ctrl_handle_p->pGPIOx_OutputEnable, 
	                 wheel_ctrl_handle_p->GPIO_Pin_OutputEnable, 
	                (GPIO_PinState)wheel_para_handle_p->output_enable_flag);
 return 0;
}
/***************************************************************************
** Function name:       Motor_speed_pwm_duty_cycle_set    
** Descriptions:        转速占空比调节
** input parameters:    wheel_ctrl_handle_p：电机控制结构体
**                      wheel_para_handle_p：电机参数结构体
**                       无分频 72MHZ  每个PWM周期计数2500 
**                       脉冲宽度 0-2500
** Returned value:      0
***************************************************************************/
uint8_t Motor_Speed_Pwm_Duty_Cycle_Set(ST_motor_ctrl_p wheel_ctrl_handle_p,
                                       ST_motor_ctrl_parameter_p wheel_para_handle_p)
{

  TIM_OC_InitTypeDef sConfigOC;

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = wheel_para_handle_p->pwm_value;/*占空比设置*/
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
 if (HAL_TIM_PWM_ConfigChannel( wheel_ctrl_handle_p->phtim_Speed_Adjust_PWM, 
	                              &sConfigOC,  
                                wheel_ctrl_handle_p->Channel_Speed_Adjust_PWM) 
                               != HAL_OK)
  {
    Error_Handler();
  }
  
  HAL_TIM_PWM_Start( wheel_ctrl_handle_p->phtim_Speed_Adjust_PWM,  
	                   wheel_ctrl_handle_p->Channel_Speed_Adjust_PWM); 
 return 0;
}
/***************************************************************************
** Function name:       Motor_Fwd_Rev_Select    
** Descriptions:        电机正反转选择
** input parameters:    wheel_ctrl_handle_p：电机控制结构体
**                      wheel_para_handle_p：电机参数结构体 
** Returned value:      0:正常
                        MOTOR_FWD_REV_SELECT_DEFAULT：出错
***************************************************************************/
uint8_t Motor_Fwd_Rev_Select (ST_motor_ctrl_p wheel_ctrl_handle_p,
                              ST_motor_ctrl_parameter_p wheel_para_handle_p)
{

 HAL_GPIO_WritePin(wheel_ctrl_handle_p->pGPIOx_FwdRevSelect, 
	                 wheel_ctrl_handle_p->GPIO_Pin_FwdRevSelect, 
	                (GPIO_PinState)wheel_para_handle_p->fwd_rev_flag);
		return 0;
}
/***************************************************************************
** Function name:       Motor_Brake_Enable
** Descriptions:        电机驱动器制动输入
** input parameters:    wheel_ctrl_handle_p：电机控制结构体
**                      wheel_para_handle_p：电机参数结构体�
** Returned value:      MOTOR_SWITCH_DEFAULT
***************************************************************************/
uint8_t Motor_Brake_Enable (ST_motor_ctrl_p wheel_ctrl_handle_p,
	                      ST_motor_ctrl_parameter_p wheel_para_handle_p)
{
  HAL_GPIO_WritePin(wheel_ctrl_handle_p->pGPIOx_Brake, 
	                  wheel_ctrl_handle_p->GPIO_Pin_Brake , 
	                 (GPIO_PinState)wheel_para_handle_p->brake_flag);
 return 0;
}
/***************************************************************************
** Function name:       Motor_Speed_Cnt_Get   
** Descriptions:        获取电机转速解码器计数值   
** input parameters:    wheel_ctrl_handle_p：电机控制结构体
**                      wheel_para_handle_p：电机参数结构体
** Returned value:      当前电机转速解码器计数值
***************************************************************************/
//float Motor_Speed_Cnt_Get (ST_motor_ctrl_p wheel_ctrl_handle_p,)
//{  

//   return __HAL_TIM_GET_COUNTER(wheel_ctrl_handle_p->phtim_DecoderCounter);
//}
/***************************************************************************
** Function name:       Motor_Speed_Cnt_Set  
** Descriptions:        设置电机转速解码器计数值   
** input parameters:    wheel_ctrl_handle_p：电机控制结构体
**                      wheel_para_handle_p：电机参数结构体
**                      cnt             ：要设置的计数器的值 0-2500
** Returned value:      MOTOR_SWITCH_DEFAULT
***************************************************************************/

//__INLINE int Motor_Speed_Cnt_Set (ST_motor_ctrl_p wheel_ctrl_handle_p, uint16_t cnt)
//{

//    __HAL_TIM_SET_COUNTER(wheel_ctrl_handle_p->phtim_DecoderCounter,cnt);
//		 return 0;
//}

/***************************************************************************
** Function name:       Motor_Driver_Work_Mode  
** Descriptions:        设置电机驱动芯片的工作模式   
** input parameters:    wheel_ctrl_handle_p：电机控制结构体
**                      wheel_para_handle_p：电机参数结构体
** Returned value:      0
***************************************************************************/
uint8_t Motor_Driver_Work_Mode (ST_motor_ctrl_p wheel_ctrl_handle_p,
                                ST_motor_ctrl_parameter_p wheel_para_handle_p)
{   
		//--模式选择 
    if(wheel_para_handle_p->motor_driver_work_mode == work_mode_brake_hard) {
        /* 对于需要紧急刹车的场合，应以最快速度控制电机执行刹车动作 */
        wheel_para_handle_p->output_enable_flag = output_enable;
        wheel_para_handle_p->brake_flag = brake_enable;  
			  wheel_para_handle_p->pwm_value = 0;
    } else if(wheel_para_handle_p->motor_driver_work_mode == work_mode_normal) {
        wheel_para_handle_p->output_enable_flag = output_enable;
        wheel_para_handle_p->brake_flag = brake_disable;
    } else if(wheel_para_handle_p->motor_driver_work_mode == work_mode_output_disable) {
        wheel_para_handle_p->output_enable_flag = output_disable;
			  wheel_para_handle_p->brake_flag = brake_disable;
        wheel_para_handle_p->pwm_value = 0;
    } 
    Motor_Brake_Enable (wheel_ctrl_handle_p, wheel_para_handle_p);
		Motor_Driver_Output_Enable (wheel_ctrl_handle_p, wheel_para_handle_p);	
   return 0;		
}

/***************************************************************************
** Function name:       Get_Motor_Station  
** Descriptions:        获取电机状态 
** input parameters:    motor_switc：电机选择，可以取一下值：
**                                  LEFT_MOTOR ： 左侧电机
**                                  RIGHT_MOTOR： 右侧电机
**                                  MOWING_MOTOR：割草电机
**                      motor_station[0]
**                      xxxx xxxx
**                      bit 0  : 0-输出禁能 1-输出使能
**                      bit 1  : 0-前进     1-后退
**                      bit 2  : 0-刹车禁能 1-刹车使能
**                      bit 3-4: 00-正常模式 01-无输出模式 10-紧急刹车模式
**                      bit 5-15 : 保留
**                      motor_station[1]
**                      电机转速占空比
** Returned value:      MOTOR_SWITCH_DEFAULT
***************************************************************************/
uint8_t  Get_Motor_Station(ST_motor_ctrl_parameter_p wheel_para_handle_p,
                           uint16_t motor_station[2])
{   
		
		if(wheel_para_handle_p->output_enable_flag == output_enable)       {motor_station[0] = motor_station[0]|0x0001;}
		else if(wheel_para_handle_p->output_enable_flag == output_disable) {motor_station[0] = motor_station[0]&(~0x0001);}
		
		if(wheel_para_handle_p->fwd_rev_flag == motor_fwd)       {motor_station[0] = motor_station[0]|0x0002;}
		else if(wheel_para_handle_p->fwd_rev_flag == motor_rev)  {motor_station[0] = motor_station[0]&(~0x0002);}
		
		if(wheel_para_handle_p->brake_flag == brake_enable)       {motor_station[0] = motor_station[0]|0x0004;}
		else if(wheel_para_handle_p->brake_flag == brake_disable) {motor_station[0] = motor_station[0]&(~0x0004);}
		
	  if(wheel_para_handle_p->motor_driver_work_mode == work_mode_normal)              {motor_station[0] = motor_station[0]&(~0x0018);}
		else if(wheel_para_handle_p->motor_driver_work_mode == work_mode_output_disable) {motor_station[0] = (motor_station[0]&(~0x0010))|0x0008;}
		else if(wheel_para_handle_p->motor_driver_work_mode == work_mode_brake_hard)     {motor_station[0] = (motor_station[0]|0x0010)&(~0x0008);}
	 
		motor_station[1] = wheel_para_handle_p->pwm_value;
		return 0;
}
/***************************************************************************
** Function name:       Left_Motor_Init
** Descriptions:        左电机初始化：调用电机初始化方法函数
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Left_Motor_Init(void)
{
	return Motor_Init(left_wheel_ctrl_handle_p,left_wheel_para_handle_p);
}
/***************************************************************************
** Function name:       Left_Motor_Deinit
** Descriptions:        左电机反初始化：调用电机反初始化方法函数
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Left_Motor_Deinit(void)
{
	return Motor_Deinit(left_wheel_ctrl_handle_p,left_wheel_para_handle_p);
}
/***************************************************************************
** Function name:       Left_Motor_Start
** Descriptions:        左电机启动：调用电机启动方法函数
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Left_Motor_Start(int32_t pwm)
{
	return Motor_Start(left_wheel_ctrl_handle_p,left_wheel_para_handle_p,pwm);
}
/***************************************************************************
** Function name:       Left_Motor_Stop
** Descriptions:        左电机正常停止：调用电机正常停止方法函数
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Left_Motor_Stop(void)
{
	return Motor_Stop(left_wheel_ctrl_handle_p,left_wheel_para_handle_p);
}
/***************************************************************************
** Function name:       Left_Motor_Brake
** Descriptions:        左电机紧急制动：调用电机紧急制动方法函数
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Left_Motor_Brake(void)
{ 
	return Motor_Brake(left_wheel_ctrl_handle_p,left_wheel_para_handle_p);
}
/***************************************************************************
** Function name:       Right_Motor_Init
** Descriptions:        右电机初始化：调用电机初始化方法函数
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Right_Motor_Init(void)
{
	return Motor_Init(right_wheel_ctrl_handle_p,right_wheel_para_handle_p);
}
/***************************************************************************
** Function name:       Right_Motor_Deinit
** Descriptions:        右电机反初始化：调用电机反初始化方法函数
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Right_Motor_Deinit(void)
{
 return Motor_Deinit(right_wheel_ctrl_handle_p,right_wheel_para_handle_p);
}
/***************************************************************************
** Function name:       Right_Motor_Start
** Descriptions:        右电机启动：调用电机启动方法函数
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Right_Motor_Start(int32_t pwm)
{
 return Motor_Start(right_wheel_ctrl_handle_p,right_wheel_para_handle_p,pwm);
}
/***************************************************************************
** Function name:       Right_Motor_Stop
** Descriptions:        右电机正常停止：调用电机正常停止方法函数
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Right_Motor_Stop(void)
{
 return Motor_Stop(right_wheel_ctrl_handle_p,right_wheel_para_handle_p);
}
/***************************************************************************
** Function name:       Right_Motor_Brake
** Descriptions:        右电机紧急制动：调用电机紧急制动方法函数
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Right_Motor_Brake(void)
{
 return Motor_Brake(right_wheel_ctrl_handle_p,right_wheel_para_handle_p);
}
/***************************************************************************
** Function name:       Get_Right_Motor_Station
** Descriptions:        获取右电机状态，调用电机状态方法函数
** input parameters:    motor_station[2]：状态数组
** Returned value:      0
***************************************************************************/
uint8_t  Get_Right_Motor_Station(uint16_t motor_station[2])
{
return  Get_Motor_Station(right_wheel_para_handle_p,motor_station);
}
/***************************************************************************
** Function name:       Get_Left_Motor_Station
** Descriptions:        获取左电机状态，调用电机状态方法函数
** input parameters:    motor_station[2]：状态数组
** Returned value:      0
***************************************************************************/
uint8_t  Get_Left_Motor_Station(uint16_t motor_station[2])
{
return  Get_Motor_Station(left_wheel_para_handle_p,motor_station);
}
