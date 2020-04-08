/**
  ******************************************************************************
  * @file    WHEEL.c
  * @author  小鱼儿飞丫飞
  * @version V0.0.1
  * @date    2019/7/30
  * @brief   伽利略机器人轮子控制
  ******************************************************************************/
 #include "wheel.h"
 #include "stdio.h"
 
 /*函数*/
 /*速度更新*/
uint8_t Wheel_Speed_Updata(uint8_t motor_switch);
 /*初始化*/
uint8_t Wheel_Iint(uint8_t motor_switch);
 /*反初始化*/
uint8_t Wheel_Deinit(uint8_t motor_switch);
 /*轮子启动*/
uint8_t Wheel_start(uint8_t motor_switch,uint32_t speed);
 /*正常刹车*/
uint8_t Wheel_Stop(uint8_t motor_switch);
 /*紧急刹车*/
uint8_t Wheel_Brake(uint8_t motor_switch);
/*计算轮子里程*/
uint32_t Count_Wheel_Walk_Distance(uint8_t motor_switch);
/*获取轮子距离*/
uint32_t Get_Wheel_Walk_Distance(uint8_t motor_switch);
/*轮子距离清零*/
uint8_t Wheel_Walk_Distance_Reset(uint8_t motor_switch);
/*读取速度*/
static int32_t Read_Speed(ST_wheel_variable_p wheel_variable_p);
//--左电机
 /*速度更新*/
uint8_t Left_Wheel_Speed_Updata(void);
 /*初始化*/
uint8_t Left_Wheel_Iint(void);
 /*反初始化*/
uint8_t Left_Wheel_Deinit(void);
 /*轮子启动*/
uint8_t Left_Wheel_start(uint32_t speed);
 /*正常刹车*/
uint8_t Left_Wheel_Stop(void);
 /*紧急刹车*/
uint8_t Left_Wheel_Brake(void);
/*计算轮子里程*/
uint32_t Count_Left_Wheel_Walk_Distance(void);
/*获取轮子距离*/
uint32_t Get_Left_Wheel_Walk_Distance(void);
/*轮子距离清零*/
uint8_t Left_Wheel_Walk_Distance_Reset(void);
/*读取速度*/
int32_t Left_Wheel_Read_Speed(void);
//--右电机
 /*速度更新*/
uint8_t Right_Wheel_Speed_Updata(void);
 /*初始化*/
uint8_t Right_Wheel_Iint(void);
 /*反初始化*/
uint8_t Right_Wheel_Deinit(void);
 /*轮子启动*/
uint8_t Right_Wheel_start(uint32_t speed);
 /*正常刹车*/
uint8_t Right_Wheel_Stop(void);
 /*紧急刹车*/
uint8_t Right_Wheel_Brake(void);
/*计算轮子里程*/
uint32_t Count_Right_Wheel_Walk_Distance(void);
/*获取轮子距离*/
uint32_t Get_Right_Wheel_Walk_Distance(void);
/*轮子距离清零*/
uint8_t Right_Wheel_Walk_Distance_Reset(void);
/*读取速度*/
int32_t Right_Wheel_Read_Speed(void);

/*参数*/
ST_wheel_variable left_wheel_variable = {
  0,
	0,
	0,
	0
};
ST_wheel_variable_p  left_wheel_variable_p = &left_wheel_variable;
ST_wheel_variable right_wheel_variable = {
  0,
	0,
	0,
	0
};
ST_wheel_variable_p  right_wheel_variable_p = &right_wheel_variable;
 /*函数接口*/
ST_wheel_function_interface_handle   left_wheel_handle = {
Left_Wheel_Speed_Updata,
Left_Wheel_Iint,
Left_Wheel_Deinit,
Left_Wheel_start,
Left_Wheel_Stop,
Left_Wheel_Brake,
Count_Left_Wheel_Walk_Distance,
Get_Left_Wheel_Walk_Distance,
Left_Wheel_Walk_Distance_Reset,
Left_Wheel_Read_Speed
};
ST_wheel_function_interface_handle_p left_wheel_handle_p = &left_wheel_handle;

ST_wheel_function_interface_handle   right_wheel_handle = {
Right_Wheel_Speed_Updata,
Right_Wheel_Iint,
Right_Wheel_Deinit,
Right_Wheel_start,
Right_Wheel_Stop,
Right_Wheel_Brake,
Count_Right_Wheel_Walk_Distance,
Get_Right_Wheel_Walk_Distance,
Right_Wheel_Walk_Distance_Reset,
Right_Wheel_Read_Speed
};
ST_wheel_function_interface_handle_p right_wheel_handle_p = &right_wheel_handle;
 
 

/***************************************************************************
** Function name:       Wheel_Speed_Updata
** Descriptions:        轮子速度更新
**                      受到测速周期的限制，目前测速周期为100ms  10hz 
**                      所以本函数每100ms调用一次
** input parameters:    motor_switc：电机选择，可以取一下值：
**                                  LEFT_MOTOR ： 左侧电机
**                                  RIGHT_MOTOR： 右侧电机
**                                  MOWING_MOTOR：割草电机
** Returned value:      0
***************************************************************************/
 uint8_t Wheel_Speed_Updata(uint8_t motor_switch)
 {
  if(motor_switch == RIGHT_MOTOR)
	 {	
		
		 /*更新速度*/
		right_speed_handle_p->refresh_speed();	 
		if(car_handle_p->read_car_start_stop() == CAR_START)/*小车启动*/
		 {	 
		  /*更新PWM*/
		  right_pid_handle_p->wheel_integral_separation_pid(1000);	
		 /*更新PWM*/
		 right_motor_handle_p->start(right_pid_handle_p->read_pwm());	
		 }
	 }
	 if(motor_switch == LEFT_MOTOR)
	 { 	

		 /*更新速度*/
		left_speed_handle_p->refresh_speed();
		if(car_handle_p->read_car_start_stop() == CAR_START)/*小车启动*/
		{	
		/*更新PWM*/
		left_pid_handle_p->wheel_integral_separation_pid(1000);
		/*更新PWM*/
		left_motor_handle_p->start(left_pid_handle_p->read_pwm());/*电机启动*/		
		}
	 }
	 return 0;
 }
/***************************************************************************
** Function name:       Wheel_Iint
** Descriptions:        轮子初始化
**                      电机驱动初始化
**                      测速初始化
** input parameters:    motor_switc：电机选择，可以取一下值：
**                                  LEFT_MOTOR ： 左侧电机
**                                  RIGHT_MOTOR： 右侧电机
**                                  MOWING_MOTOR：割草电机
** Returned value:      0
***************************************************************************/
 uint8_t Wheel_Iint(uint8_t motor_switch)
 {
	 /*电机选择*/
	 if(motor_switch == RIGHT_MOTOR)
	 {
	   /*电机初始化*/	 
		right_motor_handle_p->init();
	  /*速度初始化*/
		right_speed_handle_p->speed_init();
	 }
	 if(motor_switch == LEFT_MOTOR)
	 {
	  /*电机初始化*/	
     left_motor_handle_p->init();		 
	  /*速度初始化*/
		 left_speed_handle_p->speed_init();
	 }
	 return 0;
 }
/***************************************************************************
** Function name:       Wheel_Deinit
** Descriptions:        轮子初始化
**                      电机驱动初始化
**                      测速初始化
** input parameters:    motor_switc：电机选择，可以取一下值：
**                                  LEFT_MOTOR ： 左侧电机
**                                  RIGHT_MOTOR： 右侧电机
**                                  MOWING_MOTOR：割草电机
** Returned value:      0
***************************************************************************/
  uint8_t Wheel_Deinit(uint8_t motor_switch)
 { 
	 /*电机选择*/
	 if(motor_switch == RIGHT_MOTOR)
	 {
	   /*电机初始化*/	 
		right_motor_handle_p->deinit();
	  /*速度初始化*/
		right_speed_handle_p->speed_deinit();
	 }
	 if(motor_switch == LEFT_MOTOR)
	 {
	  /*电机初始化*/	
     left_motor_handle_p->deinit();		 
	  /*速度初始化*/
		 left_speed_handle_p->speed_deinit();
	 }
	 return 0;
 }
  /***************************************************************************
** Function name:     Wheel_start
** Descriptions:       轮子启动
** input parameters:    motor_switc：电机选择，可以取一下值：
**                                  LEFT_MOTOR ： 左侧电机
**                                  RIGHT_MOTOR： 右侧电机
**                                  MOWING_MOTOR：割草电机
** Returned value:                  0
***************************************************************************/
 uint8_t Wheel_start(uint8_t motor_switch,uint32_t speed)
 { 
	 /*电机选择*/
	 if(motor_switch == RIGHT_MOTOR) 
		 right_pid_handle_p->set_wheel_pid_speed(speed);
	 if(motor_switch == LEFT_MOTOR)
		 left_pid_handle_p->set_wheel_pid_speed(speed);			 
  return 0;	 
 }
  /***************************************************************************
** Function name:       Wheel_Stop
** Descriptions:        轮子停止；此模式下轮子会因为惯性继续转动一段距离
** input parameters:    motor_switc：电机选择，可以取一下值：
**                                  LEFT_MOTOR ： 左侧电机
**                                  RIGHT_MOTOR： 右侧电机
**                                  MOWING_MOTOR：割草电机
** Returned value:                  0
***************************************************************************/
uint8_t Wheel_Stop(uint8_t motor_switch)
 {

	 
	 /*电机选择*/
	 if(motor_switch == RIGHT_MOTOR)
	 {
		  right_motor_handle_p->stop();
		 /*PWM参数清零*/
		 right_pid_handle_p->pwm_value_reset();

	 }
	 if(motor_switch == LEFT_MOTOR) 
	 {
		 left_motor_handle_p->stop();
		 /*PWM参数清零*/
		 left_pid_handle_p->pwm_value_reset();
	 }
	 return 0;
 }
/***************************************************************************
** Function name:       Wheel_Brake
** Descriptions:        轮子刹车：此模式下轮子会直接抱死
** input parameters:    motor_switc：电机选择，可以取一下值：
**                                  LEFT_MOTOR ： 左侧电机
**                                  RIGHT_MOTOR： 右侧电机
**                                  MOWING_MOTOR：割草电机
** Returned value:                  0
***************************************************************************/
   uint8_t Wheel_Brake(uint8_t motor_switch)
 {
	 /*电机选择*/
	 if(motor_switch == RIGHT_MOTOR)
	 {
		 right_motor_handle_p->brake();
		  /*PWM参数清零*/
		 right_pid_handle_p->pwm_value_reset();

	 }
	 if(motor_switch == LEFT_MOTOR)	 
	 {
		 left_motor_handle_p->brake();
		 		 /*PWM参数清零*/
		 left_pid_handle_p->pwm_value_reset();

	 }
	 return 0;
 }
   /***************************************************************************
** Function name:       Count_Wheel_Walk_Distance
** Descriptions:        计算轮子里程 受到测速周期限制，目前100ms调用一次
** input parameters:    motor_switc：电机选择，可以取一下值：
**                                  LEFT_MOTOR ： 左侧电机
**                                  RIGHT_MOTOR： 右侧电机
**                                  MOWING_MOTOR：割草电机
** Returned value:       0
***************************************************************************/
uint32_t Count_Wheel_Walk_Distance(uint8_t motor_switch)
{
	/*电机选择*/
	 if(motor_switch == RIGHT_MOTOR)
	 {
		right_wheel_variable_p->wheel_mm += right_speed_handle_p->get_motor_cnt()*10*PI*WHEEL_D/3/REDUCTION_RATIO/1000;//--距离单位mm;
	  right_wheel_handle_p->walk_distance_reset();//--清除轮子脉冲计数值
	 }
	 if(motor_switch == LEFT_MOTOR) 
	 {
	 left_wheel_variable_p->wheel_mm += left_speed_handle_p->get_motor_cnt()*10*PI*WHEEL_D/3/REDUCTION_RATIO/1000;//--距离单位mm;
	 left_wheel_handle_p->walk_distance_reset();//--清除轮子脉冲计数值
	 }
	 return 0;
}
   /***************************************************************************
** Function name:       Get_Wheel_Walk_Distance
** Descriptions:        获取轮子里程计 
** input parameters:    motor_switc：电机选择，可以取一下值：
**                                  LEFT_MOTOR ： 左侧电机
**                                  RIGHT_MOTOR： 右侧电机
**                                  MOWING_MOTOR：割草电机
** Returned value:       0
***************************************************************************/
uint32_t Get_Wheel_Walk_Distance(uint8_t motor_switch)
{
uint32_t distance = 0;
	/*电机选择*/
	 if(motor_switch == RIGHT_MOTOR)
	 {
   distance = right_wheel_variable_p->wheel_mm;
	 }
	 if(motor_switch == LEFT_MOTOR) 
	 {
   distance = left_wheel_variable_p->wheel_mm;
	 }
	 return distance;
}
 /***************************************************************************
** Function name:       Wheel_Walk_Distance_Reset
** Descriptions:        轮子脉冲数清零
** input parameters:    motor_switc：电机选择，可以取一下值：
**                                  LEFT_MOTOR ： 左侧电机
**                                  RIGHT_MOTOR： 右侧电机
**                                  MOWING_MOTOR：割草电机
** Returned value:     0
***************************************************************************/

uint8_t Wheel_Walk_Distance_Reset(uint8_t motor_switch)
{
	/*电机选择*/
	 if(motor_switch == RIGHT_MOTOR)
		right_speed_handle_p->motor_cnt_reset();
	 if(motor_switch == LEFT_MOTOR) 
    left_speed_handle_p->motor_cnt_reset();
	 return 0;
}

 /***************************************************************************
** Function name:       Read_Speed
** Descriptions:        读取轮子速度
** input parameters:    motor_switc：电机选择，可以取一下值：
**                                  LEFT_MOTOR ： 左侧电机
**                                  RIGHT_MOTOR： 右侧电机
**                                  MOWING_MOTOR：割草电机
** Returned value:     0
***************************************************************************/
static int32_t Read_Speed(ST_wheel_variable_p wheel_variable_p)
{
 if(wheel_variable_p == left_wheel_variable_p)
	 wheel_variable_p->speed = left_speed_handle_p->read_speed();
 if(wheel_variable_p == right_wheel_variable_p)
	 wheel_variable_p->speed = right_speed_handle_p->read_speed();
 return wheel_variable_p->speed;
}
/*读取左轮速度*/
int32_t Left_Wheel_Read_Speed(void)
{
 return Read_Speed(left_wheel_variable_p);
}
/*读取右轮速度*/
int32_t Right_Wheel_Read_Speed(void)
{
 return Read_Speed(right_wheel_variable_p);
}
//--左电机
 /*速度更新*/
uint8_t Left_Wheel_Speed_Updata(void)
{
 return Wheel_Speed_Updata(LEFT_MOTOR);
}
 /*初始化*/
uint8_t Left_Wheel_Iint(void)
{
return Wheel_Iint(LEFT_MOTOR);
}
 /*反初始化*/
uint8_t Left_Wheel_Deinit(void)
{
return Wheel_Deinit(LEFT_MOTOR);
}
 /*轮子启动*/
uint8_t Left_Wheel_start(uint32_t speed)
{
return Wheel_start(LEFT_MOTOR,speed);
}
 /*正常刹车*/
uint8_t Left_Wheel_Stop(void)
{
 return Wheel_Stop(LEFT_MOTOR);
}
 /*紧急刹车*/
uint8_t Left_Wheel_Brake(void)
{
 return Wheel_Brake(LEFT_MOTOR);
}
/*计算轮子里程*/
uint32_t Count_Left_Wheel_Walk_Distance(void)
{
 return Count_Wheel_Walk_Distance(LEFT_MOTOR);
}
/*获取轮子距离*/
uint32_t Get_Left_Wheel_Walk_Distance(void)
{
 return Get_Wheel_Walk_Distance(LEFT_MOTOR);
}
/*轮子距离清零*/
uint8_t Left_Wheel_Walk_Distance_Reset(void)
{
 return  Wheel_Walk_Distance_Reset(LEFT_MOTOR);
}
//--右电机
 /*速度更新*/
uint8_t Right_Wheel_Speed_Updata(void)
{
 return Wheel_Speed_Updata(RIGHT_MOTOR);
}
 /*初始化*/
uint8_t Right_Wheel_Iint(void)
{
return Wheel_Iint(RIGHT_MOTOR);
}
 /*反初始化*/
uint8_t Right_Wheel_Deinit(void)
{
return Wheel_Deinit(RIGHT_MOTOR);
}
 /*轮子启动*/
uint8_t Right_Wheel_start(uint32_t speed)
{
return Wheel_start(RIGHT_MOTOR,speed);
}
 /*正常刹车*/
uint8_t Right_Wheel_Stop(void)
{
 return Wheel_Stop(RIGHT_MOTOR);
}
 /*紧急刹车*/
uint8_t Right_Wheel_Brake(void)
{
 return Wheel_Brake(RIGHT_MOTOR);
}
/*计算轮子里程*/
uint32_t Count_Right_Wheel_Walk_Distance(void)
{
 return Count_Wheel_Walk_Distance(RIGHT_MOTOR);
}
/*获取轮子距离*/
uint32_t Get_Right_Wheel_Walk_Distance(void)
{
 return Get_Wheel_Walk_Distance(RIGHT_MOTOR);
}
/*轮子距离清零*/
uint8_t Right_Wheel_Walk_Distance_Reset(void)
{
 return  Wheel_Walk_Distance_Reset(RIGHT_MOTOR);
}
