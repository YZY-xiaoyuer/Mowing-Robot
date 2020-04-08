/**
  ******************************************************************************
  * @file    pid.c
  * @author  小鱼儿飞丫飞
  * @version V0.0.1
  * @date    2019/7/25
  * @brief   伽利略机器人PID算法
  ******************************************************************************        
*/

#include "pid.h"
#include "shell.h"
#include "stdio.h"

/*===========函数==============*/
/*轮子速度PID调节*/
int32_t  Wheel_Integral_Separation_Pid(ST_pid_variable_p pid_variable_p,int32_t rang);
/*左轮速度PID调节*/
int32_t  Left_Wheel_Integral_Separation_Pid(int32_t rang);
/*右轮速度PID调节*/
int32_t  Right_Wheel_Integral_Separation_Pid(int32_t rang);

uint8_t Set_Wheel_Pid_Speed(ST_pid_variable_p pid_variable_p,int32_t to_speed);
uint8_t Set_Left_Wheel_Pid_Speed(int32_t to_speed);
uint8_t Set_Right_Wheel_Pid_Speed(int32_t to_speed);

int32_t Read_Pwm(ST_pid_variable_p pid_variable_p);
int32_t Read_Left_Wheel_Pwm(void);
int32_t Read_Right_Wheel_Pwm(void);


uint8_t Set_Pid_Value(ST_pid_variable_p pid_variable_p,uint16_t kp,uint16_t ki, uint16_t kd);
uint8_t Set_Left_Wheel_Pid_Value(uint16_t kp,uint16_t ki, uint16_t kd);
uint8_t Set_Right_Wheel_Pid_Value(uint16_t kp,uint16_t ki, uint16_t kd);
uint8_t Set_Car_Walk_Distance_Pid_Value(uint16_t kp,uint16_t ki, uint16_t kd);
uint8_t Set_Car_Walk_Yaw_Pid_Value(uint16_t kp,uint16_t ki, uint16_t kd);

uint8_t Pwm_Value_Reset(ST_pid_variable_p pid_variable_p);
uint8_t Left_Wheel_Pwm_Value_Reset(void);
uint8_t Right_Wheel_Pwm_Value_Reset(void);

int32_t Car_Walk_Distance_PID(int32_t speed,uint32_t now_distance ,uint32_t to_distance);
int32_t Car_Walk_Yaw_Pid(int32_t yaw_err);
uint8_t Car_Walk_Distance_Pid_Value_Reset(void);
uint8_t Car_Walk_Yaw_Pid_Value_Reset(void);
/*左轮PID*/
ST_pid_variable left_pid_variable = {
  0,
	0,
  0,
  0,
  0,
	0,
	0,
	1000,   //--P
	100,   //--I
	0,   //--D
	0,
	2500,
 -2500,
	500,//--mm/s  0.48m/s
 -500
};
ST_pid_variable_p left_pid_variable_p = & left_pid_variable;
/*右轮PID*/
ST_pid_variable right_pid_variable = {
	0,
  0,
  0,
  0,
  0,
	0,
	0,
	1000,   //--P
	100,   //--I
	0,   //--D
	0,
	2500,  
 -2500, 
	500,//--mm/s  0.48m/s
 -500
};
ST_pid_variable_p right_pid_variable_p = & right_pid_variable;
/*小车走直线PID*/
ST_pid_variable car_walk_distance_pid = {
	0,
  0,
  0,
  0,
  0,
	0,
	0,
  60,   //--P
	0,   //--I
	0,   //--D
	0,
	0, 
  0,  
	0,//--mm/s  0.48m/s
  0
};
ST_pid_variable_p car_walk_distance_pid_p = &car_walk_distance_pid;

ST_pid_variable car_walk_yaw_pid = {
	0,
  0,
  0,
  0,
  0,
	0,
	0,
	1800,   //--P
	100,   //--I
	0,   //--D
	0,
	0, 
  0,  
	0,//--mm/s  0.48m/s
  0
};
ST_pid_variable_p car_walk_yaw_pid_p = &car_walk_yaw_pid;

/*PID函数接口接口*/
ST_pid_function_interface_handle   left_pid_handle = {
Left_Wheel_Integral_Separation_Pid,
Set_Left_Wheel_Pid_Speed,
Read_Left_Wheel_Pwm,
Set_Left_Wheel_Pid_Value,
Left_Wheel_Pwm_Value_Reset
};
ST_pid_function_interface_handle_p left_pid_handle_p = &left_pid_handle;

ST_pid_function_interface_handle   right_pid_handle = {
Right_Wheel_Integral_Separation_Pid,
Set_Right_Wheel_Pid_Speed,
Read_Right_Wheel_Pwm,
Set_Right_Wheel_Pid_Value,
Right_Wheel_Pwm_Value_Reset
};
ST_pid_function_interface_handle_p right_pid_handle_p = &right_pid_handle;

ST_car_pid_function_interface_handle   car_pid_handle = {
Car_Walk_Distance_PID,
Car_Walk_Yaw_Pid,
Car_Walk_Distance_Pid_Value_Reset,
Car_Walk_Yaw_Pid_Value_Reset,
Set_Car_Walk_Distance_Pid_Value,
Set_Car_Walk_Yaw_Pid_Value
};
ST_car_pid_function_interface_handle_p car_pid_handle_p = &car_pid_handle;

/***************************************************************************
** Function name:       wheel_Integral_Separation_Pid
** Descriptions:        轮子调速积分分离PID算法实现
** input parameters:     pid_variable_p :PID参数结构体
**                       range :调速范围
** Returned value:      pwm:PWM占空比
**pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last)
**在普通PID控制中，引入积分环节的目的，只要是为了消除静差，提高控制精度，
**但是在启动、结束或大幅度增减时，短时间内系统输出有很大的偏差，会造成PID运算的积分积累，
**导致控制量超过执行机构可能允许的最大动作范围对应极限控制量，从而引起交大的超调，
**甚至是震荡，这是绝对不允许，为了克服这一问题，引入积分分离的概念，其基本思路是
**当被控量与设定值偏差交大时，取消积分作用，
**当被控量接近给定值时，引入积分控制，以消除静差，提高精度
***************************************************************************/
int32_t  Wheel_Integral_Separation_Pid(ST_pid_variable_p pid_variable_p,int32_t rang)
 { 
	  int32_t pwm = 0;
	  int32_t err = 0;
	  int32_t P_index = 0;
    int32_t I_index = 0;
	  if(pid_variable_p == left_pid_variable_p)
		pid_variable_p->current_value = left_speed_handle_p->read_speed();
		if(pid_variable_p == right_pid_variable_p)
		pid_variable_p->current_value = right_speed_handle_p->read_speed();
		/*计算当前误差*/
	  err = pid_variable_p->current_err = pid_variable_p->to_value 
		                                   - pid_variable_p->current_value;
		
		//--100 1000
		/*积分分离*/
	  if(err <0)err = -err; 
	  if(err>20)
    {
			P_index = 2;
			I_index = 0;
    }
		else if (err<10)
		{
     P_index = 1;
		I_index = 2;
	  pid_variable_p->integral += pid_variable_p->current_err;//--累加积分误差
		}
		else{
    P_index = 1; 	
		I_index = 1;
    pid_variable_p->integral += pid_variable_p->current_err;//--累加积分误差
    }
    pwm = 
	             (P_index*pid_variable_p->kp*pid_variable_p->current_err
	             +I_index*pid_variable_p->ki*pid_variable_p->integral
	             +pid_variable_p->kd*(pid_variable_p->current_err-pid_variable_p->last_err))/1000;

   // pwm2+=pwm1;	
		rang = range(rang, 0, 2500);/*范围控制本身限制*/
		pid_variable_p->output_add+=range(pwm, -rang, rang);/*PWM调速速率控制*/
		pid_variable_p->output_value =range(pid_variable_p->output_add,pid_variable_p->PWM_umin, pid_variable_p->PWM_umax);//--PWM范围控制
    pid_variable_p->last_err = pid_variable_p->current_err;//--更新上一次误差
    return pid_variable_p->output_value;
 }
/*左轮速度PID调节*/
int32_t  Left_Wheel_Integral_Separation_Pid(int32_t rang)
{
return Wheel_Integral_Separation_Pid(left_pid_variable_p,rang);
}
/*右轮速度PID调节*/
int32_t  Right_Wheel_Integral_Separation_Pid(int32_t rang)
{
return Wheel_Integral_Separation_Pid(right_pid_variable_p,rang);
}
/***************************************************************************
** Function name:       Set_Wheel_Pid_Speed
** Descriptions:        设置PID的目标值
** input parameters:    pid_variable_p：PID参数结构体  to_speed：期望值
***************************************************************************/
uint8_t Set_Wheel_Pid_Speed(ST_pid_variable_p pid_variable_p,int32_t to_speed)
{
	pid_variable_p->to_value = to_speed;
	return 0;
}
uint8_t Set_Left_Wheel_Pid_Speed(int32_t to_speed)
{
 Set_Wheel_Pid_Speed(left_pid_variable_p,to_speed);
 return 0;
}
uint8_t Set_Right_Wheel_Pid_Speed(int32_t to_speed)
{
 Set_Wheel_Pid_Speed(right_pid_variable_p,to_speed);
 return 0;
}
/***************************************************************************
** Function name:       Read_Pwm
** Descriptions:        读取PID计算出来的PWM值
** input parameters:    pid_variable_p：PID参数结构体
***************************************************************************/
int32_t Read_Pwm(ST_pid_variable_p pid_variable_p)
{
return pid_variable_p->output_value;
}
int32_t Read_Left_Wheel_Pwm(void)
{
 return Read_Pwm(left_pid_variable_p);
}
int32_t Read_Right_Wheel_Pwm(void)
 {
 return Read_Pwm(right_pid_variable_p);
 }

/***************************************************************************
** Function name:       Set_Pid_Value
** Descriptions:        设置PID参数
** input parameters:    pid_variable_p：PID参数结构体 kp ki kd
***************************************************************************/
uint8_t Set_Pid_Value(ST_pid_variable_p pid_variable_p,uint16_t kp,uint16_t ki, uint16_t kd)
{
  pid_variable_p->kp = kp;
	pid_variable_p->ki = ki;
	pid_variable_p->kd = kd;
	return 0;
}

uint8_t Set_Left_Wheel_Pid_Value(uint16_t kp,uint16_t ki, uint16_t kd)
{
 return Set_Pid_Value(left_pid_variable_p,
	                    kp,
                      ki, 
                      kd);
}
uint8_t Set_Right_Wheel_Pid_Value(uint16_t kp,uint16_t ki, uint16_t kd)
{
 return Set_Pid_Value(right_pid_variable_p,
	                    kp,
                      ki, 
                      kd);
}
uint8_t Set_Car_Walk_Distance_Pid_Value(uint16_t kp,uint16_t ki, uint16_t kd)
{
 return Set_Pid_Value(car_walk_distance_pid_p,
	                    kp,
                      ki, 
                      kd);
}
uint8_t Set_Car_Walk_Yaw_Pid_Value(uint16_t kp,uint16_t ki, uint16_t kd)
{
 return Set_Pid_Value(car_walk_yaw_pid_p,
	                    kp,
                      ki, 
                      kd);
}
/***************************************************************************
** Function name:      Pwm_Value_Reset
** Descriptions:       PWM运算过程用到的中间变量清零
** input parameters:    pid_variable_p：PID参数结构体
***************************************************************************/
uint8_t Pwm_Value_Reset(ST_pid_variable_p pid_variable_p)
{
  pid_variable_p->to_value = 0;
  pid_variable_p->current_err = 0;
	pid_variable_p->current_value = 0;
	pid_variable_p->integral = 0;
	pid_variable_p->last_err = 0;
	pid_variable_p->last_last_err = 0;
	pid_variable_p->output_value = 0;
	pid_variable_p->output_add = 0;
	return 0;
}
uint8_t Left_Wheel_Pwm_Value_Reset(void)
{
 return Pwm_Value_Reset(left_pid_variable_p);
}
uint8_t Right_Wheel_Pwm_Value_Reset(void)
{
 return Pwm_Value_Reset(right_pid_variable_p);
}
uint8_t Car_Walk_Distance_Pid_Value_Reset(void)
{
return Pwm_Value_Reset(car_walk_distance_pid_p);
}
uint8_t Car_Walk_Yaw_Pid_Value_Reset(void)
{
return Pwm_Value_Reset(car_walk_yaw_pid_p);
}
/***************************************************************************
** Function name:       Car_Walk_Distance_PID()
** Descriptions:        小车末端减速PID调节
** input parameters:    speed:小车速度
                        now_distance：当前行驶的距离
**                      to_distance：将要行驶的距离
** Returned value:      speed:小车速度
**pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last)
**在普通PID控制中，引入积分环节的目的，只要是为了消除静差，提高控制精度，
**但是在启动、结束或大幅度增减时，短时间内系统输出有很大的偏差，会造成PID运算的积分积累，
**导致控制量超过执行机构可能允许的最大动作范围对应极限控制量，从而引起交大的超调，
**甚至是震荡，这是绝对不允许，为了克服这一问题，引入积分分离的概念，其基本思路是
**当被控量与设定值偏差交大时，取消积分作用，
**当被控量接近给定值时，引入积分控制，以消除静差，提高精度
***************************************************************************/
int32_t Car_Walk_Distance_PID(int32_t speed,uint32_t now_distance ,uint32_t to_distance)
{
    int32_t speed1 = 0;
	  int32_t err = 0;

		/*计算当前误差*/
	  err = to_distance - now_distance;
		if(err>200)
		{	
	    return speed;
		}
		else{//--20cm的时候减速
					
		 speed1 = (car_walk_distance_pid_p->kp*err)/1000;
     if(speed <0)speed +=speed1;
			else
	    speed -= speed1;	
			
		 if((speed<=80)&&(speed>=-80))
		 {
			if(speed<0)
		 return speed = -80;
			if(speed>0)
			return speed = 80;
		 }
		 return speed;
		}
	}
/***************************************************************************
** Function name:       Car_Walk_Yaw_Pid()
** Descriptions:        小车航向PID控制
** input parameters:    yaw_err：直线行走航向偏差值
** Returned value:      pwm:PWM占空比
**pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last)
**在普通PID控制中，引入积分环节的目的，只要是为了消除静差，提高控制精度，
**但是在启动、结束或大幅度增减时，短时间内系统输出有很大的偏差，会造成PID运算的积分积累，
**导致控制量超过执行机构可能允许的最大动作范围对应极限控制量，从而引起交大的超调，
**甚至是震荡，这是绝对不允许，为了克服这一问题，引入积分分离的概念，其基本思路是
**当被控量与设定值偏差交大时，取消积分作用，
**当被控量接近给定值时，引入积分控制，以消除静差，提高精度
***************************************************************************/
int32_t Car_Walk_Yaw_Pid(int32_t yaw_err)
{
    int32_t speed = 0;
	  int32_t err = 0;
    int32_t P_index = 0;
    int32_t I_index = 0;
		/*计算当前误差*/

	 /*如果偏差大于180，从反方向回正*/
    if(yaw_err>18000)
		{
		yaw_err = yaw_err-36000;
		}	
    if(yaw_err<-18000)
		{yaw_err = yaw_err +36000;}			
		/*---------------------------*/
		 if(yaw_err <0)err = -yaw_err; 
		 else err = yaw_err;
	  if(err>2000)
    {
			P_index = 2;
			I_index = 0;
    }
		else if (err<1000)
		{
     P_index = 1;
		 I_index = 2;
	   car_walk_yaw_pid_p->integral +=yaw_err;//--累加积分误差
		}
		else{
     P_index = 1; 	
		 //I_index = 1;
     car_walk_yaw_pid_p->integral += yaw_err;//--累加积分误差
    }
    speed = 
	             (P_index*car_walk_yaw_pid_p->kp*yaw_err
	             +I_index*car_walk_yaw_pid_p->ki*car_walk_yaw_pid_p->integral
	             +car_walk_yaw_pid_p->kd*(err-car_walk_yaw_pid_p->last_err))/100000;

		car_walk_yaw_pid_p->output_add =speed;
		car_walk_yaw_pid_p->output_value =range(speed,-500, 500);//--PWM范围控制
    car_walk_yaw_pid_p->last_err = yaw_err;//--更新上一次误差

		return car_walk_yaw_pid_p->output_value;
	}
/***************************************************************************
** Function name:       Car_Integral_Separation_Pid
** Descriptions:        小车走直线积分分离PID算法实现
** input parameters:    motor_switc：电机选择，可以取一下值：
**                                  LEFT_MOTOR ： 左侧电机
**                                  RIGHT_MOTOR： 右侧电机
**                                  MOWING_MOTOR：割草电机
**                      to_speed : 目标速度
** Returned value:      pwm:PWM占空比
**pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last)
**在普通PID控制中，引入积分环节的目的，只要是为了消除静差，提高控制精度，
**但是在启动、结束或大幅度增减时，短时间内系统输出有很大的偏差，会造成PID运算的积分积累，
**导致控制量超过执行机构可能允许的最大动作范围对应极限控制量，从而引起交大的超调，
**甚至是震荡，这是绝对不允许，为了克服这一问题，引入积分分离的概念，其基本思路是
**当被控量与设定值偏差交大时，取消积分作用，
**当被控量接近给定值时，引入积分控制，以消除静差，提高精度
***************************************************************************/
//uint16_t  Car_Integral_Separation_Pid(int32_t yaw_err){
//	 

//	 int32_t index = 0;
//   int32_t out_value1;
//	 int32_t out_value2;
//	 int32_t err = pid_variable_p->current_value = yaw_err;//--当前误差
//	  if(err <0)err = -err; 
//	  if(err>200)//--积分分离
//    {
//			index = 0;
//    }
//		else if (err<100)
//		{
//		index = 2;
//	  pid_variable_p->integral += pid_variable_p->current_err;//--累加积分误差
//		}
//		else{
//		index = 1;
//    pid_variable_p->integral += pid_variable_p->current_err;//--累加积分误差
//    }
//    out_value1 = 
//	             (pid_variable_p->kp*pid_variable_p->current_err
//	             +index*pid_variable_p->ki*pid_variable_p->integral
//	             +pid_variable_p->kd*(pid_variable_p->current_err-pid_variable_p->last_err))/1000;
//     out_value2+=out_value1;		
//		//pwm2+=range(pwm1, -rang1, rang1);//--PWM范围控制
//		pid_variable_p->output_value =range(out_value2,pid_variable_p->umin, pid_variable_p->umax);//--PWM范围控制
//    pid_variable_p->last_err = pid_variable_p->current_err;//--更新上一次误差
//    return pid_variable_p->output_value;
//}
/***************************************************************************
** Function name:       Position_Pid
** Descriptions:        位置型PID的C语言实现
** input parameters:    motor_switc：电机选择，可以取一下值：
**                                  LEFT_MOTOR ： 左侧电机
**                                  RIGHT_MOTOR： 右侧电机
**                                  MOWING_MOTOR：割草电机
**                      to_speed : 目标速度
** Returned value:      pwm:PWM占空比
**pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last)
**没有考虑死区，没有设定上下限，只是对公式的一种最直接的实现
***************************************************************************/
//int32_t  Position_Pid(uint8_t motor_switch,uint16_t to_speed){
//	  if(motor_switch == RIGHT_MOTOR)pid_variable_p = right_pid_variable_p;
//	  else if(motor_switch == LEFT_MOTOR) pid_variable_p = left_pid_variable_p;
//	  int32_t pwm;
//    pid_variable_p->to_value = to_speed;//--设置目标速度	//--放大100倍  
//	  pid_variable_p->current_value = speed_function_interface_handle_p->read_speed(motor_switch);//--当前速度
//	  pid_variable_p->current_value = pid_variable_p->to_value - pid_variable_p->current_value;//--当前误差
//	  //printf("current_err %d\r\n",pid_variable_p->current_value);
//	  pid_variable_p->integral += pid_variable_p->current_value;//--累加积分误差
//    pwm += 
//	              (pid_variable_p->kp*pid_variable_p->current_err
//	             +pid_variable_p->ki*pid_variable_p->integral
//	             +pid_variable_p->kd*(pid_variable_p->current_err-pid_variable_p->last_err))/1000;
//    pid_variable_p->output_value =range(pwm,pid_variable_p->umin, pid_variable_p->umax);//--PWM范围控制
//	  pid_variable_p->last_err = pid_variable_p->current_err;//--更新上一次误差
//    return pid_variable_p->output_value;
//}
/***************************************************************************
** Function name:       Incremental_Pid
** Descriptions:        增量型PID算法的C语言实现
** input parameters:    motor_switc：电机选择，可以取一下值：
**                                  LEFT_MOTOR ： 左侧电机
**                                  RIGHT_MOTOR： 右侧电机
**                                  MOWING_MOTOR：割草电机
**                      to_speed : 目标速度
** Returned value:      pwm:PWM占空比
**pid.Kp*(pid.err-pid.err_next)+pid.Ki*pid.err+pid.Kd*(pid.err-2*pid.err_next+pid.err_last)
**没有考虑死区，没有设定上下限，只是对公式的一种最直接的实现
***************************************************************************/
//int32_t Incremental_Pid(uint8_t motor_switch,uint16_t to_speed){

//	  if(motor_switch == RIGHT_MOTOR)pid_variable_p = right_pid_variable_p;
//	  else if(motor_switch == LEFT_MOTOR) pid_variable_p = left_pid_variable_p;
//	  int32_t pwm;

//    pid_variable_p->to_value = to_speed;//--设置目标速度
//	  pid_variable_p->current_value = speed_function_interface_handle_p->read_speed(motor_switch);//--当前速度
//    pid_variable_p->current_err = pid_variable_p->to_value - pid_variable_p->current_value;//--当前误差
//   // printf("current_err %d\r\n",pid_variable_p->current_err);  
//	  pwm += 
//	            (pid_variable_p->kp*(pid_variable_p->current_err-pid_variable_p->last_err)
//	           +pid_variable_p->ki*pid_variable_p->current_err
//	           +pid_variable_p->kd*(pid_variable_p->current_err-2*pid_variable_p->last_err
//	           +pid_variable_p->last_last_err))/1000;
//    pid_variable_p->output_value =range(pwm,pid_variable_p->umin, pid_variable_p->umax);//--PWM范围控制
//    pid_variable_p->last_last_err=pid_variable_p->last_err;
//    pid_variable_p->last_err=pid_variable_p->current_err;
//    return pid_variable_p->output_value;
//}
/***************************************************************************
** Function name:       Anti_Integral_Saturation_Pid
** Descriptions:        抗积分饱和PID算法的C语言实现
** input parameters:    motor_switc：电机选择，可以取一下值：
**                                  LEFT_MOTOR ： 左侧电机
**                                  RIGHT_MOTOR： 右侧电机
**                                  MOWING_MOTOR：割草电机
**                      to_speed : 目标速度
** Returned value:      pwm:PWM占空比
**pid.Kp*(pid.err-pid.err_next)+pid.Ki*pid.err+pid.Kd*(pid.err-2*pid.err_next+pid.err_last)
**所谓积分饱和现象是指系统存在一个方向的偏差，PID控制器的输出由于积分作用的不断累加而加大，
**从而导致执行机构达到极限位位置，若控制器输出继续增大，执行器开度不可能再增大，
**此时计算机输出控制量超过了正常运行范围而进入饱和区。一旦系统出现反向偏差，
**控制器输出逐渐从饱和区退出，进入饱和区越深则退出饱和区时间越长。在这段时间，
**执行机构仍然停留在极限位置而不随偏差反向而立即做出相应的改变，这时系统像时空一样，
**造成控制性能恶化，这种现象成为积分饱和现象或积分失控
**防止积分饱和的方法之一就是抗饱和法，该方法的思想是在计算控制器输出的时候，首先判断
**上一时刻的控制量是否已经超出了极限范围，如果上一时刻输出大于极限范围，则只累加正偏差
**从而避免控制量长是哦见停留在饱和区
***************************************************************************/
//int32_t  Anti_Integral_Saturation_Pid(uint8_t motor_switch,uint16_t to_speed){

//	 if(motor_switch == RIGHT_MOTOR)pid_variable_p = right_pid_variable_p;
//	  else if(motor_switch == LEFT_MOTOR) pid_variable_p = left_pid_variable_p; 
//  	pid_variable_p->kp = 0;
//	  pid_variable_p->ki = 0;
//	  pid_variable_p->kd = 0;
//    pid_variable_p->to_value = to_speed;//--设置目标速度
//	  pid_variable_p->current_value = speed_function_interface_handle_p->read_speed(motor_switch);//--当前速度�
//	  pid_variable_p->current_err = pid_variable_p->to_value - pid_variable_p->current_value;//--当前误差
//	  if(pid_variable_p->current_value>pid_variable_p->umax)//--抗饱和//--这里的速度应该换为PWM，用敌枰慕
//		  {
//		   if(pid_variable_p->current_err>200)//--积分分离
//         {
//          pid_variable_p->ki = 0;
//         }else{
//          pid_variable_p->ki = 1;;
//         pid_variable_p->integral += pid_variable_p->current_err;//--累加积分误差
//         }
//		 }
//	 if(pid_variable_p->current_value<pid_variable_p->umin)//--抗饱和
//		  {
//		   if(pid_variable_p->current_err>200)//--积分分离
//         {
//          pid_variable_p->ki = 0;
//         }else{
//          pid_variable_p->ki = 1;;
//         pid_variable_p->integral += pid_variable_p->current_err;//--累加积分误差
//         }
//		 }
//	if(pid_variable_p->current_err>200)//--积分分离
//    {
//   pid_variable_p->ki = 0;
//    }else{
//   pid_variable_p->ki = 1;;
//   pid_variable_p->integral += pid_variable_p->current_err;//--累加积分误差
//    }
//    pid_variable_p->output_value = 
//	              pid_variable_p->kp*pid_variable_p->current_err
//	             +pid_variable_p->ki*pid_variable_p->integral
//	             +pid_variable_p->kd*(pid_variable_p->current_err-pid_variable_p->last_err);
//    pid_variable_p->last_err = pid_variable_p->current_err;//--更新上一次误差
//    return pid_variable_p->output_value;
//}

/***************************************************************************
** Function name:       Trapezoidal_Integraln_Pid
** Descriptions:        梯形积分的PID算法实现
** input parameters:    motor_switc：电机选择，可以取一下值：
**                                  LEFT_MOTOR ： 左侧电机
**                                  RIGHT_MOTOR： 右侧电机
**                                  MOWING_MOTOR：割草电机
**                      to_speed : 目标速度
** Returned value:      pwm:PWM占空比
**pid.Kp*pid.err+index*pid.Ki*pid.integral/2+pid.Kd*(pid.err-pid.err_last);  //--梯形积分
**作为PID控制的积分项，其作用是消除余差，应提高积分项运算精度，为此可以将矩形积分改为梯形积分，
***************************************************************************/
//int32_t  Trapezoidal_Integraln_Pid(uint8_t motor_switch,uint16_t to_speed){

//	  if(motor_switch == RIGHT_MOTOR)pid_variable_p = right_pid_variable_p;
//	  else if(motor_switch == LEFT_MOTOR) pid_variable_p = left_pid_variable_p; 
//	 pid_variable_p->kp = 0;
//	  pid_variable_p->ki = 0;
//	  pid_variable_p->kd = 0;
//    pid_variable_p->to_value = to_speed;//--设置目标速度
//	  pid_variable_p->current_value = speed_function_interface_handle_p->read_speed(motor_switch);//--当前速度�
//	  pid_variable_p->current_err = pid_variable_p->to_value - pid_variable_p->current_value;//--当前误差
//	  if(pid_variable_p->current_value>pid_variable_p->umax)//--抗饱和//--这里的速度应该换为PWM，用敌枰慕
//		  {
//		   if(pid_variable_p->current_err>200)//--积分分离
//         {
//          pid_variable_p->ki = 0;
//         }else{
//          pid_variable_p->ki = 1;;
//         pid_variable_p->integral += pid_variable_p->current_err;//--累加积分误差
//         }
//		 }
//	 if(pid_variable_p->current_value<pid_variable_p->umin)//--抗饱和
//		  {
//		   if(pid_variable_p->current_err>200)//--积分分离
//         {
//          pid_variable_p->ki = 0;
//         }else{
//          pid_variable_p->ki = 1;;
//         pid_variable_p->integral += pid_variable_p->current_err;//--累加积分误差
//         }
//		 }
//	if(pid_variable_p->current_err>200)//--积分分离
//    {
//   pid_variable_p->ki = 0;
//    }else{
//   pid_variable_p->ki = 1;;
//   pid_variable_p->integral += pid_variable_p->current_err;//--累加积分误差
//    }
//    pid_variable_p->output_value = 
//	              pid_variable_p->kp*pid_variable_p->current_err
//	             +pid_variable_p->ki*pid_variable_p->integral/2
//	             +pid_variable_p->kd*(pid_variable_p->current_err-pid_variable_p->last_err);
//    pid_variable_p->last_err = pid_variable_p->current_err;//--更新上一次误差
//    return pid_variable_p->output_value;
//}
/***************************************************************************
** Function name:       Variable_Integraln_Pid
** Descriptions:        变积分的PID算法实现
** input parameters:    motor_switc：电机选择，可以取一下值：
**                                  LEFT_MOTOR ： 左侧电机
**                                  RIGHT_MOTOR： 右侧电机
**                                  MOWING_MOTOR：割草电机
**                      to_speed : 目标速度
** Returned value:      pwm:PWM占空比
**pid.Kp*pid.err+index*pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);  //--梯形积分
**变积分PID可以看成是积分分离的PID算法的更一般的形式！在普通的PID算法中，由于积分系数KI是常数
**所以在整个控制过程中，积分增量式不变的，但是，系统对于积分项的要求是，系统偏差大时，积分作用应该减弱甚至全无
**，而在偏差小时，则应该加强。积分系数取大了会产生超调，甚至积分饱和，取小了又不能短时间消除静差，因此，
**根据系统的偏差大小改变积分速度是有必要的。
**变积分PID的基本思想是设法改变积分项的累加速度，使其与偏差大小相应；偏差越大，积分越慢，偏差越小，积分越快
***************************************************************************/
//int32_t  Variable_Integraln_Pid(uint8_t motor_switch,uint16_t to_speed){

//	  if(motor_switch == RIGHT_MOTOR)pid_variable_p = right_pid_variable_p;
//	  else if(motor_switch == LEFT_MOTOR) pid_variable_p = left_pid_variable_p;  
//	 int32_t pwm1 = 0;
//	  static int32_t pwm2 = 0;
//	  int32_t index;
//	  int32_t rang1 = 0;
//	  pid_variable_p->kp = 200;
//	  pid_variable_p->ki = 50;
//	  pid_variable_p->kd = 0;
//	  pid_variable_p->current_value = speed_function_interface_handle_p->read_speed(motor_switch);//--当前速度�
//	  pid_variable_p->current_err = pid_variable_p->to_value - pid_variable_p->current_value;//--当前误差
//	
//	if(pid_variable_p->current_err>200)//--积分分离
//    {
//    index = 0;
//	  rang1 = 400;
//    }
//	else if(pid_variable_p->current_err<50)
//		{
//   index = 2;
//   pid_variable_p->integral += pid_variable_p->current_err;//--累加积分误差
//    }
//	else
//	{
//	index = (200-pid_variable_p->current_err)/20;
//	index = 1;
//	pid_variable_p->integral += pid_variable_p->current_err;//--累加积分误差
//	}
//    pwm1 = 
//	             ( pid_variable_p->kp*pid_variable_p->current_err
//	             +index*pid_variable_p->ki*pid_variable_p->integral
//	             +pid_variable_p->kd*(pid_variable_p->current_err-pid_variable_p->last_err))/1000;
//	  pwm2+=range(pwm1, -rang1, rang1);//--PWM范围控制
//	  pid_variable_p->output_value =range(pwm2, pid_variable_p->umin, pid_variable_p->umax);//--PWM范围控制
//    pid_variable_p->last_err = pid_variable_p->current_err;//--更新上一次误差
//    return pid_variable_p->output_value;
//}
/***************************************************************************
** Function name:       
** Descriptions:        专家PID与模糊PID的C语言算法实现
** input parameters:    motor_switc：电机选择，可以取一下值：
**                                  LEFT_MOTOR ： 左侧电机
**                                  RIGHT_MOTOR： 右侧电机
**                                  MOWING_MOTOR：割草电机
**                      to_speed : 目标速度
** Returned value:      pwm:PWM占空比

***************************************************************************/


