/**
  ******************************************************************************
  * @file    pid.c
  * @author  Ğ¡Óã¶ù·ÉÑ¾·É
  * @version V0.0.1
  * @date    2019/7/25
  * @brief   Ù¤ÀûÂÔ»úÆ÷ÈËPIDËã·¨
  ******************************************************************************        
*/

#include "pid.h"
#include "shell.h"
#include "stdio.h"

/*===========º¯Êı==============*/
/*ÂÖ×ÓËÙ¶ÈPIDµ÷½Ú*/
int32_t  Wheel_Integral_Separation_Pid(ST_pid_variable_p pid_variable_p,int32_t rang);
/*×óÂÖËÙ¶ÈPIDµ÷½Ú*/
int32_t  Left_Wheel_Integral_Separation_Pid(int32_t rang);
/*ÓÒÂÖËÙ¶ÈPIDµ÷½Ú*/
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
/*×óÂÖPID*/
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
/*ÓÒÂÖPID*/
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
/*Ğ¡³µ×ßÖ±ÏßPID*/
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

/*PIDº¯Êı½Ó¿Ú½Ó¿Ú*/
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
** Descriptions:        ÂÖ×Óµ÷ËÙ»ı·Ö·ÖÀëPIDËã·¨ÊµÏÖ
** input parameters:     pid_variable_p :PID²ÎÊı½á¹¹Ìå
**                       range :µ÷ËÙ·¶Î§
** Returned value:      pwm:PWMÕ¼¿Õ±È
**pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last)
**ÔÚÆÕÍ¨PID¿ØÖÆÖĞ£¬ÒıÈë»ı·Ö»·½ÚµÄÄ¿µÄ£¬Ö»ÒªÊÇÎªÁËÏû³ı¾²²î£¬Ìá¸ß¿ØÖÆ¾«¶È£¬
**µ«ÊÇÔÚÆô¶¯¡¢½áÊø»ò´ó·ù¶ÈÔö¼õÊ±£¬¶ÌÊ±¼äÄÚÏµÍ³Êä³öÓĞºÜ´óµÄÆ«²î£¬»áÔì³ÉPIDÔËËãµÄ»ı·Ö»ıÀÛ£¬
**µ¼ÖÂ¿ØÖÆÁ¿³¬¹ıÖ´ĞĞ»ú¹¹¿ÉÄÜÔÊĞíµÄ×î´ó¶¯×÷·¶Î§¶ÔÓ¦¼«ÏŞ¿ØÖÆÁ¿£¬´Ó¶øÒıÆğ½»´óµÄ³¬µ÷£¬
**ÉõÖÁÊÇÕğµ´£¬ÕâÊÇ¾ø¶Ô²»ÔÊĞí£¬ÎªÁË¿Ë·şÕâÒ»ÎÊÌâ£¬ÒıÈë»ı·Ö·ÖÀëµÄ¸ÅÄî£¬Æä»ù±¾Ë¼Â·ÊÇ
**µ±±»¿ØÁ¿ÓëÉè¶¨ÖµÆ«²î½»´óÊ±£¬È¡Ïû»ı·Ö×÷ÓÃ£¬
**µ±±»¿ØÁ¿½Ó½ü¸ø¶¨ÖµÊ±£¬ÒıÈë»ı·Ö¿ØÖÆ£¬ÒÔÏû³ı¾²²î£¬Ìá¸ß¾«¶È
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
		/*¼ÆËãµ±Ç°Îó²î*/
	  err = pid_variable_p->current_err = pid_variable_p->to_value 
		                                   - pid_variable_p->current_value;
		
		//--100 1000
		/*»ı·Ö·ÖÀë*/
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
	  pid_variable_p->integral += pid_variable_p->current_err;//--ÀÛ¼Ó»ı·ÖÎó²î
		}
		else{
    P_index = 1; 	
		I_index = 1;
    pid_variable_p->integral += pid_variable_p->current_err;//--ÀÛ¼Ó»ı·ÖÎó²î
    }
    pwm = 
	             (P_index*pid_variable_p->kp*pid_variable_p->current_err
	             +I_index*pid_variable_p->ki*pid_variable_p->integral
	             +pid_variable_p->kd*(pid_variable_p->current_err-pid_variable_p->last_err))/1000;

   // pwm2+=pwm1;	
		rang = range(rang, 0, 2500);/*·¶Î§¿ØÖÆ±¾ÉíÏŞÖÆ*/
		pid_variable_p->output_add+=range(pwm, -rang, rang);/*PWMµ÷ËÙËÙÂÊ¿ØÖÆ*/
		pid_variable_p->output_value =range(pid_variable_p->output_add,pid_variable_p->PWM_umin, pid_variable_p->PWM_umax);//--PWM·¶Î§¿ØÖÆ
    pid_variable_p->last_err = pid_variable_p->current_err;//--¸üĞÂÉÏÒ»´ÎÎó²î
    return pid_variable_p->output_value;
 }
/*×óÂÖËÙ¶ÈPIDµ÷½Ú*/
int32_t  Left_Wheel_Integral_Separation_Pid(int32_t rang)
{
return Wheel_Integral_Separation_Pid(left_pid_variable_p,rang);
}
/*ÓÒÂÖËÙ¶ÈPIDµ÷½Ú*/
int32_t  Right_Wheel_Integral_Separation_Pid(int32_t rang)
{
return Wheel_Integral_Separation_Pid(right_pid_variable_p,rang);
}
/***************************************************************************
** Function name:       Set_Wheel_Pid_Speed
** Descriptions:        ÉèÖÃPIDµÄÄ¿±êÖµ
** input parameters:    pid_variable_p£ºPID²ÎÊı½á¹¹Ìå  to_speed£ºÆÚÍûÖµ
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
** Descriptions:        ¶ÁÈ¡PID¼ÆËã³öÀ´µÄPWMÖµ
** input parameters:    pid_variable_p£ºPID²ÎÊı½á¹¹Ìå
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
** Descriptions:        ÉèÖÃPID²ÎÊı
** input parameters:    pid_variable_p£ºPID²ÎÊı½á¹¹Ìå kp ki kd
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
** Descriptions:       PWMÔËËã¹ı³ÌÓÃµ½µÄÖĞ¼ä±äÁ¿ÇåÁã
** input parameters:    pid_variable_p£ºPID²ÎÊı½á¹¹Ìå
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
** Descriptions:        Ğ¡³µÄ©¶Ë¼õËÙPIDµ÷½Ú
** input parameters:    speed:Ğ¡³µËÙ¶È
                        now_distance£ºµ±Ç°ĞĞÊ»µÄ¾àÀë
**                      to_distance£º½«ÒªĞĞÊ»µÄ¾àÀë
** Returned value:      speed:Ğ¡³µËÙ¶È
**pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last)
**ÔÚÆÕÍ¨PID¿ØÖÆÖĞ£¬ÒıÈë»ı·Ö»·½ÚµÄÄ¿µÄ£¬Ö»ÒªÊÇÎªÁËÏû³ı¾²²î£¬Ìá¸ß¿ØÖÆ¾«¶È£¬
**µ«ÊÇÔÚÆô¶¯¡¢½áÊø»ò´ó·ù¶ÈÔö¼õÊ±£¬¶ÌÊ±¼äÄÚÏµÍ³Êä³öÓĞºÜ´óµÄÆ«²î£¬»áÔì³ÉPIDÔËËãµÄ»ı·Ö»ıÀÛ£¬
**µ¼ÖÂ¿ØÖÆÁ¿³¬¹ıÖ´ĞĞ»ú¹¹¿ÉÄÜÔÊĞíµÄ×î´ó¶¯×÷·¶Î§¶ÔÓ¦¼«ÏŞ¿ØÖÆÁ¿£¬´Ó¶øÒıÆğ½»´óµÄ³¬µ÷£¬
**ÉõÖÁÊÇÕğµ´£¬ÕâÊÇ¾ø¶Ô²»ÔÊĞí£¬ÎªÁË¿Ë·şÕâÒ»ÎÊÌâ£¬ÒıÈë»ı·Ö·ÖÀëµÄ¸ÅÄî£¬Æä»ù±¾Ë¼Â·ÊÇ
**µ±±»¿ØÁ¿ÓëÉè¶¨ÖµÆ«²î½»´óÊ±£¬È¡Ïû»ı·Ö×÷ÓÃ£¬
**µ±±»¿ØÁ¿½Ó½ü¸ø¶¨ÖµÊ±£¬ÒıÈë»ı·Ö¿ØÖÆ£¬ÒÔÏû³ı¾²²î£¬Ìá¸ß¾«¶È
***************************************************************************/
int32_t Car_Walk_Distance_PID(int32_t speed,uint32_t now_distance ,uint32_t to_distance)
{
    int32_t speed1 = 0;
	  int32_t err = 0;

		/*¼ÆËãµ±Ç°Îó²î*/
	  err = to_distance - now_distance;
		if(err>200)
		{	
	    return speed;
		}
		else{//--20cmµÄÊ±ºò¼õËÙ
					
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
** Descriptions:        Ğ¡³µº½ÏòPID¿ØÖÆ
** input parameters:    yaw_err£ºÖ±ÏßĞĞ×ßº½ÏòÆ«²îÖµ
** Returned value:      pwm:PWMÕ¼¿Õ±È
**pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last)
**ÔÚÆÕÍ¨PID¿ØÖÆÖĞ£¬ÒıÈë»ı·Ö»·½ÚµÄÄ¿µÄ£¬Ö»ÒªÊÇÎªÁËÏû³ı¾²²î£¬Ìá¸ß¿ØÖÆ¾«¶È£¬
**µ«ÊÇÔÚÆô¶¯¡¢½áÊø»ò´ó·ù¶ÈÔö¼õÊ±£¬¶ÌÊ±¼äÄÚÏµÍ³Êä³öÓĞºÜ´óµÄÆ«²î£¬»áÔì³ÉPIDÔËËãµÄ»ı·Ö»ıÀÛ£¬
**µ¼ÖÂ¿ØÖÆÁ¿³¬¹ıÖ´ĞĞ»ú¹¹¿ÉÄÜÔÊĞíµÄ×î´ó¶¯×÷·¶Î§¶ÔÓ¦¼«ÏŞ¿ØÖÆÁ¿£¬´Ó¶øÒıÆğ½»´óµÄ³¬µ÷£¬
**ÉõÖÁÊÇÕğµ´£¬ÕâÊÇ¾ø¶Ô²»ÔÊĞí£¬ÎªÁË¿Ë·şÕâÒ»ÎÊÌâ£¬ÒıÈë»ı·Ö·ÖÀëµÄ¸ÅÄî£¬Æä»ù±¾Ë¼Â·ÊÇ
**µ±±»¿ØÁ¿ÓëÉè¶¨ÖµÆ«²î½»´óÊ±£¬È¡Ïû»ı·Ö×÷ÓÃ£¬
**µ±±»¿ØÁ¿½Ó½ü¸ø¶¨ÖµÊ±£¬ÒıÈë»ı·Ö¿ØÖÆ£¬ÒÔÏû³ı¾²²î£¬Ìá¸ß¾«¶È
***************************************************************************/
int32_t Car_Walk_Yaw_Pid(int32_t yaw_err)
{
    int32_t speed = 0;
	  int32_t err = 0;
    int32_t P_index = 0;
    int32_t I_index = 0;
		/*¼ÆËãµ±Ç°Îó²î*/

	 /*Èç¹ûÆ«²î´óÓÚ180£¬´Ó·´·½Ïò»ØÕı*/
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
	   car_walk_yaw_pid_p->integral +=yaw_err;//--ÀÛ¼Ó»ı·ÖÎó²î
		}
		else{
     P_index = 1; 	
		 //I_index = 1;
     car_walk_yaw_pid_p->integral += yaw_err;//--ÀÛ¼Ó»ı·ÖÎó²î
    }
    speed = 
	             (P_index*car_walk_yaw_pid_p->kp*yaw_err
	             +I_index*car_walk_yaw_pid_p->ki*car_walk_yaw_pid_p->integral
	             +car_walk_yaw_pid_p->kd*(err-car_walk_yaw_pid_p->last_err))/100000;

		car_walk_yaw_pid_p->output_add =speed;
		car_walk_yaw_pid_p->output_value =range(speed,-500, 500);//--PWM·¶Î§¿ØÖÆ
    car_walk_yaw_pid_p->last_err = yaw_err;//--¸üĞÂÉÏÒ»´ÎÎó²î

		return car_walk_yaw_pid_p->output_value;
	}
/***************************************************************************
** Function name:       Car_Integral_Separation_Pid
** Descriptions:        Ğ¡³µ×ßÖ±Ïß»ı·Ö·ÖÀëPIDËã·¨ÊµÏÖ
** input parameters:    motor_switc£ºµç»úÑ¡Ôñ£¬¿ÉÒÔÈ¡Ò»ÏÂÖµ£º
**                                  LEFT_MOTOR £º ×ó²àµç»ú
**                                  RIGHT_MOTOR£º ÓÒ²àµç»ú
**                                  MOWING_MOTOR£º¸î²İµç»ú
**                      to_speed : Ä¿±êËÙ¶È
** Returned value:      pwm:PWMÕ¼¿Õ±È
**pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last)
**ÔÚÆÕÍ¨PID¿ØÖÆÖĞ£¬ÒıÈë»ı·Ö»·½ÚµÄÄ¿µÄ£¬Ö»ÒªÊÇÎªÁËÏû³ı¾²²î£¬Ìá¸ß¿ØÖÆ¾«¶È£¬
**µ«ÊÇÔÚÆô¶¯¡¢½áÊø»ò´ó·ù¶ÈÔö¼õÊ±£¬¶ÌÊ±¼äÄÚÏµÍ³Êä³öÓĞºÜ´óµÄÆ«²î£¬»áÔì³ÉPIDÔËËãµÄ»ı·Ö»ıÀÛ£¬
**µ¼ÖÂ¿ØÖÆÁ¿³¬¹ıÖ´ĞĞ»ú¹¹¿ÉÄÜÔÊĞíµÄ×î´ó¶¯×÷·¶Î§¶ÔÓ¦¼«ÏŞ¿ØÖÆÁ¿£¬´Ó¶øÒıÆğ½»´óµÄ³¬µ÷£¬
**ÉõÖÁÊÇÕğµ´£¬ÕâÊÇ¾ø¶Ô²»ÔÊĞí£¬ÎªÁË¿Ë·şÕâÒ»ÎÊÌâ£¬ÒıÈë»ı·Ö·ÖÀëµÄ¸ÅÄî£¬Æä»ù±¾Ë¼Â·ÊÇ
**µ±±»¿ØÁ¿ÓëÉè¶¨ÖµÆ«²î½»´óÊ±£¬È¡Ïû»ı·Ö×÷ÓÃ£¬
**µ±±»¿ØÁ¿½Ó½ü¸ø¶¨ÖµÊ±£¬ÒıÈë»ı·Ö¿ØÖÆ£¬ÒÔÏû³ı¾²²î£¬Ìá¸ß¾«¶È
***************************************************************************/
//uint16_t  Car_Integral_Separation_Pid(int32_t yaw_err){
//	 

//	 int32_t index = 0;
//   int32_t out_value1;
//	 int32_t out_value2;
//	 int32_t err = pid_variable_p->current_value = yaw_err;//--µ±Ç°Îó²î
//	  if(err <0)err = -err; 
//	  if(err>200)//--»ı·Ö·ÖÀë
//    {
//			index = 0;
//    }
//		else if (err<100)
//		{
//		index = 2;
//	  pid_variable_p->integral += pid_variable_p->current_err;//--ÀÛ¼Ó»ı·ÖÎó²î
//		}
//		else{
//		index = 1;
//    pid_variable_p->integral += pid_variable_p->current_err;//--ÀÛ¼Ó»ı·ÖÎó²î
//    }
//    out_value1 = 
//	             (pid_variable_p->kp*pid_variable_p->current_err
//	             +index*pid_variable_p->ki*pid_variable_p->integral
//	             +pid_variable_p->kd*(pid_variable_p->current_err-pid_variable_p->last_err))/1000;
//     out_value2+=out_value1;		
//		//pwm2+=range(pwm1, -rang1, rang1);//--PWM·¶Î§¿ØÖÆ
//		pid_variable_p->output_value =range(out_value2,pid_variable_p->umin, pid_variable_p->umax);//--PWM·¶Î§¿ØÖÆ
//    pid_variable_p->last_err = pid_variable_p->current_err;//--¸üĞÂÉÏÒ»´ÎÎó²î
//    return pid_variable_p->output_value;
//}
/***************************************************************************
** Function name:       Position_Pid
** Descriptions:        Î»ÖÃĞÍPIDµÄCÓïÑÔÊµÏÖ
** input parameters:    motor_switc£ºµç»úÑ¡Ôñ£¬¿ÉÒÔÈ¡Ò»ÏÂÖµ£º
**                                  LEFT_MOTOR £º ×ó²àµç»ú
**                                  RIGHT_MOTOR£º ÓÒ²àµç»ú
**                                  MOWING_MOTOR£º¸î²İµç»ú
**                      to_speed : Ä¿±êËÙ¶È
** Returned value:      pwm:PWMÕ¼¿Õ±È
**pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last)
**Ã»ÓĞ¿¼ÂÇËÀÇø£¬Ã»ÓĞÉè¶¨ÉÏÏÂÏŞ£¬Ö»ÊÇ¶Ô¹«Ê½µÄÒ»ÖÖ×îÖ±½ÓµÄÊµÏÖ
***************************************************************************/
//int32_t  Position_Pid(uint8_t motor_switch,uint16_t to_speed){
//	  if(motor_switch == RIGHT_MOTOR)pid_variable_p = right_pid_variable_p;
//	  else if(motor_switch == LEFT_MOTOR) pid_variable_p = left_pid_variable_p;
//	  int32_t pwm;
//    pid_variable_p->to_value = to_speed;//--ÉèÖÃÄ¿±êËÙ¶È	//--·Å´ó100±¶  
//	  pid_variable_p->current_value = speed_function_interface_handle_p->read_speed(motor_switch);//--µ±Ç°ËÙ¶È
//	  pid_variable_p->current_value = pid_variable_p->to_value - pid_variable_p->current_value;//--µ±Ç°Îó²î
//	  //printf("current_err %d\r\n",pid_variable_p->current_value);
//	  pid_variable_p->integral += pid_variable_p->current_value;//--ÀÛ¼Ó»ı·ÖÎó²î
//    pwm += 
//	              (pid_variable_p->kp*pid_variable_p->current_err
//	             +pid_variable_p->ki*pid_variable_p->integral
//	             +pid_variable_p->kd*(pid_variable_p->current_err-pid_variable_p->last_err))/1000;
//    pid_variable_p->output_value =range(pwm,pid_variable_p->umin, pid_variable_p->umax);//--PWM·¶Î§¿ØÖÆ
//	  pid_variable_p->last_err = pid_variable_p->current_err;//--¸üĞÂÉÏÒ»´ÎÎó²î
//    return pid_variable_p->output_value;
//}
/***************************************************************************
** Function name:       Incremental_Pid
** Descriptions:        ÔöÁ¿ĞÍPIDËã·¨µÄCÓïÑÔÊµÏÖ
** input parameters:    motor_switc£ºµç»úÑ¡Ôñ£¬¿ÉÒÔÈ¡Ò»ÏÂÖµ£º
**                                  LEFT_MOTOR £º ×ó²àµç»ú
**                                  RIGHT_MOTOR£º ÓÒ²àµç»ú
**                                  MOWING_MOTOR£º¸î²İµç»ú
**                      to_speed : Ä¿±êËÙ¶È
** Returned value:      pwm:PWMÕ¼¿Õ±È
**pid.Kp*(pid.err-pid.err_next)+pid.Ki*pid.err+pid.Kd*(pid.err-2*pid.err_next+pid.err_last)
**Ã»ÓĞ¿¼ÂÇËÀÇø£¬Ã»ÓĞÉè¶¨ÉÏÏÂÏŞ£¬Ö»ÊÇ¶Ô¹«Ê½µÄÒ»ÖÖ×îÖ±½ÓµÄÊµÏÖ
***************************************************************************/
//int32_t Incremental_Pid(uint8_t motor_switch,uint16_t to_speed){

//	  if(motor_switch == RIGHT_MOTOR)pid_variable_p = right_pid_variable_p;
//	  else if(motor_switch == LEFT_MOTOR) pid_variable_p = left_pid_variable_p;
//	  int32_t pwm;

//    pid_variable_p->to_value = to_speed;//--ÉèÖÃÄ¿±êËÙ¶È
//	  pid_variable_p->current_value = speed_function_interface_handle_p->read_speed(motor_switch);//--µ±Ç°ËÙ¶È
//    pid_variable_p->current_err = pid_variable_p->to_value - pid_variable_p->current_value;//--µ±Ç°Îó²î
//   // printf("current_err %d\r\n",pid_variable_p->current_err);  
//	  pwm += 
//	            (pid_variable_p->kp*(pid_variable_p->current_err-pid_variable_p->last_err)
//	           +pid_variable_p->ki*pid_variable_p->current_err
//	           +pid_variable_p->kd*(pid_variable_p->current_err-2*pid_variable_p->last_err
//	           +pid_variable_p->last_last_err))/1000;
//    pid_variable_p->output_value =range(pwm,pid_variable_p->umin, pid_variable_p->umax);//--PWM·¶Î§¿ØÖÆ
//    pid_variable_p->last_last_err=pid_variable_p->last_err;
//    pid_variable_p->last_err=pid_variable_p->current_err;
//    return pid_variable_p->output_value;
//}
/***************************************************************************
** Function name:       Anti_Integral_Saturation_Pid
** Descriptions:        ¿¹»ı·Ö±¥ºÍPIDËã·¨µÄCÓïÑÔÊµÏÖ
** input parameters:    motor_switc£ºµç»úÑ¡Ôñ£¬¿ÉÒÔÈ¡Ò»ÏÂÖµ£º
**                                  LEFT_MOTOR £º ×ó²àµç»ú
**                                  RIGHT_MOTOR£º ÓÒ²àµç»ú
**                                  MOWING_MOTOR£º¸î²İµç»ú
**                      to_speed : Ä¿±êËÙ¶È
** Returned value:      pwm:PWMÕ¼¿Õ±È
**pid.Kp*(pid.err-pid.err_next)+pid.Ki*pid.err+pid.Kd*(pid.err-2*pid.err_next+pid.err_last)
**ËùÎ½»ı·Ö±¥ºÍÏÖÏóÊÇÖ¸ÏµÍ³´æÔÚÒ»¸ö·½ÏòµÄÆ«²î£¬PID¿ØÖÆÆ÷µÄÊä³öÓÉÓÚ»ı·Ö×÷ÓÃµÄ²»¶ÏÀÛ¼Ó¶ø¼Ó´ó£¬
**´Ó¶øµ¼ÖÂÖ´ĞĞ»ú¹¹´ïµ½¼«ÏŞÎ»Î»ÖÃ£¬Èô¿ØÖÆÆ÷Êä³ö¼ÌĞøÔö´ó£¬Ö´ĞĞÆ÷¿ª¶È²»¿ÉÄÜÔÙÔö´ó£¬
**´ËÊ±¼ÆËã»úÊä³ö¿ØÖÆÁ¿³¬¹ıÁËÕı³£ÔËĞĞ·¶Î§¶ø½øÈë±¥ºÍÇø¡£Ò»µ©ÏµÍ³³öÏÖ·´ÏòÆ«²î£¬
**¿ØÖÆÆ÷Êä³öÖğ½¥´Ó±¥ºÍÇøÍË³ö£¬½øÈë±¥ºÍÇøÔ½ÉîÔòÍË³ö±¥ºÍÇøÊ±¼äÔ½³¤¡£ÔÚÕâ¶ÎÊ±¼ä£¬
**Ö´ĞĞ»ú¹¹ÈÔÈ»Í£ÁôÔÚ¼«ÏŞÎ»ÖÃ¶ø²»ËæÆ«²î·´Ïò¶øÁ¢¼´×ö³öÏàÓ¦µÄ¸Ä±ä£¬ÕâÊ±ÏµÍ³ÏñÊ±¿ÕÒ»Ñù£¬
**Ôì³É¿ØÖÆĞÔÄÜ¶ñ»¯£¬ÕâÖÖÏÖÏó³ÉÎª»ı·Ö±¥ºÍÏÖÏó»ò»ı·ÖÊ§¿Ø
**·ÀÖ¹»ı·Ö±¥ºÍµÄ·½·¨Ö®Ò»¾ÍÊÇ¿¹±¥ºÍ·¨£¬¸Ã·½·¨µÄË¼ÏëÊÇÔÚ¼ÆËã¿ØÖÆÆ÷Êä³öµÄÊ±ºò£¬Ê×ÏÈÅĞ¶Ï
**ÉÏÒ»Ê±¿ÌµÄ¿ØÖÆÁ¿ÊÇ·ñÒÑ¾­³¬³öÁË¼«ÏŞ·¶Î§£¬Èç¹ûÉÏÒ»Ê±¿ÌÊä³ö´óÓÚ¼«ÏŞ·¶Î§£¬ÔòÖ»ÀÛ¼ÓÕıÆ«²î
**´Ó¶ø±ÜÃâ¿ØÖÆÁ¿³¤ÊÇÅ¶¼ûÍ£ÁôÔÚ±¥ºÍÇø
***************************************************************************/
//int32_t  Anti_Integral_Saturation_Pid(uint8_t motor_switch,uint16_t to_speed){

//	 if(motor_switch == RIGHT_MOTOR)pid_variable_p = right_pid_variable_p;
//	  else if(motor_switch == LEFT_MOTOR) pid_variable_p = left_pid_variable_p; 
//  	pid_variable_p->kp = 0;
//	  pid_variable_p->ki = 0;
//	  pid_variable_p->kd = 0;
//    pid_variable_p->to_value = to_speed;//--ÉèÖÃÄ¿±êËÙ¶È
//	  pid_variable_p->current_value = speed_function_interface_handle_p->read_speed(motor_switch);//--µ±Ç°ËÙ¶ÈÈ
//	  pid_variable_p->current_err = pid_variable_p->to_value - pid_variable_p->current_value;//--µ±Ç°Îó²î
//	  if(pid_variable_p->current_value>pid_variable_p->umax)//--¿¹±¥ºÍ//--ÕâÀïµÄËÙ¶ÈÓ¦¸Ã»»ÎªPWM£¬ÓÃµĞèÒª¸Ä½
//		  {
//		   if(pid_variable_p->current_err>200)//--»ı·Ö·ÖÀë
//         {
//          pid_variable_p->ki = 0;
//         }else{
//          pid_variable_p->ki = 1;;
//         pid_variable_p->integral += pid_variable_p->current_err;//--ÀÛ¼Ó»ı·ÖÎó²î
//         }
//		 }
//	 if(pid_variable_p->current_value<pid_variable_p->umin)//--¿¹±¥ºÍ
//		  {
//		   if(pid_variable_p->current_err>200)//--»ı·Ö·ÖÀë
//         {
//          pid_variable_p->ki = 0;
//         }else{
//          pid_variable_p->ki = 1;;
//         pid_variable_p->integral += pid_variable_p->current_err;//--ÀÛ¼Ó»ı·ÖÎó²î
//         }
//		 }
//	if(pid_variable_p->current_err>200)//--»ı·Ö·ÖÀë
//    {
//   pid_variable_p->ki = 0;
//    }else{
//   pid_variable_p->ki = 1;;
//   pid_variable_p->integral += pid_variable_p->current_err;//--ÀÛ¼Ó»ı·ÖÎó²î
//    }
//    pid_variable_p->output_value = 
//	              pid_variable_p->kp*pid_variable_p->current_err
//	             +pid_variable_p->ki*pid_variable_p->integral
//	             +pid_variable_p->kd*(pid_variable_p->current_err-pid_variable_p->last_err);
//    pid_variable_p->last_err = pid_variable_p->current_err;//--¸üĞÂÉÏÒ»´ÎÎó²î
//    return pid_variable_p->output_value;
//}

/***************************************************************************
** Function name:       Trapezoidal_Integraln_Pid
** Descriptions:        ÌİĞÎ»ı·ÖµÄPIDËã·¨ÊµÏÖ
** input parameters:    motor_switc£ºµç»úÑ¡Ôñ£¬¿ÉÒÔÈ¡Ò»ÏÂÖµ£º
**                                  LEFT_MOTOR £º ×ó²àµç»ú
**                                  RIGHT_MOTOR£º ÓÒ²àµç»ú
**                                  MOWING_MOTOR£º¸î²İµç»ú
**                      to_speed : Ä¿±êËÙ¶È
** Returned value:      pwm:PWMÕ¼¿Õ±È
**pid.Kp*pid.err+index*pid.Ki*pid.integral/2+pid.Kd*(pid.err-pid.err_last);  //--ÌİĞÎ»ı·Ö
**×÷ÎªPID¿ØÖÆµÄ»ı·ÖÏî£¬Æä×÷ÓÃÊÇÏû³ıÓà²î£¬Ó¦Ìá¸ß»ı·ÖÏîÔËËã¾«¶È£¬Îª´Ë¿ÉÒÔ½«¾ØĞÎ»ı·Ö¸ÄÎªÌİĞÎ»ı·Ö£¬
***************************************************************************/
//int32_t  Trapezoidal_Integraln_Pid(uint8_t motor_switch,uint16_t to_speed){

//	  if(motor_switch == RIGHT_MOTOR)pid_variable_p = right_pid_variable_p;
//	  else if(motor_switch == LEFT_MOTOR) pid_variable_p = left_pid_variable_p; 
//	 pid_variable_p->kp = 0;
//	  pid_variable_p->ki = 0;
//	  pid_variable_p->kd = 0;
//    pid_variable_p->to_value = to_speed;//--ÉèÖÃÄ¿±êËÙ¶È
//	  pid_variable_p->current_value = speed_function_interface_handle_p->read_speed(motor_switch);//--µ±Ç°ËÙ¶ÈÈ
//	  pid_variable_p->current_err = pid_variable_p->to_value - pid_variable_p->current_value;//--µ±Ç°Îó²î
//	  if(pid_variable_p->current_value>pid_variable_p->umax)//--¿¹±¥ºÍ//--ÕâÀïµÄËÙ¶ÈÓ¦¸Ã»»ÎªPWM£¬ÓÃµĞèÒª¸Ä½
//		  {
//		   if(pid_variable_p->current_err>200)//--»ı·Ö·ÖÀë
//         {
//          pid_variable_p->ki = 0;
//         }else{
//          pid_variable_p->ki = 1;;
//         pid_variable_p->integral += pid_variable_p->current_err;//--ÀÛ¼Ó»ı·ÖÎó²î
//         }
//		 }
//	 if(pid_variable_p->current_value<pid_variable_p->umin)//--¿¹±¥ºÍ
//		  {
//		   if(pid_variable_p->current_err>200)//--»ı·Ö·ÖÀë
//         {
//          pid_variable_p->ki = 0;
//         }else{
//          pid_variable_p->ki = 1;;
//         pid_variable_p->integral += pid_variable_p->current_err;//--ÀÛ¼Ó»ı·ÖÎó²î
//         }
//		 }
//	if(pid_variable_p->current_err>200)//--»ı·Ö·ÖÀë
//    {
//   pid_variable_p->ki = 0;
//    }else{
//   pid_variable_p->ki = 1;;
//   pid_variable_p->integral += pid_variable_p->current_err;//--ÀÛ¼Ó»ı·ÖÎó²î
//    }
//    pid_variable_p->output_value = 
//	              pid_variable_p->kp*pid_variable_p->current_err
//	             +pid_variable_p->ki*pid_variable_p->integral/2
//	             +pid_variable_p->kd*(pid_variable_p->current_err-pid_variable_p->last_err);
//    pid_variable_p->last_err = pid_variable_p->current_err;//--¸üĞÂÉÏÒ»´ÎÎó²î
//    return pid_variable_p->output_value;
//}
/***************************************************************************
** Function name:       Variable_Integraln_Pid
** Descriptions:        ±ä»ı·ÖµÄPIDËã·¨ÊµÏÖ
** input parameters:    motor_switc£ºµç»úÑ¡Ôñ£¬¿ÉÒÔÈ¡Ò»ÏÂÖµ£º
**                                  LEFT_MOTOR £º ×ó²àµç»ú
**                                  RIGHT_MOTOR£º ÓÒ²àµç»ú
**                                  MOWING_MOTOR£º¸î²İµç»ú
**                      to_speed : Ä¿±êËÙ¶È
** Returned value:      pwm:PWMÕ¼¿Õ±È
**pid.Kp*pid.err+index*pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);  //--ÌİĞÎ»ı·Ö
**±ä»ı·ÖPID¿ÉÒÔ¿´³ÉÊÇ»ı·Ö·ÖÀëµÄPIDËã·¨µÄ¸üÒ»°ãµÄĞÎÊ½£¡ÔÚÆÕÍ¨µÄPIDËã·¨ÖĞ£¬ÓÉÓÚ»ı·ÖÏµÊıKIÊÇ³£Êı
**ËùÒÔÔÚÕû¸ö¿ØÖÆ¹ı³ÌÖĞ£¬»ı·ÖÔöÁ¿Ê½²»±äµÄ£¬µ«ÊÇ£¬ÏµÍ³¶ÔÓÚ»ı·ÖÏîµÄÒªÇóÊÇ£¬ÏµÍ³Æ«²î´óÊ±£¬»ı·Ö×÷ÓÃÓ¦¸Ã¼õÈõÉõÖÁÈ«ÎŞ
**£¬¶øÔÚÆ«²îĞ¡Ê±£¬ÔòÓ¦¸Ã¼ÓÇ¿¡£»ı·ÖÏµÊıÈ¡´óÁË»á²úÉú³¬µ÷£¬ÉõÖÁ»ı·Ö±¥ºÍ£¬È¡Ğ¡ÁËÓÖ²»ÄÜ¶ÌÊ±¼äÏû³ı¾²²î£¬Òò´Ë£¬
**¸ù¾İÏµÍ³µÄÆ«²î´óĞ¡¸Ä±ä»ı·ÖËÙ¶ÈÊÇÓĞ±ØÒªµÄ¡£
**±ä»ı·ÖPIDµÄ»ù±¾Ë¼ÏëÊÇÉè·¨¸Ä±ä»ı·ÖÏîµÄÀÛ¼ÓËÙ¶È£¬Ê¹ÆäÓëÆ«²î´óĞ¡ÏàÓ¦£»Æ«²îÔ½´ó£¬»ı·ÖÔ½Âı£¬Æ«²îÔ½Ğ¡£¬»ı·ÖÔ½¿ì
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
//	  pid_variable_p->current_value = speed_function_interface_handle_p->read_speed(motor_switch);//--µ±Ç°ËÙ¶ÈÈ
//	  pid_variable_p->current_err = pid_variable_p->to_value - pid_variable_p->current_value;//--µ±Ç°Îó²î
//	
//	if(pid_variable_p->current_err>200)//--»ı·Ö·ÖÀë
//    {
//    index = 0;
//	  rang1 = 400;
//    }
//	else if(pid_variable_p->current_err<50)
//		{
//   index = 2;
//   pid_variable_p->integral += pid_variable_p->current_err;//--ÀÛ¼Ó»ı·ÖÎó²î
//    }
//	else
//	{
//	index = (200-pid_variable_p->current_err)/20;
//	index = 1;
//	pid_variable_p->integral += pid_variable_p->current_err;//--ÀÛ¼Ó»ı·ÖÎó²î
//	}
//    pwm1 = 
//	             ( pid_variable_p->kp*pid_variable_p->current_err
//	             +index*pid_variable_p->ki*pid_variable_p->integral
//	             +pid_variable_p->kd*(pid_variable_p->current_err-pid_variable_p->last_err))/1000;
//	  pwm2+=range(pwm1, -rang1, rang1);//--PWM·¶Î§¿ØÖÆ
//	  pid_variable_p->output_value =range(pwm2, pid_variable_p->umin, pid_variable_p->umax);//--PWM·¶Î§¿ØÖÆ
//    pid_variable_p->last_err = pid_variable_p->current_err;//--¸üĞÂÉÏÒ»´ÎÎó²î
//    return pid_variable_p->output_value;
//}
/***************************************************************************
** Function name:       
** Descriptions:        ×¨¼ÒPIDÓëÄ£ºıPIDµÄCÓïÑÔËã·¨ÊµÏÖ
** input parameters:    motor_switc£ºµç»úÑ¡Ôñ£¬¿ÉÒÔÈ¡Ò»ÏÂÖµ£º
**                                  LEFT_MOTOR £º ×ó²àµç»ú
**                                  RIGHT_MOTOR£º ÓÒ²àµç»ú
**                                  MOWING_MOTOR£º¸î²İµç»ú
**                      to_speed : Ä¿±êËÙ¶È
** Returned value:      pwm:PWMÕ¼¿Õ±È

***************************************************************************/


