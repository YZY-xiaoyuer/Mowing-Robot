/**
  ******************************************************************************
  * @file    pid.c
  * @author  С�����Ѿ��
  * @version V0.0.1
  * @date    2019/7/25
  * @brief   ٤���Ի�����PID�㷨
  ******************************************************************************        
*/

#include "pid.h"
#include "shell.h"
#include "stdio.h"

/*===========����==============*/
/*�����ٶ�PID����*/
int32_t  Wheel_Integral_Separation_Pid(ST_pid_variable_p pid_variable_p,int32_t rang);
/*�����ٶ�PID����*/
int32_t  Left_Wheel_Integral_Separation_Pid(int32_t rang);
/*�����ٶ�PID����*/
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
/*����PID*/
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
/*����PID*/
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
/*С����ֱ��PID*/
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

/*PID�����ӿڽӿ�*/
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
** Descriptions:        ���ӵ��ٻ��ַ���PID�㷨ʵ��
** input parameters:     pid_variable_p :PID�����ṹ��
**                       range :���ٷ�Χ
** Returned value:      pwm:PWMռ�ձ�
**pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last)
**����ͨPID�����У�������ֻ��ڵ�Ŀ�ģ�ֻҪ��Ϊ�����������߿��ƾ��ȣ�
**��������������������������ʱ����ʱ����ϵͳ����кܴ��ƫ������PID����Ļ��ֻ��ۣ�
**���¿���������ִ�л�������������������Χ��Ӧ���޿��������Ӷ����𽻴�ĳ�����
**�������𵴣����Ǿ��Բ�����Ϊ�˿˷���һ���⣬������ַ���ĸ�������˼·��
**�����������趨ֵƫ���ʱ��ȡ���������ã�
**���������ӽ�����ֵʱ��������ֿ��ƣ������������߾���
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
		/*���㵱ǰ���*/
	  err = pid_variable_p->current_err = pid_variable_p->to_value 
		                                   - pid_variable_p->current_value;
		
		//--100 1000
		/*���ַ���*/
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
	  pid_variable_p->integral += pid_variable_p->current_err;//--�ۼӻ������
		}
		else{
    P_index = 1; 	
		I_index = 1;
    pid_variable_p->integral += pid_variable_p->current_err;//--�ۼӻ������
    }
    pwm = 
	             (P_index*pid_variable_p->kp*pid_variable_p->current_err
	             +I_index*pid_variable_p->ki*pid_variable_p->integral
	             +pid_variable_p->kd*(pid_variable_p->current_err-pid_variable_p->last_err))/1000;

   // pwm2+=pwm1;	
		rang = range(rang, 0, 2500);/*��Χ���Ʊ�������*/
		pid_variable_p->output_add+=range(pwm, -rang, rang);/*PWM�������ʿ���*/
		pid_variable_p->output_value =range(pid_variable_p->output_add,pid_variable_p->PWM_umin, pid_variable_p->PWM_umax);//--PWM��Χ����
    pid_variable_p->last_err = pid_variable_p->current_err;//--������һ�����
    return pid_variable_p->output_value;
 }
/*�����ٶ�PID����*/
int32_t  Left_Wheel_Integral_Separation_Pid(int32_t rang)
{
return Wheel_Integral_Separation_Pid(left_pid_variable_p,rang);
}
/*�����ٶ�PID����*/
int32_t  Right_Wheel_Integral_Separation_Pid(int32_t rang)
{
return Wheel_Integral_Separation_Pid(right_pid_variable_p,rang);
}
/***************************************************************************
** Function name:       Set_Wheel_Pid_Speed
** Descriptions:        ����PID��Ŀ��ֵ
** input parameters:    pid_variable_p��PID�����ṹ��  to_speed������ֵ
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
** Descriptions:        ��ȡPID���������PWMֵ
** input parameters:    pid_variable_p��PID�����ṹ��
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
** Descriptions:        ����PID����
** input parameters:    pid_variable_p��PID�����ṹ�� kp ki kd
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
** Descriptions:       PWM��������õ����м��������
** input parameters:    pid_variable_p��PID�����ṹ��
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
** Descriptions:        С��ĩ�˼���PID����
** input parameters:    speed:С���ٶ�
                        now_distance����ǰ��ʻ�ľ���
**                      to_distance����Ҫ��ʻ�ľ���
** Returned value:      speed:С���ٶ�
**pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last)
**����ͨPID�����У�������ֻ��ڵ�Ŀ�ģ�ֻҪ��Ϊ�����������߿��ƾ��ȣ�
**��������������������������ʱ����ʱ����ϵͳ����кܴ��ƫ������PID����Ļ��ֻ��ۣ�
**���¿���������ִ�л�������������������Χ��Ӧ���޿��������Ӷ����𽻴�ĳ�����
**�������𵴣����Ǿ��Բ�����Ϊ�˿˷���һ���⣬������ַ���ĸ�������˼·��
**�����������趨ֵƫ���ʱ��ȡ���������ã�
**���������ӽ�����ֵʱ��������ֿ��ƣ������������߾���
***************************************************************************/
int32_t Car_Walk_Distance_PID(int32_t speed,uint32_t now_distance ,uint32_t to_distance)
{
    int32_t speed1 = 0;
	  int32_t err = 0;

		/*���㵱ǰ���*/
	  err = to_distance - now_distance;
		if(err>200)
		{	
	    return speed;
		}
		else{//--20cm��ʱ�����
					
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
** Descriptions:        С������PID����
** input parameters:    yaw_err��ֱ�����ߺ���ƫ��ֵ
** Returned value:      pwm:PWMռ�ձ�
**pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last)
**����ͨPID�����У�������ֻ��ڵ�Ŀ�ģ�ֻҪ��Ϊ�����������߿��ƾ��ȣ�
**��������������������������ʱ����ʱ����ϵͳ����кܴ��ƫ������PID����Ļ��ֻ��ۣ�
**���¿���������ִ�л�������������������Χ��Ӧ���޿��������Ӷ����𽻴�ĳ�����
**�������𵴣����Ǿ��Բ�����Ϊ�˿˷���һ���⣬������ַ���ĸ�������˼·��
**�����������趨ֵƫ���ʱ��ȡ���������ã�
**���������ӽ�����ֵʱ��������ֿ��ƣ������������߾���
***************************************************************************/
int32_t Car_Walk_Yaw_Pid(int32_t yaw_err)
{
    int32_t speed = 0;
	  int32_t err = 0;
    int32_t P_index = 0;
    int32_t I_index = 0;
		/*���㵱ǰ���*/

	 /*���ƫ�����180���ӷ��������*/
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
	   car_walk_yaw_pid_p->integral +=yaw_err;//--�ۼӻ������
		}
		else{
     P_index = 1; 	
		 //I_index = 1;
     car_walk_yaw_pid_p->integral += yaw_err;//--�ۼӻ������
    }
    speed = 
	             (P_index*car_walk_yaw_pid_p->kp*yaw_err
	             +I_index*car_walk_yaw_pid_p->ki*car_walk_yaw_pid_p->integral
	             +car_walk_yaw_pid_p->kd*(err-car_walk_yaw_pid_p->last_err))/100000;

		car_walk_yaw_pid_p->output_add =speed;
		car_walk_yaw_pid_p->output_value =range(speed,-500, 500);//--PWM��Χ����
    car_walk_yaw_pid_p->last_err = yaw_err;//--������һ�����

		return car_walk_yaw_pid_p->output_value;
	}
/***************************************************************************
** Function name:       Car_Integral_Separation_Pid
** Descriptions:        С����ֱ�߻��ַ���PID�㷨ʵ��
** input parameters:    motor_switc�����ѡ�񣬿���ȡһ��ֵ��
**                                  LEFT_MOTOR �� �����
**                                  RIGHT_MOTOR�� �Ҳ���
**                                  MOWING_MOTOR����ݵ��
**                      to_speed : Ŀ���ٶ�
** Returned value:      pwm:PWMռ�ձ�
**pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last)
**����ͨPID�����У�������ֻ��ڵ�Ŀ�ģ�ֻҪ��Ϊ�����������߿��ƾ��ȣ�
**��������������������������ʱ����ʱ����ϵͳ����кܴ��ƫ������PID����Ļ��ֻ��ۣ�
**���¿���������ִ�л�������������������Χ��Ӧ���޿��������Ӷ����𽻴�ĳ�����
**�������𵴣����Ǿ��Բ�����Ϊ�˿˷���һ���⣬������ַ���ĸ�������˼·��
**�����������趨ֵƫ���ʱ��ȡ���������ã�
**���������ӽ�����ֵʱ��������ֿ��ƣ������������߾���
***************************************************************************/
//uint16_t  Car_Integral_Separation_Pid(int32_t yaw_err){
//	 

//	 int32_t index = 0;
//   int32_t out_value1;
//	 int32_t out_value2;
//	 int32_t err = pid_variable_p->current_value = yaw_err;//--��ǰ���
//	  if(err <0)err = -err; 
//	  if(err>200)//--���ַ���
//    {
//			index = 0;
//    }
//		else if (err<100)
//		{
//		index = 2;
//	  pid_variable_p->integral += pid_variable_p->current_err;//--�ۼӻ������
//		}
//		else{
//		index = 1;
//    pid_variable_p->integral += pid_variable_p->current_err;//--�ۼӻ������
//    }
//    out_value1 = 
//	             (pid_variable_p->kp*pid_variable_p->current_err
//	             +index*pid_variable_p->ki*pid_variable_p->integral
//	             +pid_variable_p->kd*(pid_variable_p->current_err-pid_variable_p->last_err))/1000;
//     out_value2+=out_value1;		
//		//pwm2+=range(pwm1, -rang1, rang1);//--PWM��Χ����
//		pid_variable_p->output_value =range(out_value2,pid_variable_p->umin, pid_variable_p->umax);//--PWM��Χ����
//    pid_variable_p->last_err = pid_variable_p->current_err;//--������һ�����
//    return pid_variable_p->output_value;
//}
/***************************************************************************
** Function name:       Position_Pid
** Descriptions:        λ����PID��C����ʵ��
** input parameters:    motor_switc�����ѡ�񣬿���ȡһ��ֵ��
**                                  LEFT_MOTOR �� �����
**                                  RIGHT_MOTOR�� �Ҳ���
**                                  MOWING_MOTOR����ݵ��
**                      to_speed : Ŀ���ٶ�
** Returned value:      pwm:PWMռ�ձ�
**pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last)
**û�п���������û���趨�����ޣ�ֻ�ǶԹ�ʽ��һ����ֱ�ӵ�ʵ��
***************************************************************************/
//int32_t  Position_Pid(uint8_t motor_switch,uint16_t to_speed){
//	  if(motor_switch == RIGHT_MOTOR)pid_variable_p = right_pid_variable_p;
//	  else if(motor_switch == LEFT_MOTOR) pid_variable_p = left_pid_variable_p;
//	  int32_t pwm;
//    pid_variable_p->to_value = to_speed;//--����Ŀ���ٶ�	//--�Ŵ�100��  
//	  pid_variable_p->current_value = speed_function_interface_handle_p->read_speed(motor_switch);//--��ǰ�ٶ�
//	  pid_variable_p->current_value = pid_variable_p->to_value - pid_variable_p->current_value;//--��ǰ���
//	  //printf("current_err %d\r\n",pid_variable_p->current_value);
//	  pid_variable_p->integral += pid_variable_p->current_value;//--�ۼӻ������
//    pwm += 
//	              (pid_variable_p->kp*pid_variable_p->current_err
//	             +pid_variable_p->ki*pid_variable_p->integral
//	             +pid_variable_p->kd*(pid_variable_p->current_err-pid_variable_p->last_err))/1000;
//    pid_variable_p->output_value =range(pwm,pid_variable_p->umin, pid_variable_p->umax);//--PWM��Χ����
//	  pid_variable_p->last_err = pid_variable_p->current_err;//--������һ�����
//    return pid_variable_p->output_value;
//}
/***************************************************************************
** Function name:       Incremental_Pid
** Descriptions:        ������PID�㷨��C����ʵ��
** input parameters:    motor_switc�����ѡ�񣬿���ȡһ��ֵ��
**                                  LEFT_MOTOR �� �����
**                                  RIGHT_MOTOR�� �Ҳ���
**                                  MOWING_MOTOR����ݵ��
**                      to_speed : Ŀ���ٶ�
** Returned value:      pwm:PWMռ�ձ�
**pid.Kp*(pid.err-pid.err_next)+pid.Ki*pid.err+pid.Kd*(pid.err-2*pid.err_next+pid.err_last)
**û�п���������û���趨�����ޣ�ֻ�ǶԹ�ʽ��һ����ֱ�ӵ�ʵ��
***************************************************************************/
//int32_t Incremental_Pid(uint8_t motor_switch,uint16_t to_speed){

//	  if(motor_switch == RIGHT_MOTOR)pid_variable_p = right_pid_variable_p;
//	  else if(motor_switch == LEFT_MOTOR) pid_variable_p = left_pid_variable_p;
//	  int32_t pwm;

//    pid_variable_p->to_value = to_speed;//--����Ŀ���ٶ�
//	  pid_variable_p->current_value = speed_function_interface_handle_p->read_speed(motor_switch);//--��ǰ�ٶ�
//    pid_variable_p->current_err = pid_variable_p->to_value - pid_variable_p->current_value;//--��ǰ���
//   // printf("current_err %d\r\n",pid_variable_p->current_err);  
//	  pwm += 
//	            (pid_variable_p->kp*(pid_variable_p->current_err-pid_variable_p->last_err)
//	           +pid_variable_p->ki*pid_variable_p->current_err
//	           +pid_variable_p->kd*(pid_variable_p->current_err-2*pid_variable_p->last_err
//	           +pid_variable_p->last_last_err))/1000;
//    pid_variable_p->output_value =range(pwm,pid_variable_p->umin, pid_variable_p->umax);//--PWM��Χ����
//    pid_variable_p->last_last_err=pid_variable_p->last_err;
//    pid_variable_p->last_err=pid_variable_p->current_err;
//    return pid_variable_p->output_value;
//}
/***************************************************************************
** Function name:       Anti_Integral_Saturation_Pid
** Descriptions:        �����ֱ���PID�㷨��C����ʵ��
** input parameters:    motor_switc�����ѡ�񣬿���ȡһ��ֵ��
**                                  LEFT_MOTOR �� �����
**                                  RIGHT_MOTOR�� �Ҳ���
**                                  MOWING_MOTOR����ݵ��
**                      to_speed : Ŀ���ٶ�
** Returned value:      pwm:PWMռ�ձ�
**pid.Kp*(pid.err-pid.err_next)+pid.Ki*pid.err+pid.Kd*(pid.err-2*pid.err_next+pid.err_last)
**��ν���ֱ���������ָϵͳ����һ�������ƫ�PID��������������ڻ������õĲ����ۼӶ��Ӵ�
**�Ӷ�����ִ�л����ﵽ����λλ�ã��������������������ִ�������Ȳ�����������
**��ʱ���������������������������з�Χ�����뱥������һ��ϵͳ���ַ���ƫ�
**����������𽥴ӱ������˳������뱥����Խ�����˳�������ʱ��Խ���������ʱ�䣬
**ִ�л�����Ȼͣ���ڼ���λ�ö�����ƫ��������������Ӧ�ĸı䣬��ʱϵͳ��ʱ��һ����
**��ɿ������ܶ񻯣����������Ϊ���ֱ�����������ʧ��
**��ֹ���ֱ��͵ķ���֮һ���ǿ����ͷ����÷�����˼�����ڼ�������������ʱ�������ж�
**��һʱ�̵Ŀ������Ƿ��Ѿ������˼��޷�Χ�������һʱ��������ڼ��޷�Χ����ֻ�ۼ���ƫ��
**�Ӷ��������������Ŷ��ͣ���ڱ�����
***************************************************************************/
//int32_t  Anti_Integral_Saturation_Pid(uint8_t motor_switch,uint16_t to_speed){

//	 if(motor_switch == RIGHT_MOTOR)pid_variable_p = right_pid_variable_p;
//	  else if(motor_switch == LEFT_MOTOR) pid_variable_p = left_pid_variable_p; 
//  	pid_variable_p->kp = 0;
//	  pid_variable_p->ki = 0;
//	  pid_variable_p->kd = 0;
//    pid_variable_p->to_value = to_speed;//--����Ŀ���ٶ�
//	  pid_variable_p->current_value = speed_function_interface_handle_p->read_speed(motor_switch);//--��ǰ�ٶ��
//	  pid_variable_p->current_err = pid_variable_p->to_value - pid_variable_p->current_value;//--��ǰ���
//	  if(pid_variable_p->current_value>pid_variable_p->umax)//--������//--������ٶ�Ӧ�û�ΪPWM���õ��Ҫ�Ľ
//		  {
//		   if(pid_variable_p->current_err>200)//--���ַ���
//         {
//          pid_variable_p->ki = 0;
//         }else{
//          pid_variable_p->ki = 1;;
//         pid_variable_p->integral += pid_variable_p->current_err;//--�ۼӻ������
//         }
//		 }
//	 if(pid_variable_p->current_value<pid_variable_p->umin)//--������
//		  {
//		   if(pid_variable_p->current_err>200)//--���ַ���
//         {
//          pid_variable_p->ki = 0;
//         }else{
//          pid_variable_p->ki = 1;;
//         pid_variable_p->integral += pid_variable_p->current_err;//--�ۼӻ������
//         }
//		 }
//	if(pid_variable_p->current_err>200)//--���ַ���
//    {
//   pid_variable_p->ki = 0;
//    }else{
//   pid_variable_p->ki = 1;;
//   pid_variable_p->integral += pid_variable_p->current_err;//--�ۼӻ������
//    }
//    pid_variable_p->output_value = 
//	              pid_variable_p->kp*pid_variable_p->current_err
//	             +pid_variable_p->ki*pid_variable_p->integral
//	             +pid_variable_p->kd*(pid_variable_p->current_err-pid_variable_p->last_err);
//    pid_variable_p->last_err = pid_variable_p->current_err;//--������һ�����
//    return pid_variable_p->output_value;
//}

/***************************************************************************
** Function name:       Trapezoidal_Integraln_Pid
** Descriptions:        ���λ��ֵ�PID�㷨ʵ��
** input parameters:    motor_switc�����ѡ�񣬿���ȡһ��ֵ��
**                                  LEFT_MOTOR �� �����
**                                  RIGHT_MOTOR�� �Ҳ���
**                                  MOWING_MOTOR����ݵ��
**                      to_speed : Ŀ���ٶ�
** Returned value:      pwm:PWMռ�ձ�
**pid.Kp*pid.err+index*pid.Ki*pid.integral/2+pid.Kd*(pid.err-pid.err_last);  //--���λ���
**��ΪPID���ƵĻ������������������Ӧ��߻��������㾫�ȣ�Ϊ�˿��Խ����λ��ָ�Ϊ���λ��֣�
***************************************************************************/
//int32_t  Trapezoidal_Integraln_Pid(uint8_t motor_switch,uint16_t to_speed){

//	  if(motor_switch == RIGHT_MOTOR)pid_variable_p = right_pid_variable_p;
//	  else if(motor_switch == LEFT_MOTOR) pid_variable_p = left_pid_variable_p; 
//	 pid_variable_p->kp = 0;
//	  pid_variable_p->ki = 0;
//	  pid_variable_p->kd = 0;
//    pid_variable_p->to_value = to_speed;//--����Ŀ���ٶ�
//	  pid_variable_p->current_value = speed_function_interface_handle_p->read_speed(motor_switch);//--��ǰ�ٶ��
//	  pid_variable_p->current_err = pid_variable_p->to_value - pid_variable_p->current_value;//--��ǰ���
//	  if(pid_variable_p->current_value>pid_variable_p->umax)//--������//--������ٶ�Ӧ�û�ΪPWM���õ��Ҫ�Ľ
//		  {
//		   if(pid_variable_p->current_err>200)//--���ַ���
//         {
//          pid_variable_p->ki = 0;
//         }else{
//          pid_variable_p->ki = 1;;
//         pid_variable_p->integral += pid_variable_p->current_err;//--�ۼӻ������
//         }
//		 }
//	 if(pid_variable_p->current_value<pid_variable_p->umin)//--������
//		  {
//		   if(pid_variable_p->current_err>200)//--���ַ���
//         {
//          pid_variable_p->ki = 0;
//         }else{
//          pid_variable_p->ki = 1;;
//         pid_variable_p->integral += pid_variable_p->current_err;//--�ۼӻ������
//         }
//		 }
//	if(pid_variable_p->current_err>200)//--���ַ���
//    {
//   pid_variable_p->ki = 0;
//    }else{
//   pid_variable_p->ki = 1;;
//   pid_variable_p->integral += pid_variable_p->current_err;//--�ۼӻ������
//    }
//    pid_variable_p->output_value = 
//	              pid_variable_p->kp*pid_variable_p->current_err
//	             +pid_variable_p->ki*pid_variable_p->integral/2
//	             +pid_variable_p->kd*(pid_variable_p->current_err-pid_variable_p->last_err);
//    pid_variable_p->last_err = pid_variable_p->current_err;//--������һ�����
//    return pid_variable_p->output_value;
//}
/***************************************************************************
** Function name:       Variable_Integraln_Pid
** Descriptions:        ����ֵ�PID�㷨ʵ��
** input parameters:    motor_switc�����ѡ�񣬿���ȡһ��ֵ��
**                                  LEFT_MOTOR �� �����
**                                  RIGHT_MOTOR�� �Ҳ���
**                                  MOWING_MOTOR����ݵ��
**                      to_speed : Ŀ���ٶ�
** Returned value:      pwm:PWMռ�ձ�
**pid.Kp*pid.err+index*pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);  //--���λ���
**�����PID���Կ����ǻ��ַ����PID�㷨�ĸ�һ�����ʽ������ͨ��PID�㷨�У����ڻ���ϵ��KI�ǳ���
**�������������ƹ����У���������ʽ����ģ����ǣ�ϵͳ���ڻ������Ҫ���ǣ�ϵͳƫ���ʱ����������Ӧ�ü�������ȫ��
**������ƫ��Сʱ����Ӧ�ü�ǿ������ϵ��ȡ���˻�����������������ֱ��ͣ�ȡС���ֲ��ܶ�ʱ�����������ˣ�
**����ϵͳ��ƫ���С�ı�����ٶ����б�Ҫ�ġ�
**�����PID�Ļ���˼�����跨�ı��������ۼ��ٶȣ�ʹ����ƫ���С��Ӧ��ƫ��Խ�󣬻���Խ����ƫ��ԽС������Խ��
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
//	  pid_variable_p->current_value = speed_function_interface_handle_p->read_speed(motor_switch);//--��ǰ�ٶ��
//	  pid_variable_p->current_err = pid_variable_p->to_value - pid_variable_p->current_value;//--��ǰ���
//	
//	if(pid_variable_p->current_err>200)//--���ַ���
//    {
//    index = 0;
//	  rang1 = 400;
//    }
//	else if(pid_variable_p->current_err<50)
//		{
//   index = 2;
//   pid_variable_p->integral += pid_variable_p->current_err;//--�ۼӻ������
//    }
//	else
//	{
//	index = (200-pid_variable_p->current_err)/20;
//	index = 1;
//	pid_variable_p->integral += pid_variable_p->current_err;//--�ۼӻ������
//	}
//    pwm1 = 
//	             ( pid_variable_p->kp*pid_variable_p->current_err
//	             +index*pid_variable_p->ki*pid_variable_p->integral
//	             +pid_variable_p->kd*(pid_variable_p->current_err-pid_variable_p->last_err))/1000;
//	  pwm2+=range(pwm1, -rang1, rang1);//--PWM��Χ����
//	  pid_variable_p->output_value =range(pwm2, pid_variable_p->umin, pid_variable_p->umax);//--PWM��Χ����
//    pid_variable_p->last_err = pid_variable_p->current_err;//--������һ�����
//    return pid_variable_p->output_value;
//}
/***************************************************************************
** Function name:       
** Descriptions:        ר��PID��ģ��PID��C�����㷨ʵ��
** input parameters:    motor_switc�����ѡ�񣬿���ȡһ��ֵ��
**                                  LEFT_MOTOR �� �����
**                                  RIGHT_MOTOR�� �Ҳ���
**                                  MOWING_MOTOR����ݵ��
**                      to_speed : Ŀ���ٶ�
** Returned value:      pwm:PWMռ�ձ�

***************************************************************************/


