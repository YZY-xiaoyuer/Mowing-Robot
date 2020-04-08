/**
  ******************************************************************************
  * @file    WHEEL.c
  * @author  С�����Ѿ��
  * @version V0.0.1
  * @date    2019/7/30
  * @brief   ٤���Ի��������ӿ���
  ******************************************************************************/
 #include "wheel.h"
 #include "stdio.h"
 
 /*����*/
 /*�ٶȸ���*/
uint8_t Wheel_Speed_Updata(uint8_t motor_switch);
 /*��ʼ��*/
uint8_t Wheel_Iint(uint8_t motor_switch);
 /*����ʼ��*/
uint8_t Wheel_Deinit(uint8_t motor_switch);
 /*��������*/
uint8_t Wheel_start(uint8_t motor_switch,uint32_t speed);
 /*����ɲ��*/
uint8_t Wheel_Stop(uint8_t motor_switch);
 /*����ɲ��*/
uint8_t Wheel_Brake(uint8_t motor_switch);
/*�����������*/
uint32_t Count_Wheel_Walk_Distance(uint8_t motor_switch);
/*��ȡ���Ӿ���*/
uint32_t Get_Wheel_Walk_Distance(uint8_t motor_switch);
/*���Ӿ�������*/
uint8_t Wheel_Walk_Distance_Reset(uint8_t motor_switch);
/*��ȡ�ٶ�*/
static int32_t Read_Speed(ST_wheel_variable_p wheel_variable_p);
//--����
 /*�ٶȸ���*/
uint8_t Left_Wheel_Speed_Updata(void);
 /*��ʼ��*/
uint8_t Left_Wheel_Iint(void);
 /*����ʼ��*/
uint8_t Left_Wheel_Deinit(void);
 /*��������*/
uint8_t Left_Wheel_start(uint32_t speed);
 /*����ɲ��*/
uint8_t Left_Wheel_Stop(void);
 /*����ɲ��*/
uint8_t Left_Wheel_Brake(void);
/*�����������*/
uint32_t Count_Left_Wheel_Walk_Distance(void);
/*��ȡ���Ӿ���*/
uint32_t Get_Left_Wheel_Walk_Distance(void);
/*���Ӿ�������*/
uint8_t Left_Wheel_Walk_Distance_Reset(void);
/*��ȡ�ٶ�*/
int32_t Left_Wheel_Read_Speed(void);
//--�ҵ��
 /*�ٶȸ���*/
uint8_t Right_Wheel_Speed_Updata(void);
 /*��ʼ��*/
uint8_t Right_Wheel_Iint(void);
 /*����ʼ��*/
uint8_t Right_Wheel_Deinit(void);
 /*��������*/
uint8_t Right_Wheel_start(uint32_t speed);
 /*����ɲ��*/
uint8_t Right_Wheel_Stop(void);
 /*����ɲ��*/
uint8_t Right_Wheel_Brake(void);
/*�����������*/
uint32_t Count_Right_Wheel_Walk_Distance(void);
/*��ȡ���Ӿ���*/
uint32_t Get_Right_Wheel_Walk_Distance(void);
/*���Ӿ�������*/
uint8_t Right_Wheel_Walk_Distance_Reset(void);
/*��ȡ�ٶ�*/
int32_t Right_Wheel_Read_Speed(void);

/*����*/
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
 /*�����ӿ�*/
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
** Descriptions:        �����ٶȸ���
**                      �ܵ��������ڵ����ƣ�Ŀǰ��������Ϊ100ms  10hz 
**                      ���Ա�����ÿ100ms����һ��
** input parameters:    motor_switc�����ѡ�񣬿���ȡһ��ֵ��
**                                  LEFT_MOTOR �� �����
**                                  RIGHT_MOTOR�� �Ҳ���
**                                  MOWING_MOTOR����ݵ��
** Returned value:      0
***************************************************************************/
 uint8_t Wheel_Speed_Updata(uint8_t motor_switch)
 {
  if(motor_switch == RIGHT_MOTOR)
	 {	
		
		 /*�����ٶ�*/
		right_speed_handle_p->refresh_speed();	 
		if(car_handle_p->read_car_start_stop() == CAR_START)/*С������*/
		 {	 
		  /*����PWM*/
		  right_pid_handle_p->wheel_integral_separation_pid(1000);	
		 /*����PWM*/
		 right_motor_handle_p->start(right_pid_handle_p->read_pwm());	
		 }
	 }
	 if(motor_switch == LEFT_MOTOR)
	 { 	

		 /*�����ٶ�*/
		left_speed_handle_p->refresh_speed();
		if(car_handle_p->read_car_start_stop() == CAR_START)/*С������*/
		{	
		/*����PWM*/
		left_pid_handle_p->wheel_integral_separation_pid(1000);
		/*����PWM*/
		left_motor_handle_p->start(left_pid_handle_p->read_pwm());/*�������*/		
		}
	 }
	 return 0;
 }
/***************************************************************************
** Function name:       Wheel_Iint
** Descriptions:        ���ӳ�ʼ��
**                      ���������ʼ��
**                      ���ٳ�ʼ��
** input parameters:    motor_switc�����ѡ�񣬿���ȡһ��ֵ��
**                                  LEFT_MOTOR �� �����
**                                  RIGHT_MOTOR�� �Ҳ���
**                                  MOWING_MOTOR����ݵ��
** Returned value:      0
***************************************************************************/
 uint8_t Wheel_Iint(uint8_t motor_switch)
 {
	 /*���ѡ��*/
	 if(motor_switch == RIGHT_MOTOR)
	 {
	   /*�����ʼ��*/	 
		right_motor_handle_p->init();
	  /*�ٶȳ�ʼ��*/
		right_speed_handle_p->speed_init();
	 }
	 if(motor_switch == LEFT_MOTOR)
	 {
	  /*�����ʼ��*/	
     left_motor_handle_p->init();		 
	  /*�ٶȳ�ʼ��*/
		 left_speed_handle_p->speed_init();
	 }
	 return 0;
 }
/***************************************************************************
** Function name:       Wheel_Deinit
** Descriptions:        ���ӳ�ʼ��
**                      ���������ʼ��
**                      ���ٳ�ʼ��
** input parameters:    motor_switc�����ѡ�񣬿���ȡһ��ֵ��
**                                  LEFT_MOTOR �� �����
**                                  RIGHT_MOTOR�� �Ҳ���
**                                  MOWING_MOTOR����ݵ��
** Returned value:      0
***************************************************************************/
  uint8_t Wheel_Deinit(uint8_t motor_switch)
 { 
	 /*���ѡ��*/
	 if(motor_switch == RIGHT_MOTOR)
	 {
	   /*�����ʼ��*/	 
		right_motor_handle_p->deinit();
	  /*�ٶȳ�ʼ��*/
		right_speed_handle_p->speed_deinit();
	 }
	 if(motor_switch == LEFT_MOTOR)
	 {
	  /*�����ʼ��*/	
     left_motor_handle_p->deinit();		 
	  /*�ٶȳ�ʼ��*/
		 left_speed_handle_p->speed_deinit();
	 }
	 return 0;
 }
  /***************************************************************************
** Function name:     Wheel_start
** Descriptions:       ��������
** input parameters:    motor_switc�����ѡ�񣬿���ȡһ��ֵ��
**                                  LEFT_MOTOR �� �����
**                                  RIGHT_MOTOR�� �Ҳ���
**                                  MOWING_MOTOR����ݵ��
** Returned value:                  0
***************************************************************************/
 uint8_t Wheel_start(uint8_t motor_switch,uint32_t speed)
 { 
	 /*���ѡ��*/
	 if(motor_switch == RIGHT_MOTOR) 
		 right_pid_handle_p->set_wheel_pid_speed(speed);
	 if(motor_switch == LEFT_MOTOR)
		 left_pid_handle_p->set_wheel_pid_speed(speed);			 
  return 0;	 
 }
  /***************************************************************************
** Function name:       Wheel_Stop
** Descriptions:        ����ֹͣ����ģʽ�����ӻ���Ϊ���Լ���ת��һ�ξ���
** input parameters:    motor_switc�����ѡ�񣬿���ȡһ��ֵ��
**                                  LEFT_MOTOR �� �����
**                                  RIGHT_MOTOR�� �Ҳ���
**                                  MOWING_MOTOR����ݵ��
** Returned value:                  0
***************************************************************************/
uint8_t Wheel_Stop(uint8_t motor_switch)
 {

	 
	 /*���ѡ��*/
	 if(motor_switch == RIGHT_MOTOR)
	 {
		  right_motor_handle_p->stop();
		 /*PWM��������*/
		 right_pid_handle_p->pwm_value_reset();

	 }
	 if(motor_switch == LEFT_MOTOR) 
	 {
		 left_motor_handle_p->stop();
		 /*PWM��������*/
		 left_pid_handle_p->pwm_value_reset();
	 }
	 return 0;
 }
/***************************************************************************
** Function name:       Wheel_Brake
** Descriptions:        ����ɲ������ģʽ�����ӻ�ֱ�ӱ���
** input parameters:    motor_switc�����ѡ�񣬿���ȡһ��ֵ��
**                                  LEFT_MOTOR �� �����
**                                  RIGHT_MOTOR�� �Ҳ���
**                                  MOWING_MOTOR����ݵ��
** Returned value:                  0
***************************************************************************/
   uint8_t Wheel_Brake(uint8_t motor_switch)
 {
	 /*���ѡ��*/
	 if(motor_switch == RIGHT_MOTOR)
	 {
		 right_motor_handle_p->brake();
		  /*PWM��������*/
		 right_pid_handle_p->pwm_value_reset();

	 }
	 if(motor_switch == LEFT_MOTOR)	 
	 {
		 left_motor_handle_p->brake();
		 		 /*PWM��������*/
		 left_pid_handle_p->pwm_value_reset();

	 }
	 return 0;
 }
   /***************************************************************************
** Function name:       Count_Wheel_Walk_Distance
** Descriptions:        ����������� �ܵ������������ƣ�Ŀǰ100ms����һ��
** input parameters:    motor_switc�����ѡ�񣬿���ȡһ��ֵ��
**                                  LEFT_MOTOR �� �����
**                                  RIGHT_MOTOR�� �Ҳ���
**                                  MOWING_MOTOR����ݵ��
** Returned value:       0
***************************************************************************/
uint32_t Count_Wheel_Walk_Distance(uint8_t motor_switch)
{
	/*���ѡ��*/
	 if(motor_switch == RIGHT_MOTOR)
	 {
		right_wheel_variable_p->wheel_mm += right_speed_handle_p->get_motor_cnt()*10*PI*WHEEL_D/3/REDUCTION_RATIO/1000;//--���뵥λmm;
	  right_wheel_handle_p->walk_distance_reset();//--��������������ֵ
	 }
	 if(motor_switch == LEFT_MOTOR) 
	 {
	 left_wheel_variable_p->wheel_mm += left_speed_handle_p->get_motor_cnt()*10*PI*WHEEL_D/3/REDUCTION_RATIO/1000;//--���뵥λmm;
	 left_wheel_handle_p->walk_distance_reset();//--��������������ֵ
	 }
	 return 0;
}
   /***************************************************************************
** Function name:       Get_Wheel_Walk_Distance
** Descriptions:        ��ȡ������̼� 
** input parameters:    motor_switc�����ѡ�񣬿���ȡһ��ֵ��
**                                  LEFT_MOTOR �� �����
**                                  RIGHT_MOTOR�� �Ҳ���
**                                  MOWING_MOTOR����ݵ��
** Returned value:       0
***************************************************************************/
uint32_t Get_Wheel_Walk_Distance(uint8_t motor_switch)
{
uint32_t distance = 0;
	/*���ѡ��*/
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
** Descriptions:        ��������������
** input parameters:    motor_switc�����ѡ�񣬿���ȡһ��ֵ��
**                                  LEFT_MOTOR �� �����
**                                  RIGHT_MOTOR�� �Ҳ���
**                                  MOWING_MOTOR����ݵ��
** Returned value:     0
***************************************************************************/

uint8_t Wheel_Walk_Distance_Reset(uint8_t motor_switch)
{
	/*���ѡ��*/
	 if(motor_switch == RIGHT_MOTOR)
		right_speed_handle_p->motor_cnt_reset();
	 if(motor_switch == LEFT_MOTOR) 
    left_speed_handle_p->motor_cnt_reset();
	 return 0;
}

 /***************************************************************************
** Function name:       Read_Speed
** Descriptions:        ��ȡ�����ٶ�
** input parameters:    motor_switc�����ѡ�񣬿���ȡһ��ֵ��
**                                  LEFT_MOTOR �� �����
**                                  RIGHT_MOTOR�� �Ҳ���
**                                  MOWING_MOTOR����ݵ��
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
/*��ȡ�����ٶ�*/
int32_t Left_Wheel_Read_Speed(void)
{
 return Read_Speed(left_wheel_variable_p);
}
/*��ȡ�����ٶ�*/
int32_t Right_Wheel_Read_Speed(void)
{
 return Read_Speed(right_wheel_variable_p);
}
//--����
 /*�ٶȸ���*/
uint8_t Left_Wheel_Speed_Updata(void)
{
 return Wheel_Speed_Updata(LEFT_MOTOR);
}
 /*��ʼ��*/
uint8_t Left_Wheel_Iint(void)
{
return Wheel_Iint(LEFT_MOTOR);
}
 /*����ʼ��*/
uint8_t Left_Wheel_Deinit(void)
{
return Wheel_Deinit(LEFT_MOTOR);
}
 /*��������*/
uint8_t Left_Wheel_start(uint32_t speed)
{
return Wheel_start(LEFT_MOTOR,speed);
}
 /*����ɲ��*/
uint8_t Left_Wheel_Stop(void)
{
 return Wheel_Stop(LEFT_MOTOR);
}
 /*����ɲ��*/
uint8_t Left_Wheel_Brake(void)
{
 return Wheel_Brake(LEFT_MOTOR);
}
/*�����������*/
uint32_t Count_Left_Wheel_Walk_Distance(void)
{
 return Count_Wheel_Walk_Distance(LEFT_MOTOR);
}
/*��ȡ���Ӿ���*/
uint32_t Get_Left_Wheel_Walk_Distance(void)
{
 return Get_Wheel_Walk_Distance(LEFT_MOTOR);
}
/*���Ӿ�������*/
uint8_t Left_Wheel_Walk_Distance_Reset(void)
{
 return  Wheel_Walk_Distance_Reset(LEFT_MOTOR);
}
//--�ҵ��
 /*�ٶȸ���*/
uint8_t Right_Wheel_Speed_Updata(void)
{
 return Wheel_Speed_Updata(RIGHT_MOTOR);
}
 /*��ʼ��*/
uint8_t Right_Wheel_Iint(void)
{
return Wheel_Iint(RIGHT_MOTOR);
}
 /*����ʼ��*/
uint8_t Right_Wheel_Deinit(void)
{
return Wheel_Deinit(RIGHT_MOTOR);
}
 /*��������*/
uint8_t Right_Wheel_start(uint32_t speed)
{
return Wheel_start(RIGHT_MOTOR,speed);
}
 /*����ɲ��*/
uint8_t Right_Wheel_Stop(void)
{
 return Wheel_Stop(RIGHT_MOTOR);
}
 /*����ɲ��*/
uint8_t Right_Wheel_Brake(void)
{
 return Wheel_Brake(RIGHT_MOTOR);
}
/*�����������*/
uint32_t Count_Right_Wheel_Walk_Distance(void)
{
 return Count_Wheel_Walk_Distance(RIGHT_MOTOR);
}
/*��ȡ���Ӿ���*/
uint32_t Get_Right_Wheel_Walk_Distance(void)
{
 return Get_Wheel_Walk_Distance(RIGHT_MOTOR);
}
/*���Ӿ�������*/
uint8_t Right_Wheel_Walk_Distance_Reset(void)
{
 return  Wheel_Walk_Distance_Reset(RIGHT_MOTOR);
}
