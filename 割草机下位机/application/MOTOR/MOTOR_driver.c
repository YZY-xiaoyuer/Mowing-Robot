/**
  ******************************************************************************
  * @file    MOTOR_drive.c
  * @author  С�����Ѿ��
  * @version V0.0.1
  * @date    2019/7/17
  * @brief   ٤���Ի����˵������
  ******************************************************************************        
*/

#include "MOTOR_driver.h"
#include "stdio.h"

//==============================����=====================================
/*�����ʼ��*/
uint8_t Motor_Init(ST_motor_ctrl_p wheel_ctrl_handle_p,
                   ST_motor_ctrl_parameter_p wheel_para_handle_p);
/*�������ʼ��*/
uint8_t Motor_Deinit(ST_motor_ctrl_p wheel_ctrl_handle_p,
                     ST_motor_ctrl_parameter_p wheel_para_handle_p);
/*�������*/
uint8_t Motor_Start(ST_motor_ctrl_p wheel_ctrl_handle_p,
                    ST_motor_ctrl_parameter_p wheel_para_handle_p, 
                    int32_t pwm);
/*����ֹͣ*/
uint8_t Motor_Stop(ST_motor_ctrl_p wheel_ctrl_handle_p,
                   ST_motor_ctrl_parameter_p wheel_para_handle_p);
/*�����ƶ�*/
uint8_t Motor_Brake(ST_motor_ctrl_p wheel_ctrl_handle_p,
                    ST_motor_ctrl_parameter_p wheel_para_handle_p);
/*���״̬��ȡ*/
uint8_t  Get_Motor_Station(ST_motor_ctrl_parameter_p wheel_para_handle_p,
                           uint16_t motor_station[2]);

/*������ʹ��*/
uint8_t Motor_Driver_Output_Enable (ST_motor_ctrl_p wheel_ctrl_handle_p,
                                    ST_motor_ctrl_parameter_p wheel_para_handle_p);
/*���PWMռ�ձ�����*/
uint8_t Motor_Speed_Pwm_Duty_Cycle_Set(ST_motor_ctrl_p wheel_ctrl_handle_p,
                                       ST_motor_ctrl_parameter_p wheel_para_handle_p);
/*���ת������*/
uint8_t Motor_Fwd_Rev_Select (ST_motor_ctrl_p wheel_ctrl_handle_p,
	                        ST_motor_ctrl_parameter_p wheel_para_handle_p);
/*���ɲ��ʹ��*/
uint8_t Motor_Brake_Enable (ST_motor_ctrl_p wheel_ctrl_handle_p,
	                      ST_motor_ctrl_parameter_p wheel_para_handle_p);
/*�������ģʽ����*/
uint8_t Motor_Driver_Work_Mode (ST_motor_ctrl_p wheel_ctrl_handle_p,
	                          ST_motor_ctrl_parameter_p wheel_para_handle_p);

 //--����
/*������ʼ��*/
uint8_t Left_Motor_Init(void);
/*��������ʼ��*/
uint8_t Left_Motor_Deinit(void);
/*��������*/
uint8_t Left_Motor_Start(int32_t pwm);
/*������ֹͣ*/
uint8_t Left_Motor_Stop(void);
/*������ƶ�*/
uint8_t Left_Motor_Brake(void);
/*��ȡ����״̬*/
uint8_t  Get_Left_Motor_Station(uint16_t motor_station[2]);
//--�ҵ��
/*�ҵ����ʼ��*/
uint8_t Right_Motor_Init(void);
/*�ҵ������ʼ��*/
uint8_t Right_Motor_Deinit(void);
/*�ҵ������*/
uint8_t Right_Motor_Start(int32_t pwm);
/*������ֹͣ*/
uint8_t Right_Motor_Stop(void);
/*�ҽ����ƶ�*/
uint8_t Right_Motor_Brake(void);
/*��ȡ�ҵ��״̬*/
uint8_t  Get_Right_Motor_Station(uint16_t motor_station[2]);
//--�Ҳ೵�ֵ�����ƽṹ�� 
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
    &htim1,  //--PWM���
    TIM_CHANNEL_4,
};
ST_motor_ctrl_p right_wheel_ctrl_handle_p = &right_wheel_ctrl_handle;
//-- �Ҳ೵�ֵ�����ƽṹ�� 
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
    &htim1,  /*PWM��*/
    TIM_CHANNEL_3,
};
ST_motor_ctrl_p left_wheel_ctrl_handle_p = &left_wheel_ctrl_handle;
/*==============================================================================*/
/*�Ҳ೵�ֵ�����Ʋ����ṹ��*/ 
ST_motor_ctrl_parameter   right_wheel_para_handle={
    output_disable,             /*������������ʹ�ܱ�־*/
    motor_fwd,                  /*�����ת�����־*/
  	brake_disable,              /*ɲ��ʹ�ܱ�־*/ 
	  work_mode_normal,         /*�������оƬѡ��*/ 
    right_motor,	                /*���ѡ��*/
    0                           /*PWM*/
};
ST_motor_ctrl_parameter_p right_wheel_para_handle_p =&right_wheel_para_handle;

/*��೵�ֵ�����Ʋ����ṹ�� */ 
ST_motor_ctrl_parameter left_wheel_para_handle={
    output_disable,             //--������������ʹ�ܱ�־
    motor_rev,                  //--�����ת�����־
    brake_disable,              //--ɲ��ʹ�ܱ�־ 
	  work_mode_normal,         //--�������оƬѡ�� 
    left_motor,	
    0                           //--ת��PWM���� 
};
ST_motor_ctrl_parameter_p left_wheel_para_handle_p =&left_wheel_para_handle;

/*==============================================================================*/
/*���������ӿ�*/
 ST_motor_handle   left_motor_handle = {
  Left_Motor_Init,
  Left_Motor_Deinit,
  Left_Motor_Start,
  Left_Motor_Stop,
  Left_Motor_Brake,
	Get_Left_Motor_Station
 };
 ST_motor_handle_p left_motor_handle_p = &left_motor_handle;
 /*�ҵ�������ӿ�*/
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
** Descriptions:        �����ʼ�� 
**                      ��ʼ���������Ĭ��С��ǰ������ע���������ַ����෴
**                      ��ʼ�����ģʽ �����õ��Ϊ��������ģʽ
** input parameters:    wheel_ctrl_handle_p��������ƽṹ��
**                      wheel_para_handle_p����������ṹ��
** Returned value:      0
***************************************************************************/
uint8_t Motor_Init(ST_motor_ctrl_p wheel_ctrl_handle_p,
                   ST_motor_ctrl_parameter_p wheel_para_handle_p)
{
  	/*�ҵ�������������෴*/
	  if(wheel_para_handle_p->motor_switc == right_motor)
	  wheel_para_handle_p->fwd_rev_flag = motor_fwd;
	  if(wheel_para_handle_p->motor_switc == left_motor)
	  wheel_para_handle_p->fwd_rev_flag = motor_rev;
		Motor_Fwd_Rev_Select (wheel_ctrl_handle_p, wheel_para_handle_p);
	 /*���ģʽѡ��*/
		wheel_para_handle_p->motor_driver_work_mode = work_mode_normal;
	 Motor_Driver_Work_Mode (wheel_ctrl_handle_p, wheel_para_handle_p);
	return 0;
}
/***************************************************************************
** Function name:       Motor_Deinit
** Descriptions:        �������ʼ�� 
**                      ���õ��Ϊ�����ģʽ
** input parameters:    wheel_ctrl_handle_p��������ƽṹ��
**                      wheel_para_handle_p����������ṹ��
** Returned value:      0
***************************************************************************/
uint8_t Motor_Deinit(ST_motor_ctrl_p wheel_ctrl_handle_p,
                     ST_motor_ctrl_parameter_p wheel_para_handle_p)
{
  	/*���ģʽѡ��*/
	wheel_para_handle_p->motor_driver_work_mode = work_mode_output_disable;
	Motor_Driver_Work_Mode (wheel_ctrl_handle_p, wheel_para_handle_p);
	return 0;
}
/***************************************************************************
** Function name:       Motor_Start
** Descriptions:        �������
**                      ����PWM������ѡ������ת����ע���������ַ������෴��
**                      ע�⣬PWM�����ʱ�������и�������PWM������õ�ʱ��ֻ������
**                      ���õ��ģʽΪ��������ģʽ
** input parameters:    wheel_ctrl_handle_p��������ƽṹ��
**                      wheel_para_handle_p����������ṹ��
**                      PWM�����ת��PWM��
** Returned value:      0
***************************************************************************/
uint8_t Motor_Start(ST_motor_ctrl_p wheel_ctrl_handle_p,
                    ST_motor_ctrl_parameter_p wheel_para_handle_p, 
                    int32_t pwm)
{
  if(pwm >=0)//--ǰ��
	{
	if(wheel_para_handle_p->motor_switc == right_motor)
	  wheel_para_handle_p->fwd_rev_flag = motor_fwd;
	  if(wheel_para_handle_p->motor_switc == left_motor)
	  wheel_para_handle_p->fwd_rev_flag = motor_rev;
	wheel_para_handle_p->pwm_value = pwm;
	}

  if(pwm <0)//--����
	{
		if(wheel_para_handle_p->motor_switc == right_motor)
	  wheel_para_handle_p->fwd_rev_flag = motor_rev;
	  if(wheel_para_handle_p->motor_switc == left_motor)
	  wheel_para_handle_p->fwd_rev_flag = motor_fwd;
		wheel_para_handle_p->pwm_value = -pwm;
	}		
	
	 Motor_Fwd_Rev_Select (wheel_ctrl_handle_p,wheel_para_handle_p);
	  /*ռ�ձ�����*/
	 Motor_Speed_Pwm_Duty_Cycle_Set(wheel_ctrl_handle_p,wheel_para_handle_p);
	 /*���ģʽѡ��*/
		wheel_para_handle_p->motor_driver_work_mode = work_mode_normal;
	 Motor_Driver_Work_Mode (wheel_ctrl_handle_p, wheel_para_handle_p);
 
	return 0;
}
/***************************************************************************
** Function name:       Motor_Stop
** Descriptions:        �������ֹͣ��ע�����ڴ�ģʽ�»���Ϊ���Խ��л���
**                      �޸ĵ������ģʽ
**                      ע�� һ��ҪֹͣPWM����������ǰ�PWM����Ϊ0������ᵼ�µ��
**                      ����������ʱ��ǿ�Ҷ�����
** input parameters:    wheel_ctrl_handle_p��������ƽṹ��
**                      wheel_para_handle_p����������ṹ��
**                      PWM�����ת��PWM��
** Returned value:      0
***************************************************************************/
uint8_t Motor_Stop(ST_motor_ctrl_p wheel_ctrl_handle_p,
                   ST_motor_ctrl_parameter_p wheel_para_handle_p)
{
   /*���ģʽѡ��*/
	 wheel_para_handle_p->motor_driver_work_mode = work_mode_output_disable;
	 Motor_Driver_Work_Mode (wheel_ctrl_handle_p, wheel_para_handle_p); 
	/*�ر�PWM*/
	  HAL_TIM_PWM_Stop( wheel_ctrl_handle_p->phtim_Speed_Adjust_PWM,  
	                   wheel_ctrl_handle_p->Channel_Speed_Adjust_PWM); 
	return 0;
}
/***************************************************************************
** Function name:       Motor_Brake
** Descriptions:        ��������ƶ���ע�����ڴ�ģʽ�»ᱧ��
**                      �޸ĵ������ģʽ
**                      ע�� һ��ҪֹͣPWM����������ǰ�PWM����Ϊ0������ᵼ�µ��
**                      ����������ʱ��ǿ�Ҷ�����
** input parameters:    wheel_ctrl_handle_p��������ƽṹ��
**                      wheel_para_handle_p����������ṹ��
**                      PWM�����ת��PWM��
** Returned value:      0
***************************************************************************/
/*�����ƶ�*/
uint8_t Motor_Brake(ST_motor_ctrl_p wheel_ctrl_handle_p,
                   ST_motor_ctrl_parameter_p wheel_para_handle_p)
{
   /*���ģʽѡ��*/
	 wheel_para_handle_p->motor_driver_work_mode = work_mode_brake_hard;
	 Motor_Driver_Work_Mode (wheel_ctrl_handle_p, wheel_para_handle_p); 
	/*�ر�PWM*/
	  HAL_TIM_PWM_Stop( wheel_ctrl_handle_p->phtim_Speed_Adjust_PWM,  
	                   wheel_ctrl_handle_p->Channel_Speed_Adjust_PWM); 
	return 0;
}

/***************************************************************************
** Function name:       Motor_Driver_Output_Enable
** Descriptions:        ������������ʹ�ܿ���  
** input parameters:    wheel_ctrl_handle_p��������ƽṹ��
**                      wheel_para_handle_p����������ṹ��
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
** Descriptions:        ת��ռ�ձȵ���
** input parameters:    wheel_ctrl_handle_p��������ƽṹ��
**                      wheel_para_handle_p����������ṹ��
**                       �޷�Ƶ 72MHZ  ÿ��PWM���ڼ���2500 
**                       ������ 0-2500
** Returned value:      0
***************************************************************************/
uint8_t Motor_Speed_Pwm_Duty_Cycle_Set(ST_motor_ctrl_p wheel_ctrl_handle_p,
                                       ST_motor_ctrl_parameter_p wheel_para_handle_p)
{

  TIM_OC_InitTypeDef sConfigOC;

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = wheel_para_handle_p->pwm_value;/*ռ�ձ�����*/
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
** Descriptions:        �������תѡ��
** input parameters:    wheel_ctrl_handle_p��������ƽṹ��
**                      wheel_para_handle_p����������ṹ�� 
** Returned value:      0:����
                        MOTOR_FWD_REV_SELECT_DEFAULT������
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
** Descriptions:        ����������ƶ�����
** input parameters:    wheel_ctrl_handle_p��������ƽṹ��
**                      wheel_para_handle_p����������ṹ���
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
** Descriptions:        ��ȡ���ת�ٽ���������ֵ   
** input parameters:    wheel_ctrl_handle_p��������ƽṹ��
**                      wheel_para_handle_p����������ṹ��
** Returned value:      ��ǰ���ת�ٽ���������ֵ
***************************************************************************/
//float Motor_Speed_Cnt_Get (ST_motor_ctrl_p wheel_ctrl_handle_p,)
//{  

//   return __HAL_TIM_GET_COUNTER(wheel_ctrl_handle_p->phtim_DecoderCounter);
//}
/***************************************************************************
** Function name:       Motor_Speed_Cnt_Set  
** Descriptions:        ���õ��ת�ٽ���������ֵ   
** input parameters:    wheel_ctrl_handle_p��������ƽṹ��
**                      wheel_para_handle_p����������ṹ��
**                      cnt             ��Ҫ���õļ�������ֵ 0-2500
** Returned value:      MOTOR_SWITCH_DEFAULT
***************************************************************************/

//__INLINE int Motor_Speed_Cnt_Set (ST_motor_ctrl_p wheel_ctrl_handle_p, uint16_t cnt)
//{

//    __HAL_TIM_SET_COUNTER(wheel_ctrl_handle_p->phtim_DecoderCounter,cnt);
//		 return 0;
//}

/***************************************************************************
** Function name:       Motor_Driver_Work_Mode  
** Descriptions:        ���õ������оƬ�Ĺ���ģʽ   
** input parameters:    wheel_ctrl_handle_p��������ƽṹ��
**                      wheel_para_handle_p����������ṹ��
** Returned value:      0
***************************************************************************/
uint8_t Motor_Driver_Work_Mode (ST_motor_ctrl_p wheel_ctrl_handle_p,
                                ST_motor_ctrl_parameter_p wheel_para_handle_p)
{   
		//--ģʽѡ�� 
    if(wheel_para_handle_p->motor_driver_work_mode == work_mode_brake_hard) {
        /* ������Ҫ����ɲ���ĳ��ϣ�Ӧ������ٶȿ��Ƶ��ִ��ɲ������ */
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
** Descriptions:        ��ȡ���״̬ 
** input parameters:    motor_switc�����ѡ�񣬿���ȡһ��ֵ��
**                                  LEFT_MOTOR �� �����
**                                  RIGHT_MOTOR�� �Ҳ���
**                                  MOWING_MOTOR����ݵ��
**                      motor_station[0]
**                      xxxx xxxx
**                      bit 0  : 0-������� 1-���ʹ��
**                      bit 1  : 0-ǰ��     1-����
**                      bit 2  : 0-ɲ������ 1-ɲ��ʹ��
**                      bit 3-4: 00-����ģʽ 01-�����ģʽ 10-����ɲ��ģʽ
**                      bit 5-15 : ����
**                      motor_station[1]
**                      ���ת��ռ�ձ�
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
** Descriptions:        ������ʼ�������õ����ʼ����������
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Left_Motor_Init(void)
{
	return Motor_Init(left_wheel_ctrl_handle_p,left_wheel_para_handle_p);
}
/***************************************************************************
** Function name:       Left_Motor_Deinit
** Descriptions:        ��������ʼ�������õ������ʼ����������
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Left_Motor_Deinit(void)
{
	return Motor_Deinit(left_wheel_ctrl_handle_p,left_wheel_para_handle_p);
}
/***************************************************************************
** Function name:       Left_Motor_Start
** Descriptions:        �������������õ��������������
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Left_Motor_Start(int32_t pwm)
{
	return Motor_Start(left_wheel_ctrl_handle_p,left_wheel_para_handle_p,pwm);
}
/***************************************************************************
** Function name:       Left_Motor_Stop
** Descriptions:        ��������ֹͣ�����õ������ֹͣ��������
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Left_Motor_Stop(void)
{
	return Motor_Stop(left_wheel_ctrl_handle_p,left_wheel_para_handle_p);
}
/***************************************************************************
** Function name:       Left_Motor_Brake
** Descriptions:        ���������ƶ������õ�������ƶ���������
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Left_Motor_Brake(void)
{ 
	return Motor_Brake(left_wheel_ctrl_handle_p,left_wheel_para_handle_p);
}
/***************************************************************************
** Function name:       Right_Motor_Init
** Descriptions:        �ҵ����ʼ�������õ����ʼ����������
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Right_Motor_Init(void)
{
	return Motor_Init(right_wheel_ctrl_handle_p,right_wheel_para_handle_p);
}
/***************************************************************************
** Function name:       Right_Motor_Deinit
** Descriptions:        �ҵ������ʼ�������õ������ʼ����������
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Right_Motor_Deinit(void)
{
 return Motor_Deinit(right_wheel_ctrl_handle_p,right_wheel_para_handle_p);
}
/***************************************************************************
** Function name:       Right_Motor_Start
** Descriptions:        �ҵ�����������õ��������������
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Right_Motor_Start(int32_t pwm)
{
 return Motor_Start(right_wheel_ctrl_handle_p,right_wheel_para_handle_p,pwm);
}
/***************************************************************************
** Function name:       Right_Motor_Stop
** Descriptions:        �ҵ������ֹͣ�����õ������ֹͣ��������
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Right_Motor_Stop(void)
{
 return Motor_Stop(right_wheel_ctrl_handle_p,right_wheel_para_handle_p);
}
/***************************************************************************
** Function name:       Right_Motor_Brake
** Descriptions:        �ҵ�������ƶ������õ�������ƶ���������
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Right_Motor_Brake(void)
{
 return Motor_Brake(right_wheel_ctrl_handle_p,right_wheel_para_handle_p);
}
/***************************************************************************
** Function name:       Get_Right_Motor_Station
** Descriptions:        ��ȡ�ҵ��״̬�����õ��״̬��������
** input parameters:    motor_station[2]��״̬����
** Returned value:      0
***************************************************************************/
uint8_t  Get_Right_Motor_Station(uint16_t motor_station[2])
{
return  Get_Motor_Station(right_wheel_para_handle_p,motor_station);
}
/***************************************************************************
** Function name:       Get_Left_Motor_Station
** Descriptions:        ��ȡ����״̬�����õ��״̬��������
** input parameters:    motor_station[2]��״̬����
** Returned value:      0
***************************************************************************/
uint8_t  Get_Left_Motor_Station(uint16_t motor_station[2])
{
return  Get_Motor_Station(left_wheel_para_handle_p,motor_station);
}
