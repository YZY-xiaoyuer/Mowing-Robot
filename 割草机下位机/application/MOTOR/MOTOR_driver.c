/**
  ******************************************************************************
  * @file    MOTOR_drive.c
  * @author  Ð¡Óã¶ù·ÉÑ¾·É
  * @version V0.0.1
  * @date    2019/7/17
  * @brief   Ù¤ÀûÂÔ»úÆ÷ÈËµç»úÇý¶¯
  ******************************************************************************        
*/

#include "MOTOR_driver.h"
#include "stdio.h"

//==============================º¯Êý=====================================
/*µç»ú³õÊ¼»¯*/
uint8_t Motor_Init(ST_motor_ctrl_p wheel_ctrl_handle_p,
                   ST_motor_ctrl_parameter_p wheel_para_handle_p);
/*µç»ú·´³õÊ¼»¯*/
uint8_t Motor_Deinit(ST_motor_ctrl_p wheel_ctrl_handle_p,
                     ST_motor_ctrl_parameter_p wheel_para_handle_p);
/*µç»úÆô¶¯*/
uint8_t Motor_Start(ST_motor_ctrl_p wheel_ctrl_handle_p,
                    ST_motor_ctrl_parameter_p wheel_para_handle_p, 
                    int32_t pwm);
/*Õý³£Í£Ö¹*/
uint8_t Motor_Stop(ST_motor_ctrl_p wheel_ctrl_handle_p,
                   ST_motor_ctrl_parameter_p wheel_para_handle_p);
/*½ô¼±ÖÆ¶¯*/
uint8_t Motor_Brake(ST_motor_ctrl_p wheel_ctrl_handle_p,
                    ST_motor_ctrl_parameter_p wheel_para_handle_p);
/*µç»ú×´Ì¬¶ÁÈ¡*/
uint8_t  Get_Motor_Station(ST_motor_ctrl_parameter_p wheel_para_handle_p,
                           uint16_t motor_station[2]);

/*µç»úÊä³öÊ¹ÄÜ*/
uint8_t Motor_Driver_Output_Enable (ST_motor_ctrl_p wheel_ctrl_handle_p,
                                    ST_motor_ctrl_parameter_p wheel_para_handle_p);
/*µç»úPWMÕ¼¿Õ±ÈÉèÖÃ*/
uint8_t Motor_Speed_Pwm_Duty_Cycle_Set(ST_motor_ctrl_p wheel_ctrl_handle_p,
                                       ST_motor_ctrl_parameter_p wheel_para_handle_p);
/*µç»ú×ªÏòÉèÖÃ*/
uint8_t Motor_Fwd_Rev_Select (ST_motor_ctrl_p wheel_ctrl_handle_p,
	                        ST_motor_ctrl_parameter_p wheel_para_handle_p);
/*µç»úÉ²³µÊ¹ÄÜ*/
uint8_t Motor_Brake_Enable (ST_motor_ctrl_p wheel_ctrl_handle_p,
	                      ST_motor_ctrl_parameter_p wheel_para_handle_p);
/*µç»ú¹¤×÷Ä£Ê½ÉèÖÃ*/
uint8_t Motor_Driver_Work_Mode (ST_motor_ctrl_p wheel_ctrl_handle_p,
	                          ST_motor_ctrl_parameter_p wheel_para_handle_p);

 //--×óµç»ú
/*×óµç»ú³õÊ¼»¯*/
uint8_t Left_Motor_Init(void);
/*×óµç»ú·´³õÊ¼»¯*/
uint8_t Left_Motor_Deinit(void);
/*×óµç»úÆô¶¯*/
uint8_t Left_Motor_Start(int32_t pwm);
/*×óÕý³£Í£Ö¹*/
uint8_t Left_Motor_Stop(void);
/*×ó½ô¼±ÖÆ¶¯*/
uint8_t Left_Motor_Brake(void);
/*»ñÈ¡×óµç»ú×´Ì¬*/
uint8_t  Get_Left_Motor_Station(uint16_t motor_station[2]);
//--ÓÒµç»ú
/*ÓÒµç»ú³õÊ¼»¯*/
uint8_t Right_Motor_Init(void);
/*ÓÒµç»ú·´³õÊ¼»¯*/
uint8_t Right_Motor_Deinit(void);
/*ÓÒµç»úÆô¶¯*/
uint8_t Right_Motor_Start(int32_t pwm);
/*ÓÒÕý³£Í£Ö¹*/
uint8_t Right_Motor_Stop(void);
/*ÓÒ½ô¼±ÖÆ¶¯*/
uint8_t Right_Motor_Brake(void);
/*»ñÈ¡ÓÒµç»ú×´Ì¬*/
uint8_t  Get_Right_Motor_Station(uint16_t motor_station[2]);
//--ÓÒ²à³µÂÖµç»ú¿ØÖÆ½á¹¹Ìå 
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
    &htim1,  //--PWMÊä³ö
    TIM_CHANNEL_4,
};
ST_motor_ctrl_p right_wheel_ctrl_handle_p = &right_wheel_ctrl_handle;
//-- ÓÒ²à³µÂÖµç»ú¿ØÖÆ½á¹¹Ìå 
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
    &htim1,  /*PWMÊä*/
    TIM_CHANNEL_3,
};
ST_motor_ctrl_p left_wheel_ctrl_handle_p = &left_wheel_ctrl_handle;
/*==============================================================================*/
/*ÓÒ²à³µÂÖµç»ú¿ØÖÆ²ÎÊý½á¹¹Ìå*/ 
ST_motor_ctrl_parameter   right_wheel_para_handle={
    output_disable,             /*µç»úÇý¶¯Æ÷Êä³öÊ¹ÄÜ±êÖ¾*/
    motor_fwd,                  /*µç»úÐý×ª·½Ïò±êÖ¾*/
  	brake_disable,              /*É²³µÊ¹ÄÜ±êÖ¾*/ 
	  work_mode_normal,         /*µç»úÇý¶¯Ð¾Æ¬Ñ¡Ôñ*/ 
    right_motor,	                /*µç»úÑ¡Ôñ*/
    0                           /*PWM*/
};
ST_motor_ctrl_parameter_p right_wheel_para_handle_p =&right_wheel_para_handle;

/*×ó²à³µÂÖµç»ú¿ØÖÆ²ÎÊý½á¹¹Ìå */ 
ST_motor_ctrl_parameter left_wheel_para_handle={
    output_disable,             //--µç»úÇý¶¯Æ÷Êä³öÊ¹ÄÜ±êÖ¾
    motor_rev,                  //--µç»úÐý×ª·½Ïò±êÖ¾
    brake_disable,              //--É²³µÊ¹ÄÜ±êÖ¾ 
	  work_mode_normal,         //--µç»úÇý¶¯Ð¾Æ¬Ñ¡Ôñ 
    left_motor,	
    0                           //--×ªËÙPWMÂö¿í 
};
ST_motor_ctrl_parameter_p left_wheel_para_handle_p =&left_wheel_para_handle;

/*==============================================================================*/
/*×óµç»úº¯Êý½Ó¿Ú*/
 ST_motor_handle   left_motor_handle = {
  Left_Motor_Init,
  Left_Motor_Deinit,
  Left_Motor_Start,
  Left_Motor_Stop,
  Left_Motor_Brake,
	Get_Left_Motor_Station
 };
 ST_motor_handle_p left_motor_handle_p = &left_motor_handle;
 /*ÓÒµç»úº¯Êý½Ó¿Ú*/
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
** Descriptions:        µç»ú³õÊ¼»¯ 
**                      ³õÊ¼»¯µç»ú·½Ïò£ºÄ¬ÈÏÐ¡³µÇ°½ø·½Ïò£¬×¢Òâ×óÓÒÁ½ÂÖ·½ÏòÏà·´
**                      ³õÊ¼»¯µç»úÄ£Ê½ £ºÉèÖÃµç»úÎªÕý³£¹¤×÷Ä£Ê½
** input parameters:    wheel_ctrl_handle_p£ºµç»ú¿ØÖÆ½á¹¹Ìå
**                      wheel_para_handle_p£ºµç»ú²ÎÊý½á¹¹Ìå
** Returned value:      0
***************************************************************************/
uint8_t Motor_Init(ST_motor_ctrl_p wheel_ctrl_handle_p,
                   ST_motor_ctrl_parameter_p wheel_para_handle_p)
{
  	/*ÓÒµç»úºÍ×óµç»ú·½ÏòÏà·´*/
	  if(wheel_para_handle_p->motor_switc == right_motor)
	  wheel_para_handle_p->fwd_rev_flag = motor_fwd;
	  if(wheel_para_handle_p->motor_switc == left_motor)
	  wheel_para_handle_p->fwd_rev_flag = motor_rev;
		Motor_Fwd_Rev_Select (wheel_ctrl_handle_p, wheel_para_handle_p);
	 /*µç»úÄ£Ê½Ñ¡Ôñ*/
		wheel_para_handle_p->motor_driver_work_mode = work_mode_normal;
	 Motor_Driver_Work_Mode (wheel_ctrl_handle_p, wheel_para_handle_p);
	return 0;
}
/***************************************************************************
** Function name:       Motor_Deinit
** Descriptions:        µç»ú·´³õÊ¼»¯ 
**                      ÉèÖÃµç»úÎªÎÞÊä³öÄ£Ê½
** input parameters:    wheel_ctrl_handle_p£ºµç»ú¿ØÖÆ½á¹¹Ìå
**                      wheel_para_handle_p£ºµç»ú²ÎÊý½á¹¹Ìå
** Returned value:      0
***************************************************************************/
uint8_t Motor_Deinit(ST_motor_ctrl_p wheel_ctrl_handle_p,
                     ST_motor_ctrl_parameter_p wheel_para_handle_p)
{
  	/*µç»úÄ£Ê½Ñ¡Ôñ*/
	wheel_para_handle_p->motor_driver_work_mode = work_mode_output_disable;
	Motor_Driver_Work_Mode (wheel_ctrl_handle_p, wheel_para_handle_p);
	return 0;
}
/***************************************************************************
** Function name:       Motor_Start
** Descriptions:        Æô¶¯µç»ú
**                      ¸ù¾ÝPWM²¨Õý¸ºÑ¡Ôñµç»úÐý×ª·½Ïò£¬×¢Òâ×óÓÒÁ½ÂÖ·½ÏòÊÇÏà·´µÄ
**                      ×¢Òâ£¬PWM¼ÆËãµÄÊ±ºòÓÐÕýÓÐ¸º£¬½øÐÐPWMÊä³öÉèÖÃµÄÊ±ºòÖ»ÓÐÕýÊý
**                      ÉèÖÃµç»úÄ£Ê½ÎªÕý³£¹¤×÷Ä£Ê½
** input parameters:    wheel_ctrl_handle_p£ºµç»ú¿ØÖÆ½á¹¹Ìå
**                      wheel_para_handle_p£ºµç»ú²ÎÊý½á¹¹Ìå
**                      PWM£ºµç»ú×ªËÙPWM²¨
** Returned value:      0
***************************************************************************/
uint8_t Motor_Start(ST_motor_ctrl_p wheel_ctrl_handle_p,
                    ST_motor_ctrl_parameter_p wheel_para_handle_p, 
                    int32_t pwm)
{
  if(pwm >=0)//--Ç°½ø
	{
	if(wheel_para_handle_p->motor_switc == right_motor)
	  wheel_para_handle_p->fwd_rev_flag = motor_fwd;
	  if(wheel_para_handle_p->motor_switc == left_motor)
	  wheel_para_handle_p->fwd_rev_flag = motor_rev;
	wheel_para_handle_p->pwm_value = pwm;
	}

  if(pwm <0)//--ºóÍË
	{
		if(wheel_para_handle_p->motor_switc == right_motor)
	  wheel_para_handle_p->fwd_rev_flag = motor_rev;
	  if(wheel_para_handle_p->motor_switc == left_motor)
	  wheel_para_handle_p->fwd_rev_flag = motor_fwd;
		wheel_para_handle_p->pwm_value = -pwm;
	}		
	
	 Motor_Fwd_Rev_Select (wheel_ctrl_handle_p,wheel_para_handle_p);
	  /*Õ¼¿Õ±ÈÉèÖÃ*/
	 Motor_Speed_Pwm_Duty_Cycle_Set(wheel_ctrl_handle_p,wheel_para_handle_p);
	 /*µç»úÄ£Ê½Ñ¡Ôñ*/
		wheel_para_handle_p->motor_driver_work_mode = work_mode_normal;
	 Motor_Driver_Work_Mode (wheel_ctrl_handle_p, wheel_para_handle_p);
 
	return 0;
}
/***************************************************************************
** Function name:       Motor_Stop
** Descriptions:        µç»úÕý³£Í£Ö¹£¬×¢Òâµç»úÔÚ´ËÄ£Ê½ÏÂ»áÒòÎª¹ßÐÔ½øÐÐ»¬ÐÐ
**                      ÐÞ¸Äµç»ú¹¤×÷Ä£Ê½
**                      ×¢Òâ Ò»¶¨ÒªÍ£Ö¹PWMÊä³ö£¬¶ø²»ÊÇ°ÑPWMÉèÖÃÎª0£¬·ñÔò»áµ¼ÖÂµç»ú
**                      ´ÓÐÂÆô¶¯µÄÊ±ºòÇ¿ÁÒ¶¶¶¯¡£
** input parameters:    wheel_ctrl_handle_p£ºµç»ú¿ØÖÆ½á¹¹Ìå
**                      wheel_para_handle_p£ºµç»ú²ÎÊý½á¹¹Ìå
**                      PWM£ºµç»ú×ªËÙPWM²¨
** Returned value:      0
***************************************************************************/
uint8_t Motor_Stop(ST_motor_ctrl_p wheel_ctrl_handle_p,
                   ST_motor_ctrl_parameter_p wheel_para_handle_p)
{
   /*µç»úÄ£Ê½Ñ¡Ôñ*/
	 wheel_para_handle_p->motor_driver_work_mode = work_mode_output_disable;
	 Motor_Driver_Work_Mode (wheel_ctrl_handle_p, wheel_para_handle_p); 
	/*¹Ø±ÕPWM*/
	  HAL_TIM_PWM_Stop( wheel_ctrl_handle_p->phtim_Speed_Adjust_PWM,  
	                   wheel_ctrl_handle_p->Channel_Speed_Adjust_PWM); 
	return 0;
}
/***************************************************************************
** Function name:       Motor_Brake
** Descriptions:        µç»ú½ô¼±ÖÆ¶¯£¬×¢Òâµç»úÔÚ´ËÄ£Ê½ÏÂ»á±§ËÀ
**                      ÐÞ¸Äµç»ú¹¤×÷Ä£Ê½
**                      ×¢Òâ Ò»¶¨ÒªÍ£Ö¹PWMÊä³ö£¬¶ø²»ÊÇ°ÑPWMÉèÖÃÎª0£¬·ñÔò»áµ¼ÖÂµç»ú
**                      ´ÓÐÂÆô¶¯µÄÊ±ºòÇ¿ÁÒ¶¶¶¯¡£
** input parameters:    wheel_ctrl_handle_p£ºµç»ú¿ØÖÆ½á¹¹Ìå
**                      wheel_para_handle_p£ºµç»ú²ÎÊý½á¹¹Ìå
**                      PWM£ºµç»ú×ªËÙPWM²¨
** Returned value:      0
***************************************************************************/
/*½ô¼±ÖÆ¶¯*/
uint8_t Motor_Brake(ST_motor_ctrl_p wheel_ctrl_handle_p,
                   ST_motor_ctrl_parameter_p wheel_para_handle_p)
{
   /*µç»úÄ£Ê½Ñ¡Ôñ*/
	 wheel_para_handle_p->motor_driver_work_mode = work_mode_brake_hard;
	 Motor_Driver_Work_Mode (wheel_ctrl_handle_p, wheel_para_handle_p); 
	/*¹Ø±ÕPWM*/
	  HAL_TIM_PWM_Stop( wheel_ctrl_handle_p->phtim_Speed_Adjust_PWM,  
	                   wheel_ctrl_handle_p->Channel_Speed_Adjust_PWM); 
	return 0;
}

/***************************************************************************
** Function name:       Motor_Driver_Output_Enable
** Descriptions:        µç»úÇý¶¯Æ÷Êä³öÊ¹ÄÜ¿ØÖÆ  
** input parameters:    wheel_ctrl_handle_p£ºµç»ú¿ØÖÆ½á¹¹Ìå
**                      wheel_para_handle_p£ºµç»ú²ÎÊý½á¹¹Ìå
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
** Descriptions:        ×ªËÙÕ¼¿Õ±Èµ÷½Ú
** input parameters:    wheel_ctrl_handle_p£ºµç»ú¿ØÖÆ½á¹¹Ìå
**                      wheel_para_handle_p£ºµç»ú²ÎÊý½á¹¹Ìå
**                       ÎÞ·ÖÆµ 72MHZ  Ã¿¸öPWMÖÜÆÚ¼ÆÊý2500 
**                       Âö³å¿í¶È 0-2500
** Returned value:      0
***************************************************************************/
uint8_t Motor_Speed_Pwm_Duty_Cycle_Set(ST_motor_ctrl_p wheel_ctrl_handle_p,
                                       ST_motor_ctrl_parameter_p wheel_para_handle_p)
{

  TIM_OC_InitTypeDef sConfigOC;

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = wheel_para_handle_p->pwm_value;/*Õ¼¿Õ±ÈÉèÖÃ*/
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
** Descriptions:        µç»úÕý·´×ªÑ¡Ôñ
** input parameters:    wheel_ctrl_handle_p£ºµç»ú¿ØÖÆ½á¹¹Ìå
**                      wheel_para_handle_p£ºµç»ú²ÎÊý½á¹¹Ìå 
** Returned value:      0:Õý³£
                        MOTOR_FWD_REV_SELECT_DEFAULT£º³ö´í
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
** Descriptions:        µç»úÇý¶¯Æ÷ÖÆ¶¯ÊäÈë
** input parameters:    wheel_ctrl_handle_p£ºµç»ú¿ØÖÆ½á¹¹Ìå
**                      wheel_para_handle_p£ºµç»ú²ÎÊý½á¹¹ÌåÜ
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
** Descriptions:        »ñÈ¡µç»ú×ªËÙ½âÂëÆ÷¼ÆÊýÖµ   
** input parameters:    wheel_ctrl_handle_p£ºµç»ú¿ØÖÆ½á¹¹Ìå
**                      wheel_para_handle_p£ºµç»ú²ÎÊý½á¹¹Ìå
** Returned value:      µ±Ç°µç»ú×ªËÙ½âÂëÆ÷¼ÆÊýÖµ
***************************************************************************/
//float Motor_Speed_Cnt_Get (ST_motor_ctrl_p wheel_ctrl_handle_p,)
//{  

//   return __HAL_TIM_GET_COUNTER(wheel_ctrl_handle_p->phtim_DecoderCounter);
//}
/***************************************************************************
** Function name:       Motor_Speed_Cnt_Set  
** Descriptions:        ÉèÖÃµç»ú×ªËÙ½âÂëÆ÷¼ÆÊýÖµ   
** input parameters:    wheel_ctrl_handle_p£ºµç»ú¿ØÖÆ½á¹¹Ìå
**                      wheel_para_handle_p£ºµç»ú²ÎÊý½á¹¹Ìå
**                      cnt             £ºÒªÉèÖÃµÄ¼ÆÊýÆ÷µÄÖµ 0-2500
** Returned value:      MOTOR_SWITCH_DEFAULT
***************************************************************************/

//__INLINE int Motor_Speed_Cnt_Set (ST_motor_ctrl_p wheel_ctrl_handle_p, uint16_t cnt)
//{

//    __HAL_TIM_SET_COUNTER(wheel_ctrl_handle_p->phtim_DecoderCounter,cnt);
//		 return 0;
//}

/***************************************************************************
** Function name:       Motor_Driver_Work_Mode  
** Descriptions:        ÉèÖÃµç»úÇý¶¯Ð¾Æ¬µÄ¹¤×÷Ä£Ê½   
** input parameters:    wheel_ctrl_handle_p£ºµç»ú¿ØÖÆ½á¹¹Ìå
**                      wheel_para_handle_p£ºµç»ú²ÎÊý½á¹¹Ìå
** Returned value:      0
***************************************************************************/
uint8_t Motor_Driver_Work_Mode (ST_motor_ctrl_p wheel_ctrl_handle_p,
                                ST_motor_ctrl_parameter_p wheel_para_handle_p)
{   
		//--Ä£Ê½Ñ¡Ôñ 
    if(wheel_para_handle_p->motor_driver_work_mode == work_mode_brake_hard) {
        /* ¶ÔÓÚÐèÒª½ô¼±É²³µµÄ³¡ºÏ£¬Ó¦ÒÔ×î¿ìËÙ¶È¿ØÖÆµç»úÖ´ÐÐÉ²³µ¶¯×÷ */
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
** Descriptions:        »ñÈ¡µç»ú×´Ì¬ 
** input parameters:    motor_switc£ºµç»úÑ¡Ôñ£¬¿ÉÒÔÈ¡Ò»ÏÂÖµ£º
**                                  LEFT_MOTOR £º ×ó²àµç»ú
**                                  RIGHT_MOTOR£º ÓÒ²àµç»ú
**                                  MOWING_MOTOR£º¸î²Ýµç»ú
**                      motor_station[0]
**                      xxxx xxxx
**                      bit 0  : 0-Êä³ö½ûÄÜ 1-Êä³öÊ¹ÄÜ
**                      bit 1  : 0-Ç°½ø     1-ºóÍË
**                      bit 2  : 0-É²³µ½ûÄÜ 1-É²³µÊ¹ÄÜ
**                      bit 3-4: 00-Õý³£Ä£Ê½ 01-ÎÞÊä³öÄ£Ê½ 10-½ô¼±É²³µÄ£Ê½
**                      bit 5-15 : ±£Áô
**                      motor_station[1]
**                      µç»ú×ªËÙÕ¼¿Õ±È
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
** Descriptions:        ×óµç»ú³õÊ¼»¯£ºµ÷ÓÃµç»ú³õÊ¼»¯·½·¨º¯Êý
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Left_Motor_Init(void)
{
	return Motor_Init(left_wheel_ctrl_handle_p,left_wheel_para_handle_p);
}
/***************************************************************************
** Function name:       Left_Motor_Deinit
** Descriptions:        ×óµç»ú·´³õÊ¼»¯£ºµ÷ÓÃµç»ú·´³õÊ¼»¯·½·¨º¯Êý
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Left_Motor_Deinit(void)
{
	return Motor_Deinit(left_wheel_ctrl_handle_p,left_wheel_para_handle_p);
}
/***************************************************************************
** Function name:       Left_Motor_Start
** Descriptions:        ×óµç»úÆô¶¯£ºµ÷ÓÃµç»úÆô¶¯·½·¨º¯Êý
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Left_Motor_Start(int32_t pwm)
{
	return Motor_Start(left_wheel_ctrl_handle_p,left_wheel_para_handle_p,pwm);
}
/***************************************************************************
** Function name:       Left_Motor_Stop
** Descriptions:        ×óµç»úÕý³£Í£Ö¹£ºµ÷ÓÃµç»úÕý³£Í£Ö¹·½·¨º¯Êý
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Left_Motor_Stop(void)
{
	return Motor_Stop(left_wheel_ctrl_handle_p,left_wheel_para_handle_p);
}
/***************************************************************************
** Function name:       Left_Motor_Brake
** Descriptions:        ×óµç»ú½ô¼±ÖÆ¶¯£ºµ÷ÓÃµç»ú½ô¼±ÖÆ¶¯·½·¨º¯Êý
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Left_Motor_Brake(void)
{ 
	return Motor_Brake(left_wheel_ctrl_handle_p,left_wheel_para_handle_p);
}
/***************************************************************************
** Function name:       Right_Motor_Init
** Descriptions:        ÓÒµç»ú³õÊ¼»¯£ºµ÷ÓÃµç»ú³õÊ¼»¯·½·¨º¯Êý
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Right_Motor_Init(void)
{
	return Motor_Init(right_wheel_ctrl_handle_p,right_wheel_para_handle_p);
}
/***************************************************************************
** Function name:       Right_Motor_Deinit
** Descriptions:        ÓÒµç»ú·´³õÊ¼»¯£ºµ÷ÓÃµç»ú·´³õÊ¼»¯·½·¨º¯Êý
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Right_Motor_Deinit(void)
{
 return Motor_Deinit(right_wheel_ctrl_handle_p,right_wheel_para_handle_p);
}
/***************************************************************************
** Function name:       Right_Motor_Start
** Descriptions:        ÓÒµç»úÆô¶¯£ºµ÷ÓÃµç»úÆô¶¯·½·¨º¯Êý
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Right_Motor_Start(int32_t pwm)
{
 return Motor_Start(right_wheel_ctrl_handle_p,right_wheel_para_handle_p,pwm);
}
/***************************************************************************
** Function name:       Right_Motor_Stop
** Descriptions:        ÓÒµç»úÕý³£Í£Ö¹£ºµ÷ÓÃµç»úÕý³£Í£Ö¹·½·¨º¯Êý
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Right_Motor_Stop(void)
{
 return Motor_Stop(right_wheel_ctrl_handle_p,right_wheel_para_handle_p);
}
/***************************************************************************
** Function name:       Right_Motor_Brake
** Descriptions:        ÓÒµç»ú½ô¼±ÖÆ¶¯£ºµ÷ÓÃµç»ú½ô¼±ÖÆ¶¯·½·¨º¯Êý
** input parameters:    void
** Returned value:      0
***************************************************************************/
uint8_t Right_Motor_Brake(void)
{
 return Motor_Brake(right_wheel_ctrl_handle_p,right_wheel_para_handle_p);
}
/***************************************************************************
** Function name:       Get_Right_Motor_Station
** Descriptions:        »ñÈ¡ÓÒµç»ú×´Ì¬£¬µ÷ÓÃµç»ú×´Ì¬·½·¨º¯Êý
** input parameters:    motor_station[2]£º×´Ì¬Êý×é
** Returned value:      0
***************************************************************************/
uint8_t  Get_Right_Motor_Station(uint16_t motor_station[2])
{
return  Get_Motor_Station(right_wheel_para_handle_p,motor_station);
}
/***************************************************************************
** Function name:       Get_Left_Motor_Station
** Descriptions:        »ñÈ¡×óµç»ú×´Ì¬£¬µ÷ÓÃµç»ú×´Ì¬·½·¨º¯Êý
** input parameters:    motor_station[2]£º×´Ì¬Êý×é
** Returned value:      0
***************************************************************************/
uint8_t  Get_Left_Motor_Station(uint16_t motor_station[2])
{
return  Get_Motor_Station(left_wheel_para_handle_p,motor_station);
}
