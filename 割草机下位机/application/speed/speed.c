/*
  ******************************************************************************
  * @file    speed.c
  * @author  xiaoyuer
  * @version V0.0.1
  * @date    2019/7/16
  * @brief   ٤���Ի����˵��T������
  ******************************************************************************        
*/

#include "speed.h"

/*תһȦ������ֵ����*/
uint32_t  Counting_Difference(ST_speed_variable_p speed_variable_p);
/*�ٶȸ��£�mm/s*/
uint8_t Refresh_Speed(ST_speed_variable_p speed_variable_p,
                      ST_speed_ctrl_p speed_ctrl_p);
/*��ȡ�ٶ�*/
int32_t Read_Speed(ST_speed_variable_p speed_variable_p);
/*�ٶȳ�ʼ��*/
uint8_t Speed_Init(ST_speed_ctrl_p speed_ctrl_p);
/*�ٶȷ���ʼ��*/
uint8_t Speed_Deinit(ST_speed_ctrl_p speed_ctrl_p);
/*��ȡ�������ֵ*/
uint32_t Get_Motor_Cnt(ST_speed_variable_p speed_variable_p);
/*�����������ֵ*/
uint8_t Motor_Cnt_Reset(ST_speed_variable_p speed_variable_p);
//--����
/*תһȦ������ֵ����*/
uint32_t  Left_Motor_Counting_Difference(void);
/*�ٶȸ��£�mm/s*/
uint8_t Left_Motor_Refresh_Speed(void);
/*��ȡ�ٶ�*/
int32_t Left_Motor_Read_Speed(void);
/*�ٶȳ�ʼ��*/
uint8_t Left_Motor_Speed_Init(void);
/*�ٶȷ���ʼ��*/
uint8_t Left_Motor_Speed_Deinit(void);
/*��ȡ�������ֵ*/
uint32_t Left_Motor_Get_Motor_Cnt(void);
/*�����������ֵ*/
uint8_t Left_Motor_Motor_Cnt_Reset(void);
//--�ҵ��
/*תһȦ������ֵ����*/
uint32_t  Right_Motor_Counting_Difference(void);
/*�ٶȸ��£�mm/s*/
uint8_t Right_Motor_Refresh_Speed(void);
/*��ȡ�ٶ�*/
int32_t Right_Motor_Read_Speed(void);
/*�ٶȳ�ʼ��*/
uint8_t Right_Motor_Speed_Init(void);
/*�ٶȷ���ʼ��*/
uint8_t Right_Motor_Speed_Deinit(void);
/*��ȡ�������ֵ*/
uint32_t Right_Motor_Get_Motor_Cnt(void);
/*�����������ֵ*/
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
 EXTI15_10_IRQn, /*�ⲿ�ж϶˿ں�*/
 &htim5  /*������*/
};
ST_speed_ctrl_p right_speed_ctrl_p = &right_speed_ctrl;

ST_speed_ctrl left_speed_ctrl = {
 EXTI2_IRQn,/*�ⲿ�ж϶˿ں�*/
 &htim5  /*������*/
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
//--�ٶȽӿں���
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
** Descriptions:        �ⲿ�����������жϴ�����
** input parameters:    GPIO_Pin���ж�����
** Returned value:      ��
***************************************************************************/
//--�жϴ�����
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
	if(GPIO_Pin == R_MOTOR_EXTI15_Pin)
      { 
       /*��ȡTIM5����ֵ*/
				
       *(r_motor_begin_p++) = __HAL_TIM_GET_COUNTER(&htim5);
			 /*ѭ������*/
	      if(r_motor_begin_p == r_motor_end_p){
				    r_motor_begin_p = r_motor;	
			  }
				if(right_speed_variable_p->HAL_CNT == 65535)
				++right_speed_variable_p->overflow_count;
				++right_speed_variable_p->HAL_CNT;/*������������Ϊ��̼Ʒ���*/
				/*�������һλ��¼������,ֻΪ�����ٶ�ʹ��*/
				++(*r_motor_end_p);/*����������*/
      }	
    if (GPIO_Pin == L_MOTOR_EXTI2_Pin)
     {		
       /*��ȡTIM5����ֵ*/
       *(l_motor_begin_p++) = __HAL_TIM_GET_COUNTER(&htim5);
		    /*ѭ������*/
		    if(l_motor_begin_p == l_motor_end_p){
				   l_motor_begin_p = l_motor;
				 }	
			 	if(left_speed_variable_p->HAL_CNT == 65535)
				++left_speed_variable_p->overflow_count;
				++left_speed_variable_p->HAL_CNT;/*������������Ϊ��̼Ʒ���*/
				/*�������һλ��¼������*/
				++(*l_motor_end_p);/*����������*/
	   }
 }
/***************************************************************************
** Function name:       Counting_Difference
** Descriptions:        ������������ֵ
** input parameters:    speed_variable_p����������ṹ��
** Returned value:      motor_cnt1��������ֵ
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
	 /*��ȡ���״̬*/
	 if(speed_variable_p == left_speed_variable_p)
	 left_motor_handle_p->get_motor_station(read_motor);
	 if(speed_variable_p == right_speed_variable_p)
	 right_motor_handle_p->get_motor_station(read_motor);
	 //---------------------------------------------------------------------//
	  if(read_motor[1]>=1000)/*���PWM��1000,������ֵ�˲�*/
	  {
		 cnt = 9;/*100ms��3���ٶ�*/
			speed_variable_p->smoothing_en = 1;
		}
		else
		{
		cnt = 3;/*100ms��һ���ٶ�*/
			speed_variable_p->smoothing_en = 0;
		}
		//---------------------------------------------------------------------//
		if(speed_variable_p == left_speed_variable_p)
		{
			 /*ѭ���洢ʱ��ָ���+1������-1�����������Ч����*/
			 if(l_motor_begin_p == l_motor)/*�����ǰָ��ָ���׵�ַ*/
				  motor_p = &l_motor[SPEED_SIZE-2];/*��Ч������β��ַ*/
			 else motor_p = l_motor_begin_p-1;/*���浱ǰ��Ч��ַָ��*/
	    for(i=0;i<cnt;i++)
			 {
			 if(motor_p == l_motor) /*���ָ�����׵�ַ��ָ���8λ*/
				 {
				  motor_p = &l_motor[SPEED_SIZE-2];
					 if(*l_motor <= l_motor[SPEED_SIZE-2])/*���������*/
				  {
						motor_cnt = *l_motor + (65536-l_motor[SPEED_SIZE-2]);
						goto left_motor_end;
					}
				  motor_cnt = *l_motor - *motor_p;/*�ۼ������������ֵ*/
					goto left_motor_end;
				 }
				 motor_p--;
				 if(*(motor_p+1) <= *motor_p)/*���������*/
				  {
						motor_cnt = *(motor_p+1) + (65536-*motor_p);
					  goto left_motor_end;
					}	
				motor_cnt = *(motor_p+1) - *motor_p;/*�ۼ������������ֵ*/
				left_motor_end:
				motor_cnt1+=motor_cnt;
				}
			}
		//-------------------------------------------------------------------------//
	if(speed_variable_p == right_speed_variable_p)
		{
			 /*ѭ���洢ʱ��ָ���+1������-1�����������Ч����*/
			 if(r_motor_begin_p == r_motor)/*�����ǰָ��ָ���׵�ַ*/
				  motor_p = &r_motor[SPEED_SIZE-2];/*��Ч������β��ַ*/
			 else motor_p = r_motor_begin_p-1;/*���浱ǰ��Ч��ַָ��*/
	    for(i=0;i<cnt;i++)
			 {
			 if(motor_p == r_motor) /*���ָ�����׵�ַ��ָ���8λ*/
				 {
				  motor_p = &r_motor[SPEED_SIZE-2];
					 if(*r_motor <= r_motor[SPEED_SIZE-2])/*���������*/
				  {
						motor_cnt = *r_motor + (65536-r_motor[SPEED_SIZE-2]);
						goto right_motor_end;
					}
				  motor_cnt = *r_motor - *motor_p;/*�ۼ������������ֵ*/
					goto right_motor_end;
				 }
				 motor_p--;
				 if(*(motor_p+1) <= *motor_p)/*���������*/
				  {
						motor_cnt = *(motor_p+1) + (65536-*motor_p);
					  goto right_motor_end;
					}	
				motor_cnt = *(motor_p+1) - *motor_p;/*�ۼ������������ֵ*/
				right_motor_end:
				motor_cnt1+=motor_cnt;
				}
			}
	return motor_cnt1;
 }

/***************************************************************************
** Function name:       Refresh_Speed
** Descriptions:        �����ٶ�
** input parameters:    speed_variable_p������ٶȲ����ṹ��
**                      speed_ctrl_p������ٶȿ��ƽṹ��
** Returned value:      0
***************************************************************************/
uint8_t Refresh_Speed(ST_speed_variable_p speed_variable_p,ST_speed_ctrl_p speed_ctrl_p)
{	
   
	uint32_t cnt = 0;
	int32_t speed = 0;
	uint16_t read_motor[2];
	static int8_t r_cnt =0,l_cnt = 0;

	//-----------------------------------------------------
	/*���������������ٶȼ����������*/
	/*����ڳ�ʼ����ʱ����ⲿ�жϣ��ڵ��øú���֮ǰ�����ʱ����ν����ж�*/
	/*�жϼ���ֵ���ۼӣ����³��μ����ٶ����ݴ���*/
   if(speed_variable_p->first_electrification_flag == 1)/*����ǳ����ϵ�*/
	 {
	   speed_variable_p->first_electrification_flag = 0;
		 HAL_NVIC_EnableIRQ(speed_ctrl_p->external_interrupts);/*���ⲿ�ж�*/
	 return 0;
	 }
	 //----------------------------------------------------
	 if(speed_variable_p == left_speed_variable_p)
	 { 	
		 left_motor_handle_p->get_motor_station(read_motor);/*��ȡ���״̬*/
		//--------------------------------------------------- 	 
		 /*���100ms��⵽��������С��4�����β������ٶ�*/
		 /*���������ٶ�������*/
		 if(*l_motor_end_p<4) 
		 { 
			 //---------------------------------------------
			 l_cnt ++; 
			 if(l_cnt >=2)/*�����������ζ�δ�����Ч���ݣ��ٶ�����*/
			 {
			l_cnt = 0;
			speed_variable_p->motor_speed = 0;
			 }
			 //---------------------------------------------
			 memset(l_motor,0,SPEED_SIZE-1);//--���һλ������
			 l_motor_begin_p = &l_motor[0];
			
			 return 0;
		 }
		 else *l_motor_end_p = 0;
		 //-----------------------------------------------------
		 		l_cnt --;
		 if(l_cnt <=0)l_cnt = 0;
		//----------------------------------------------------- 
	  /*�ر��ⲿ�ж�*/
     HAL_NVIC_DisableIRQ(speed_ctrl_p->external_interrupts); 
		//-------------------------------------------------------------//
		 cnt = Counting_Difference(speed_variable_p);/*��ȡ������ֵ*/
		 if(speed_variable_p->smoothing_en == 1)
		 {
		 cnt = cnt /3;
		 }
		/*���ⲿ�ж�*/
		 HAL_NVIC_EnableIRQ(speed_ctrl_p->external_interrupts);
		 /*�����ٶ�*/ 
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
		right_motor_handle_p->get_motor_station(read_motor);/*��ȡ���״̬*/
		 /*���10ms��⵽��������С��2�����β������ٶ�*/
		 /*���������ٶ�������*/
		if(*r_motor_end_p<4) 
		 {	
			 //-------------------------------------------------------
			 r_cnt ++; 
			 if(r_cnt >=2)/*�����������ζ�δ�����Ч���ݣ��ٶ�����*/
			 {
			r_cnt = 0;
			speed_variable_p->motor_speed = 0;
			 }
				 //---------------------------------------------
			 memset(r_motor,0,SPEED_SIZE-1);//--���һλ������
			 r_motor_begin_p = &r_motor[0];
			 return 0;
		 }
     else *r_motor_end_p = 0;
		 //-----------------------------------------------
		 r_cnt --;
		 if(r_cnt <=0)r_cnt = 0;
		//----------------------------------------------------- 
	 /*�ر��ⲿ�ж�*/
    HAL_NVIC_DisableIRQ(speed_ctrl_p->external_interrupts);
		//-------------------------------------------------------------/
		cnt = Counting_Difference(speed_variable_p);/*��ȡ������ֵ*/
		 if(speed_variable_p->smoothing_en == 1)
		 {
		 cnt = cnt /3;
		 }
	
		/*���ⲿ�ж�*/
	 HAL_NVIC_EnableIRQ(speed_ctrl_p->external_interrupts);
		 /*============�����ٶ�=================*/	
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

	*l_motor_end_p = 0;/*ʲôû��ҲҪ������β����������ֵ*/
	 return 0;
}
/***************************************************************************
** Function name:       Read_Speed
** Descriptions:        �ٶȶ�ȡ
** input parameters:    speed_variable_p������ٶȲ����ṹ��
** Returned value:      speed���ٶ�
***************************************************************************/
int32_t Read_Speed(ST_speed_variable_p speed_variable_p)
{
 int32_t speed = 0;
  speed = speed_variable_p->motor_speed;
	return speed;
}
/***************************************************************************
** Function name:       Speed_Init
** Descriptions:        �ر��ⲿ�жϣ�����������
**input parameters:     speed_ctrl_p ��������Ʋ����ṹ��
** Returned value:      ��
***************************************************************************/
uint8_t Speed_Init(ST_speed_ctrl_p speed_ctrl_p)
{ 
		/*�ر��ⲿ�ж�*/
    HAL_NVIC_DisableIRQ(speed_ctrl_p->external_interrupts);
	  /*����������*/
	  __HAL_TIM_SET_COUNTER(speed_ctrl_p->phtim_Speed_cnt,0);
	  /*����������*/
    HAL_TIM_Base_Start(speed_ctrl_p->phtim_Speed_cnt); 
	 /*���ⲿ�ж�*/
	 //HAL_NVIC_EnableIRQ(speed_ctrl_p->external_interrupts);
	return 0;
}
/***************************************************************************
** Function name:       Speed_Deinit
** Descriptions:        �ر��ⲿ�жϣ����������㣬�رռ�����
**input parameters:     speed_ctrl_p ��������Ʋ����ṹ��
** Returned value:      ��
***************************************************************************/
uint8_t Speed_Deinit(ST_speed_ctrl_p speed_ctrl_p)
{ 
		/*�ر��ⲿ�ж�*/
    HAL_NVIC_DisableIRQ(speed_ctrl_p->external_interrupts);
	  /*����������*/
	  __HAL_TIM_SET_COUNTER(speed_ctrl_p->phtim_Speed_cnt,0);
	  /*�رռ�����*/
    HAL_TIM_Base_Stop(&htim5); 
	//===========���ⲿ�ж�===============//
	 //HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	 //HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	return 0;
}
/***************************************************************************
** Function name:       Get_Motor_Cnt
** Descriptions:        ������������
** input parameters:    speed_variable_p������ٶȲ����ṹ��
** Returned value:      ���ص��������
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

//--����
/*תһȦ������ֵ����*/
uint32_t  Left_Motor_Counting_Difference(void)
{
return Counting_Difference(left_speed_variable_p);
}
/*�ٶȸ��£�mm/s*/
uint8_t Left_Motor_Refresh_Speed(void)
{
return Refresh_Speed(left_speed_variable_p,left_speed_ctrl_p);
}
/*��ȡ�ٶ�*/
int32_t Left_Motor_Read_Speed(void)
{
return Read_Speed(left_speed_variable_p);
}
/*�ٶȳ�ʼ��*/
uint8_t Left_Motor_Speed_Init(void)
{
return Speed_Init(left_speed_ctrl_p);
}
/*�ٶȷ���ʼ��*/
uint8_t Left_Motor_Speed_Deinit(void)
{
return Speed_Deinit(left_speed_ctrl_p);
}
/*��ȡ�������ֵ*/
uint32_t Left_Motor_Get_Motor_Cnt(void)
{
return Get_Motor_Cnt(left_speed_variable_p);
}
/*�����������ֵ*/
uint8_t Left_Motor_Motor_Cnt_Reset(void)
{
return Motor_Cnt_Reset(left_speed_variable_p);
}

//--�ҵ��
/*תһȦ������ֵ����*/
uint32_t  Right_Motor_Counting_Difference(void)
{
return Counting_Difference(right_speed_variable_p);
}
/*�ٶȸ��£�mm/s*/
uint8_t Right_Motor_Refresh_Speed(void)
{
return Refresh_Speed(right_speed_variable_p,right_speed_ctrl_p);
}
/*��ȡ�ٶ�*/
int32_t Right_Motor_Read_Speed(void)
{
return Read_Speed(right_speed_variable_p);
}
/*�ٶȳ�ʼ��*/
uint8_t Right_Motor_Speed_Init(void)
{
return Speed_Init(right_speed_ctrl_p);
}
/*�ٶȷ���ʼ��*/
uint8_t Right_Motor_Speed_Deinit(void)
{
return Speed_Deinit(right_speed_ctrl_p);
}
/*��ȡ�������ֵ*/
uint32_t Right_Motor_Get_Motor_Cnt(void)
{
return Get_Motor_Cnt(right_speed_variable_p);
}
/*�����������ֵ*/
uint8_t Right_Motor_Motor_Cnt_Reset(void)
{
return Motor_Cnt_Reset(right_speed_variable_p);
}

