/**
  ******************************************************************************
  * @file    car.c
  * @author  С�����Ѿ��
  * @version V0.0.1
  * @date    2019/7/30
  * @brief   ٤���Ի�����С�����̿���
  ******************************************************************************/
	#include "car.h"
  #include "stdio.h"
	#include "math.h"
	#include "bno005.h"
  #include "uppercmd.h"
  #include "uart3_rcv_queue.h"
	#include "uppercmd.h"
	  /*С����ʼ��*/
static	uint8_t Car_Init(void);
	/*С������ʼ��*/
static	uint8_t Car_Deinit(void);
	/*С���ٶȸ���*/
static	uint8_t Car_Speed_Updata(void);
	/*С������ֹͣ*/
static	uint8_t Car_Stop(void);
	/*С������ֹͣ*/
static	uint8_t Car_Brake(void);
/*������λ������*/
static int8_t Car_Walk_Straight_Line_Updata(int32_t v,uint32_t s);
/*������λ������*/
static int8_t Car_Turn_Updata(int32_t V,float W,float A);
/*С��ң����ֱ��*/
static int8_t Car_Remote_Line(void);
/*С��ң��ת��*/
static int8_t Car_Remote_Turn(void);
/*С����ֱ��*/
static	int8_t Car_Walk_Straight_Line(void);
/*С��ԭ��ת��*/
/*С����ĳ������ǰ�����ߺ���*/
static uint8_t Car_Turn(void);
/*��ת����*/
/*��ȡС�����ٶ�*/
static int32_t Read_Speed(void);
/*��ȡС�����ٶ�*/
/*��ȡС����ʻ����*/
static uint32_t Read_Car_Distance(void);
 /*��ȡС��״̬����*/
static	uint8_t Read_Car_Start_Stop(void);
/*С����������*/
static uint8_t Car_Variable_Reset(void);
/*����С��ֱ�����ߵĽǶ����*/
static uint32_t Car_Walk_Straight_Line_Calculate_Angle_Difference(uint16_t range,int32_t naw_angle , int32_t last_angle);
/*����С��ֱ�������������ٶ�*/
static int32_t* Car_Walk_Straight_Line_Calculate_Wheel_Speed(int32_t reference_value,int32_t adjusted_value);
/*����ͣ��*/
static uint32_t Slow_Down_And_Stop(void);
/*����ת��ǶȺ�*/
static int32_t Car_Turn_Calculate_Angle_Difference(int32_t w,int32_t naw_angle , int32_t last_angle);
/*��ȡС����ͣ״̬*/
static uint8_t Get_Car_Stop_or_Start(void);
/*��8��*/
	static uint8_t Car_Turn_8(void);
	static int8_t Car_Turn_8_Updata(int32_t V,float W,int Q);

  ST_car_variable  car_variable = {0};
	ST_car_variable_p  car_variable_p = &car_variable;
	ST_car_function_interface_handle   car_handle={
	  /*С����ʼ��*/
  Car_Init,
	/*С������ʼ��*/
  Car_Deinit,
	/*С���ٶȸ���*/
  Car_Speed_Updata,
	/*С������ֹͣ*/
  Car_Stop,
	/*С������ֹͣ*/
  Car_Brake,
	/*������λ������*/
  Car_Walk_Straight_Line_Updata,
  /*������λ������*/
   Car_Turn_Updata,
	/*С��ң����ֱ��*/
    Car_Remote_Line,
/*С��ң��ת��*/
    Car_Remote_Turn,
	/*С����ֱ��*/
  Car_Walk_Straight_Line,
	/*С��ԭ��ת��*/
	/*С����ĳ������ǰ�����ߺ���*/
	Car_Turn,
	/*��ת����*/
	/*��ȡС�����ٶ�*/
  Read_Speed,
	/*��ȡС�����ٶ�*/
	/*��ȡС����ʻ����*/
  Read_Car_Distance,
	/*��ȡС��״̬����*/	
	Read_Car_Start_Stop,
	/*С����������*/
  Car_Variable_Reset,
	/*����С��ֱ�����ߵĽǶ����*/
	Car_Walk_Straight_Line_Calculate_Angle_Difference,
	/*����С��ֱ�������������ٶ�*/
	Car_Walk_Straight_Line_Calculate_Wheel_Speed,
	/*����ͣ��*/
	Slow_Down_And_Stop,
	/*����ת��ǶȺ�*/
	Car_Turn_Calculate_Angle_Difference,
	/*��ȡС����ͣ״̬*/
	Get_Car_Stop_or_Start,
	Car_Turn_8,
	 /*������λ������*/
Car_Turn_8_Updata,
	};
  ST_car_function_interface_handle_p car_handle_p =&car_handle;


  /*С����ʼ��*/
static	uint8_t Car_Init(void)
	{
	 /*���ֳ�ʼ��*/
		left_wheel_handle_p->init();
	/*���ֳ�ʼ��*/
		right_wheel_handle_p->init();
		return 0;
	}
	/*С������ʼ��*/
static	uint8_t Car_Deinit(void)
	{
	 /*���ַ���ʼ��*/
		left_wheel_handle_p->deinit();
	/*���ַ���ʼ��*/
		right_wheel_handle_p->deinit();
		return 0;
	}
	/*С���ٶȸ���*/
static	uint8_t Car_Speed_Updata(void)
	{
	 /*�����ٶȸ���*/
	int32_t rv = right_wheel_handle_p->speed_updata();
	 /*�����ٶȸ���*/
	int32_t lv = left_wheel_handle_p->speed_updata();
	 /*����С������ͬ�������ٶ�*/
	 car_variable_p->now_car_speed = (left_wheel_handle_p->read_speed()
		                               +right_wheel_handle_p->read_speed())/2;
		return 0;
	}
	/*С������ֹͣ*/
static	uint8_t Car_Stop(void)
	{
	/*����С��״̬*/
	car_variable_p->car_start_stop = CAR_STOP;//--ֹͣ
	 /*����ֹͣ*/
	right_wheel_handle_p->stop();
	/*����ֹͣ*/
	left_wheel_handle_p->stop();
		/*PID�м��������*/
	left_pid_handle_p->pwm_value_reset();
	right_pid_handle_p->pwm_value_reset();
	car_pid_handle_p->car_walk_distance_pid_value_reset();
	car_pid_handle_p->car_walk_yaw_pid_value_reset();
	/*С���м��������*/
  car_handle_p->car_variable_reset();
	upper_cmd_header_p->set_cmd_type(E_UPPER_CMD_STOP);
		return 0;
	}
	/*С������ֹͣ*/
static	uint8_t Car_Brake(void)
	{
	/*����С��״̬*/
	car_variable_p->car_start_stop = CAR_STOP;//--ֹͣ
	 /*����ֹͣ*/
	right_wheel_handle_p->brake();
	/*����ֹͣ*/
	left_wheel_handle_p->brake();
	/*PID�м��������*/
	left_pid_handle_p->pwm_value_reset();
	right_pid_handle_p->pwm_value_reset();
	car_pid_handle_p->car_walk_distance_pid_value_reset();
	car_pid_handle_p->car_walk_yaw_pid_value_reset();		
	/*С���м��������*/
  car_handle_p->car_variable_reset();
	upper_cmd_header_p->set_cmd_type(E_UPPER_CMD_BRAKE);
		return 0;
	}

/*������λ������*/
static int8_t Car_Walk_Straight_Line_Updata(int32_t v,uint32_t s)	
 {
                           
 /*�����趨���ٶ�*/
	car_variable_p->car_speed = v;
	/*�����趨����*/
	car_variable_p->car_distance = s;
	/*ɾ����ǰ����ʻ����*/
	car_variable_p->now_car_distance = 0;
	 /*��ȡС����ʼ�Ƕ�*/
	car_variable_p->car_start_angle = imu_data_center_p->yaw;
	car_variable_p->last_yaw = car_variable_p->car_start_angle;
	/*��ȡ��ʼ���*/
	car_variable_p->last_car_distance = 	car_handle_p->read_car_distance();
	car_variable_p->car_start_stop = CAR_START;//--����
	 return 0;
 }
/*������λ������*/
static int8_t Car_Turn_Updata(int32_t V,float W,float A)	
 {


	car_variable_p->err_add =0;
  /*�����趨���ٶ�*/
	car_variable_p->car_speed = V;
	/*�����趨�Ľ��ٶ�*/
	car_variable_p->car_angular_speed = W;
	/*�����趨�ĽǶ�*/
	car_variable_p->car_angle = A;
	/*ɾ����ǰ�ĽǶ�*/
	car_variable_p->now_car_angle = 0;
	/*���³�ʼ��̬*/
	car_variable_p->car_start_angle = imu_data_center_p->yaw;
	car_variable_p->last_yaw = car_variable_p->car_start_angle;
	car_variable_p->car_start_stop = CAR_START;//--����
	return 0;
 }
 /*������λ������*/
static int8_t Car_Turn_8_Updata(int32_t V,float W,int Q)	
 {


	car_variable_p->err_add =0;
  /*�����趨���ٶ�*/
	car_variable_p->car_speed = V;
	/*�����趨�Ľ��ٶ�*/
	car_variable_p->car_angular_speed = W;
	/*�����趨�ĽǶ�*/
	car_variable_p->car_q = Q*2;
	/*ɾ����ǰ�ĽǶ�*/
	car_variable_p->now_car_angle = 0;
	/*���³�ʼ��̬*/
	car_variable_p->car_start_angle = imu_data_center_p->yaw;
	car_variable_p->last_yaw = car_variable_p->car_start_angle;
	car_variable_p->car_start_stop = CAR_START;//--����
	return 0;
 }
 //===================================================================================
//--С��ң����ֱ��
static int8_t Car_Remote_Line(void)
	{

	 /*������*/
	 left_wheel_handle_p->start(car_variable_p->car_speed);
   /*������*/
	 right_wheel_handle_p->start(car_variable_p->car_speed);
	 return 0;
	}
//--С��ң��ת��
static int8_t Car_Remote_Turn(void)
{
  int32_t w;
	int32_t vl,vr;
	/*��ȡ���ٶ�*/
	w = car_variable_p->car_angular_speed*100;//--������ת����
	/*�����趨�Ľ��ٶȶ�Ӧ�ĽǶ�*/
	int32_t angle= w*180/PI;
	/*V=0 ԭ����ת*/
	/*����Ϊ���֣�����Ϊ����*/
  vr = car_variable_p->car_speed - angle*PI*WHEEL_SPACING/2/180/100;//--����
  vl = car_variable_p->car_speed + angle*PI*WHEEL_SPACING/2/180/100;//--����
  /*������*/
	left_wheel_handle_p->start(vr);
  /*������*/
	right_wheel_handle_p->start(vl);
	return 0;
}
//=======================================================================================
//--С����ֱ��
static int8_t Car_Walk_Straight_Line(void)
	{
		int32_t reference_value, adjusted_value;
		int32_t last_yaw,naw_yaw;

		/*�ж�����ֹͣ��־*/
	if(car_variable_p->car_start_stop == CAR_STOP)
	{
	 /*�ǣ�ɲ��ͣ��*/
	 car_handle_p->car_stop();
	 return 0;
	}

//--------------------------��ȡ�Ƕ�����-----------------------------
	/*��ȡ��һ��ƫ����*/
	last_yaw = car_variable_p->last_yaw*100;//--������ת����
	/*��ȡ��ǰƫ����*/
	naw_yaw = imu_data_center_p->yaw*100;//--������ת����
	/*��ǰƫ���Ǹ��Ƹ���һ��ƫ����*/
	car_variable_p->last_yaw = imu_data_center_p->yaw;

//-------------------------------------------------------------------
	/*����С��ֱ�����߽Ƕ����*/
	car_handle_p->car_walk_straight_line_calculate_angle_difference(360,naw_yaw,last_yaw);
//--------------------------------------------------------------------------
	/*����Ƕ��������ٶ�ƫ����*/
	adjusted_value = car_pid_handle_p->car_walk_yaw_pid(car_variable_p->yaw_add);	
//--------------------------------------------------------------------------
  /*����С�����߾���*/
	uint32_t car_distance_temp = car_handle_p->read_car_distance();//--��ȡ��ǰ����
	uint32_t car_distance_temp1 = (car_distance_temp - car_variable_p->last_car_distance)*cos(car_variable_p->yaw_add/100);//--����С��ǰ������
  car_variable_p->last_car_distance = car_distance_temp;//--���浱ǰ�����	
	car_variable_p->now_car_distance += car_distance_temp1;//--�ۼ�ǰ�������
//--------------------------------------------------------------------------	
/*��������յ����20cm����ʼ����ͣ��*/
 reference_value = car_handle_p->slow_down_and_stop();
	/*����С���趨�ٶ�*/
	car_variable_p->car_speed = reference_value;	  
	/*���������ٶ�*/
	int32_t *buf;
	buf = car_handle_p->car_walk_straight_line_calculate_wheel_speed(car_variable_p->car_speed,adjusted_value);
	/*��ƫ>0 ��ƫ<0*/
	/*������*/
	left_wheel_handle_p->start(buf[1]);
  /*������*/
	right_wheel_handle_p->start(buf[0]);
	return 0;
	}

//==============================================================================
	//--С����ĳ������ǰ�����ߺ���
	
	/*
	*С��ת��뾶��ʽ�Ƶ��Ƶ�1 ,��֪ת��뾶�������������ٶ�
	* v1      R
	* --  =  ---  = k , v1 = k*v2
	* v2     R+L
	*     v1+v2
	* v = ------, v1+v2 = 2*v
	*       2
	*      k*2*v          2*v
	* v1 = -----    v2 = ----
	*      1+k            1+k
	* v1:�����ٶ�  v2:�����ٶ�  v��С���ٶ� R�����ְ뾶   R+L �����ְ뾶
	*С��ת��뾶��ʽ�Ƶ��Ƶ�2 ����֪���ٶȺͽ��ٶȣ������������ٶ�
	*       V1 + V2
	*  V = ---------
	*         2
	*       V2 - V1
	*  W = ---------
	*         L
	*        V      L*(V1 + V2)
	*  R = ----- = -------------
	*        W      2*(V2 - V1)
	*            W*L
	*  V1 = V - -----  
	*             2
	*
	*            W*L
	*  V2 = V + -----  
	*             2
	*
	*
	*���� = �Ƕ� * PI /180
	*�Ƕ� = ����/PI * 180
	*/
	//--�ٶȣ������Ҹ�
static uint8_t Car_Turn(void)
 { 
  int32_t w,naw_yaw,last_yaw,set_yaw;
		int32_t vl,vr;
//------------------------------------------------------------
			/*�ж�����ֹͣ��־*/
	if(car_variable_p->car_start_stop == CAR_STOP)
	{
	 /*�ǣ�ɲ��ͣ��*/
	car_handle_p->car_stop();
	return 0;
	}
//------------------------------------------------------------
	/*��ȡ���ٶ�*/
	w = car_variable_p->car_angular_speed*100;//--������ת����
	/*��ȡ��һ��ƫ����*/
	last_yaw = car_variable_p->last_yaw*100;//--������ת����
	/*��ȡ��ǰƫ����*/
	naw_yaw = imu_data_center_p->yaw*100;//--������ת����
	/*��ǰƫ���Ǹ��Ƹ���һ��ƫ����*/
	car_variable_p->last_yaw = imu_data_center_p->yaw;
//------------------------------������Ϊ��ֵ-------------------
	car_handle_p->car_turn_calculate_angle_difference(w,naw_yaw,last_yaw);
//-------------------------------------------------------------------------
	/*���������ٶ�*/
		/*�����趨�Ľ��ٶȶ�Ӧ�ĽǶ�*/
	   int32_t angle= w*180/PI;
		/*����Ϊ���֣�����Ϊ����*/
		 vr = car_variable_p->car_speed - angle*PI*WHEEL_SPACING/2/180/100;//--����
     vl = car_variable_p->car_speed + angle*PI*WHEEL_SPACING/2/180/100;//--����	
	
	/*��ȡ�趨�Ƕ�*/
	set_yaw = car_variable_p->car_angle*100;//--������ת����
	/*�ж��Ƿ�ת��ָ���Ƕ�*/
	if(car_variable_p->err_add >= set_yaw)
	{
		//if(queue_handle_p->list_is_null() == 0)//--�������������
		//	   {
		//		   if(queue_handle_p->get_queue_head_cmd_type() == E_UPPER_CMD_TURN)//--״̬һ��
			//		   {
						   /*������������±�־���ڴ˴�ֱ�ӻ�ȡ���ݸ���״̬*/
			//				upper_cmd_header_p->set_update_cmd_flag(1);//--�ⲿ������		
			//				 return 0;
			//			 }
			//	 }
			 
			 /*���״̬��һ�����ȴ�������һ���ʱ��ֹͣ������״̬�л�*/
			/*����������±�־*/
			// upper_cmd_header_p->set_update_cmd_flag(1);//--�ⲿ����
	    /*�ǣ�ɲ��ͣ��*/
	   car_handle_p->car_stop();
	 return 0;
	}
	/*���ǣ�����ת*/
	//--------------------����----------------------------
	//if(queue_handle_p->list_is_null() == 0)//--�������������
			  // {
				 // if(queue_handle_p->get_queue_head_cmd_type() != E_UPPER_CMD_TURN)//--״̬����ת��
					 //  {
			/*				 int k = 0;
		          if(w<0)k = -w/100;
		         else k = w/100; 
	          if((w<100)&&(w>-100))k=1;//--w����ֵС��1
		        if((set_yaw-car_variable_p->err_add)<=3000)//--��ֵС��w*30�ȿ�ʼ����
	                { 
				          vr /=2*k;  
				          vl /=2*k; 
	                }  
							 */
						// }
						 
					// }

	 //------------------------------------------------
	 /*������*/
	   left_wheel_handle_p->start(vr);
   /*������*/
	   right_wheel_handle_p->start(vl);
	return 0;
 }

	
	/*��ȡС�����ٶ�*/
	static int32_t Read_Speed(void)
	{
	 return car_variable_p->now_car_speed;
	}
	/*��ȡС�����ٶ�*/
	/*��ȡС����ʻ����*/
	static uint32_t Read_Car_Distance(void)
	{
	/*��ȡ�������*/
	uint32_t left_distance = left_wheel_handle_p->get_walk_distance();
	/*��ȡ�������*/
	//uint32_t right_distance = right_wheel_handle_p->get_walk_distance();
	/*����С�����*//*100ms�ۼ�һ�����*/
 // return car_variable_p->now_car_distance = (left_distance + right_distance)/2;
		return left_distance;
	}
	 /*��ȡС��״̬����*/
	static uint8_t Read_Car_Start_Stop(void)
	{
	 return car_variable_p->car_start_stop;
	}


/*С����������*/
static uint8_t Car_Variable_Reset(void)
{
	car_variable_p->car_speed =0;
	car_variable_p->now_car_speed = 0;
  car_variable_p->car_distance =0;
	car_variable_p->car_angle =0;
	car_variable_p->now_car_angle =0;
	car_variable_p->car_angular_speed =0;
	car_variable_p->now_car_angular_speed =0;
	car_variable_p->last_pitch = 0;
	car_variable_p->last_roll = 0;
	car_variable_p->last_yaw = 0;	
  car_variable_p->yaw_add =0;
	car_variable_p->err_add =0;
	return 0;
}
/***************************************************************************
** Function name:      Car_Walk_Straight_Line_Calculate_Angle_Difference
** Descriptions:        ����С����ֱ�߽ǶȲ�ֵ ��ƫΪ�� ��ƫΪ��������PID�����ٶȵ�����
** input parameters:     range:ѡ��IMU�Ƕȷ�Χ 180:0-180  360:0-360 
**                       naw_angle:��ǰ�Ƕ�
**                       ע�⣺��������ݶ����Ŵ��100������
**                       last_angle:��һ�νǶ�
** Returned value:       �ǶȲ�ֵ
***************************************************************************/
static uint32_t Car_Walk_Straight_Line_Calculate_Angle_Difference(uint16_t range,int32_t naw_angle , int32_t last_angle)
{
		int32_t err = last_angle-naw_angle;
	if(range == 180)
	{
	//+-180��Χ
  //--��ʱ��ת�����Ϊ����
		if((last_angle>900)&&(last_angle<=1800)&&(naw_angle<-900)&&(naw_angle>=-1800))//--��ʱ��ת��+-180���ٽ�ֵ
		{
		err = naw_angle-last_angle+3600;//--�������ڵ�ƫ��ֵ�����
		}
   //--˳ʱ��ת�����Ϊ����
		else if((last_angle<-900)&&(last_angle>=-1800)&&(naw_angle>900)&&(naw_angle<=1800))//--˳ʱ���+-180�ٽ�ֵ
		{
		err = naw_angle-last_angle-3600;
		}
   car_variable_p->yaw_add+=err;//--�ۼ���ʱ��ƫ��ֵ
		
		/*����������+-180֮��*/
		if(car_variable_p->yaw_add>18000)car_variable_p->yaw_add = car_variable_p->yaw_add -18000;
		if(car_variable_p->yaw_add<-18000)car_variable_p->yaw_add = car_variable_p->yaw_add +18000;
	}
  if(range == 360)
	{
		//--0-360��Χ
		//--��ʱ��Ϊ����˳ʱ��Ϊ��
		if((last_angle>=0)&&(last_angle<=4500)&&(naw_angle<=36000)&&(naw_angle>=31500))//--��ʱ��ת��0-360���ٽ�ֵ
		{
		err = -(naw_angle-last_angle-36000);//--�������ڵ�ƫ��ֵ�����
		}
		else if((naw_angle>=0)&&(naw_angle<=4500)&&(last_angle<=36000)&&(last_angle>=31500))//--˳ʱ��ת��0-360���ٽ�ֵ
		{
		err = -(naw_angle-last_angle+36000);//--�������ڵ�ƫ��ֵ�����
		}
		
		car_variable_p->yaw_add+=err;//--�ۼ���ʱ��ƫ��ֵ
		
		/*����������+-360֮��*/
		if(car_variable_p->yaw_add>36000)car_variable_p->yaw_add = car_variable_p->yaw_add -36000;
		if(car_variable_p->yaw_add<-36000)car_variable_p->yaw_add = car_variable_p->yaw_add +36000;
	}
	return car_variable_p->yaw_add;
}
/***************************************************************************
** Function name:      Car_Walk_Straight_Line_Calculate_Angle_Difference
** Descriptions:       ����С��ֱ�������������ٶ�
** input parameters:     reference_value :�ٶȻ�׼ֵ
**                        adjusted_value :�ٶȵ���ֵ
**                       
** Returned value:       buf[0] :�����ٶ� buf[1]:�����ٶ�
***************************************************************************/
static int32_t* Car_Walk_Straight_Line_Calculate_Wheel_Speed(int32_t reference_value,int32_t adjusted_value)
{
	static int32_t buf[2] = {0};
		if(reference_value>=0)//--ǰ��
		{
		  if(adjusted_value>=0)//--��ƫ
		  {
		    if(reference_value+adjusted_value>500)//--�����ٶ����
				{
				 buf[0] =-2*adjusted_value+500;//--Ŀǰ�������ת��500mm/s
				buf[1] =500;			  
				}
				else
				{
				buf[1] = reference_value+adjusted_value;//-- +
        buf[0]=reference_value-adjusted_value;	 //-- -	
				}	
		  }
     else if(adjusted_value<=0)//--��ƫ	 
		  {
		   if(reference_value-adjusted_value>500)//--�����ٶ����
				{
				 buf[1] =2*adjusted_value+500;
				 buf[0] =500;  
				}
				else
				{
				buf[1] = reference_value+adjusted_value;//-- +
        buf[0]=reference_value-adjusted_value;	 //-- -	
				}	
		  }
		}
		else if(reference_value<=0)//--����
		{
		 if(adjusted_value>=0)//--��ƫ
		  {
		    if(reference_value-adjusted_value<-500)//--�����ٶ����
				{
				 buf[1] =2*adjusted_value-500;
				 buf[0] =-500;			  
				}
				else
				{
				buf[1] = reference_value+adjusted_value;//-- +
        buf[0]=reference_value-adjusted_value;	 //-- -	
				}	
		  }
     else if(adjusted_value<=0)//--��ƫ	 
		  {
		   if(reference_value+adjusted_value<-500)//--�����ٶ����
				{
				 buf[0] =-2*adjusted_value-500;
				 buf[1] =-500;  
				}
				else
				{
				buf[1] = reference_value+adjusted_value;//-- +
        buf[0]=reference_value-adjusted_value;	 //-- -	
				}	
		  }
		}
		return buf;
}

/***************************************************************************
** Function name:      Slow_Down_And_Stop
** Descriptions:       ֱ�߼���ͣ��
** input parameters:   
**                       
** Returned value:      �ٶȻ�����
***************************************************************************/
static uint32_t Slow_Down_And_Stop(void)
{
		/*�ж��Ƿ�����ָ������*/
		if(car_variable_p->now_car_distance >= car_variable_p->car_distance)
		 {
		   /*�ǣ�ɲ��ͣ��*/
			 /*��ȡ��һ����״̬�Ƿ�͵�ǰ״̬һ�������һ��������һ������̼ӵ���ǰ�����*/
	//		 if(queue_handle_p->list_is_null() == 0)//--�������������
	//		   {
	//			   if(queue_handle_p->get_queue_head_cmd_type() == E_UPPER_CMD_MOVING_LINE)//--״̬һ��
	//				   {
						   /*������������±�־���ڴ˴�ֱ�ӻ�ȡ���ݸ���״̬*/
		//					upper_cmd_header_p->set_update_cmd_flag(1);//--�ⲿ������	
		//					 return 0;
		//				 }
		//		 }
		//	 
			 /*���״̬��һ�����ȴ�������һ���ʱ��ֹͣ������״̬�л�*/
			/*����������±�־*/
		//	 upper_cmd_header_p->set_update_cmd_flag(1);//--�ⲿ����
			 car_handle_p->car_stop();
			 /*���PID����*/
			 return 0;
		 }	
		/*���ǣ������������ٶȻ�����*/
 return car_pid_handle_p->car_walk_distance_pid(car_variable_p->car_speed,
		                                                       car_variable_p->now_car_distance,
		                                                       car_variable_p->car_distance);
	
}
/***************************************************************************
** Function name:      Car_Turn_Calculate_Angle_Difference
** Descriptions:       ����С��ת��Ƕȣ�ȫ������
** input parameters:    w:����0 ��ʱ��  С��0 ˳ʱ��
**                       naw_angle:��ǰ�Ƕ�
**                       last_angle:��һ�νǶ�
**                       ע�⣺��������ݶ����Ŵ��100������
** Returned value:       �ǶȲ�ֵ
***************************************************************************/
static int32_t Car_Turn_Calculate_Angle_Difference(int32_t w,int32_t naw_angle , int32_t last_angle)
{
	int32_t err;
	if(w>0)//--��ʱ��
	{
   	err = last_angle-naw_angle;
//--0-360��Χ
		//--��ʱ��Ϊ����˳ʱ��Ϊ��
		if((last_angle>=0)&&(last_angle<=4500)&&(naw_angle<=36000)&&(naw_angle>=31500))//--��ʱ��ת��0-360���ٽ�ֵ
		{
		err = -(naw_angle-last_angle-36000);//--�������ڵ�ƫ��ֵ�����
		}
		else if((naw_angle>=0)&&(naw_angle<=4500)&&(last_angle<=36000)&&(last_angle>=31500))//--��ʱ��ת��0-360���ٽ�ֵ
		{
		err = -(naw_angle-last_angle+36000);//--�������ڵ�ƫ��ֵ�����
		}
		car_variable_p->err_add+=err;//--�ۼ���ʱ��ƫ��ֵ
	}
	else if(w<0)//--˳ʱ��
	{
	 
   	err = naw_angle-last_angle;
//--0-360��Χ
		//--��ʱ��Ϊ����˳ʱ��Ϊ��
		if((last_angle>=0)&&(last_angle<=4500)&&(naw_angle<=36000)&&(naw_angle>=31500))//--��ʱ��ת��0-360���ٽ�ֵ
		{
		err = (naw_angle-last_angle-36000);//--�������ڵ�ƫ��ֵ�����
		}
		else if((naw_angle>=0)&&(naw_angle<=4500)&&(last_angle<=36000)&&(last_angle>=31500))//--��ʱ��ת��0-360���ٽ�ֵ
		{
		err = (naw_angle-last_angle+36000);//--�������ڵ�ƫ��ֵ�����
		}
		car_variable_p->err_add+=err;//--�ۼ���ʱ��ƫ��ֵ
	}
	return car_variable_p->err_add;
}

//--��ȡС����ͣ״̬
static uint8_t Get_Car_Stop_or_Start(void)
 {
  if(car_variable_p->car_start_stop == CAR_START)
		return 1;
	else if(car_variable_p->car_start_stop == CAR_STOP)
		return 0;
	return 0;
 }  
 
static uint8_t Car_Turn_8(void)
	{	
		int32_t w,naw_yaw,last_yaw,set_yaw;
		int32_t vl,vr;
//------------------------------------------------------------
			/*�ж�����ֹͣ��־*/
	if(car_variable_p->car_start_stop == CAR_STOP)
	{
	 /*�ǣ�ɲ��ͣ��*/
	car_handle_p->car_stop();
	return 0;
	}
//------------------------------------------------------------
	/*��ȡ���ٶ�*/
	w = car_variable_p->car_angular_speed*100;//--������ת����
	/*��ȡ��һ��ƫ����*/
	last_yaw = car_variable_p->last_yaw*100;//--������ת����
	/*��ȡ��ǰƫ����*/
	naw_yaw = imu_data_center_p->yaw*100;//--������ת����
	/*��ǰƫ���Ǹ��Ƹ���һ��ƫ����*/
	car_variable_p->last_yaw = imu_data_center_p->yaw;
//------------------------------������Ϊ��ֵ-------------------
	car_handle_p->car_turn_calculate_angle_difference(w,naw_yaw,last_yaw);
//-------------------------------------------------------------------------
	/*���������ٶ�*/
		/*�����趨�Ľ��ٶȶ�Ӧ�ĽǶ�*/
	   int32_t angle= w*180/PI;
		/*����Ϊ���֣�����Ϊ����*/
		 vr = car_variable_p->car_speed - angle*PI*WHEEL_SPACING/2/180/100;//--����
     vl = car_variable_p->car_speed + angle*PI*WHEEL_SPACING/2/180/100;//--����	
	
	/*��ȡ�趨�Ƕ�*/
	set_yaw = car_variable_p->car_angle*100;//--������ת����
	
	/*�ж��Ƿ�ת��ָ���Ƕ�*/
	if(car_variable_p->err_add >= 36000)
	{
   //--����
	car_variable_p->car_angular_speed = -car_variable_p->car_angular_speed;
	car_variable_p->err_add = 0;
	car_variable_p->car_q--;
	}
	
	if(car_variable_p->car_q == 0)
	{
	 car_handle_p->car_stop();
	}
	/*���ǣ�����ת*/
	//--------------------����----------------------------
//		int k = 0;
//		if(w<0)k = -w/100;
//		 else k = w/100; 
//	  if((w<100)&&(w>-100))k=1;//--w����ֵС��1
//		 if((set_yaw-car_variable_p->err_add)<=3000)//--��ֵС��w*30�ȿ�ʼ����
//	 { 
//				 vr /=2*k;  
//				 vl /=2*k; 
//	 }  
	 //------------------------------------------------
	 /*������*/
	   left_wheel_handle_p->start(vr);
   /*������*/
	   right_wheel_handle_p->start(vl);
	return 0;
	}
