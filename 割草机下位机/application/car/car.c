/**
  ******************************************************************************
  * @file    car.c
  * @author  小鱼儿飞丫飞
  * @version V0.0.1
  * @date    2019/7/30
  * @brief   伽利略机器人小车底盘控制
  ******************************************************************************/
	#include "car.h"
  #include "stdio.h"
	#include "math.h"
	#include "bno005.h"
  #include "uppercmd.h"
  #include "uart3_rcv_queue.h"
	#include "uppercmd.h"
	  /*小车初始化*/
static	uint8_t Car_Init(void);
	/*小车反初始化*/
static	uint8_t Car_Deinit(void);
	/*小车速度更新*/
static	uint8_t Car_Speed_Updata(void);
	/*小车正常停止*/
static	uint8_t Car_Stop(void);
	/*小车紧急停止*/
static	uint8_t Car_Brake(void);
/*接收上位机数据*/
static int8_t Car_Walk_Straight_Line_Updata(int32_t v,uint32_t s);
/*接收上位机数据*/
static int8_t Car_Turn_Updata(int32_t V,float W,float A);
/*小车遥控走直线*/
static int8_t Car_Remote_Line(void);
/*小车遥控转弯*/
static int8_t Car_Remote_Turn(void);
/*小车走直线*/
static	int8_t Car_Walk_Straight_Line(void);
/*小车原地转弯*/
/*小车沿某条曲线前进或者后退*/
static uint8_t Car_Turn(void);
/*旋转拐弯*/
/*读取小车线速度*/
static int32_t Read_Speed(void);
/*读取小车角速度*/
/*读取小车行驶距离*/
static uint32_t Read_Car_Distance(void);
 /*读取小车状态命令*/
static	uint8_t Read_Car_Start_Stop(void);
/*小车参数清零*/
static uint8_t Car_Variable_Reset(void);
/*计算小车直线行走的角度误差*/
static uint32_t Car_Walk_Straight_Line_Calculate_Angle_Difference(uint16_t range,int32_t naw_angle , int32_t last_angle);
/*计算小车直线行走左右轮速度*/
static int32_t* Car_Walk_Straight_Line_Calculate_Wheel_Speed(int32_t reference_value,int32_t adjusted_value);
/*减速停车*/
static uint32_t Slow_Down_And_Stop(void);
/*计算转弯角度和*/
static int32_t Car_Turn_Calculate_Angle_Difference(int32_t w,int32_t naw_angle , int32_t last_angle);
/*获取小车起停状态*/
static uint8_t Get_Car_Stop_or_Start(void);
/*画8字*/
	static uint8_t Car_Turn_8(void);
	static int8_t Car_Turn_8_Updata(int32_t V,float W,int Q);

  ST_car_variable  car_variable = {0};
	ST_car_variable_p  car_variable_p = &car_variable;
	ST_car_function_interface_handle   car_handle={
	  /*小车初始化*/
  Car_Init,
	/*小车反初始化*/
  Car_Deinit,
	/*小车速度更新*/
  Car_Speed_Updata,
	/*小车正常停止*/
  Car_Stop,
	/*小车紧急停止*/
  Car_Brake,
	/*接收上位机数据*/
  Car_Walk_Straight_Line_Updata,
  /*接收上位机数据*/
   Car_Turn_Updata,
	/*小车遥控走直线*/
    Car_Remote_Line,
/*小车遥控转弯*/
    Car_Remote_Turn,
	/*小车走直线*/
  Car_Walk_Straight_Line,
	/*小车原地转弯*/
	/*小车沿某条曲线前进或者后退*/
	Car_Turn,
	/*旋转拐弯*/
	/*读取小车线速度*/
  Read_Speed,
	/*读取小车角速度*/
	/*读取小车行驶距离*/
  Read_Car_Distance,
	/*读取小车状态命令*/	
	Read_Car_Start_Stop,
	/*小车参数清零*/
  Car_Variable_Reset,
	/*计算小车直线行走的角度误差*/
	Car_Walk_Straight_Line_Calculate_Angle_Difference,
	/*计算小车直线行走左右轮速度*/
	Car_Walk_Straight_Line_Calculate_Wheel_Speed,
	/*减速停车*/
	Slow_Down_And_Stop,
	/*计算转弯角度和*/
	Car_Turn_Calculate_Angle_Difference,
	/*获取小车启停状态*/
	Get_Car_Stop_or_Start,
	Car_Turn_8,
	 /*接收上位机数据*/
Car_Turn_8_Updata,
	};
  ST_car_function_interface_handle_p car_handle_p =&car_handle;


  /*小车初始化*/
static	uint8_t Car_Init(void)
	{
	 /*左轮初始化*/
		left_wheel_handle_p->init();
	/*右轮初始化*/
		right_wheel_handle_p->init();
		return 0;
	}
	/*小车反初始化*/
static	uint8_t Car_Deinit(void)
	{
	 /*左轮反初始化*/
		left_wheel_handle_p->deinit();
	/*右轮反初始化*/
		right_wheel_handle_p->deinit();
		return 0;
	}
	/*小车速度更新*/
static	uint8_t Car_Speed_Updata(void)
	{
	 /*右轮速度更新*/
	int32_t rv = right_wheel_handle_p->speed_updata();
	 /*左轮速度更新*/
	int32_t lv = left_wheel_handle_p->speed_updata();
	 /*计算小车两轮同向中心速度*/
	 car_variable_p->now_car_speed = (left_wheel_handle_p->read_speed()
		                               +right_wheel_handle_p->read_speed())/2;
		return 0;
	}
	/*小车正常停止*/
static	uint8_t Car_Stop(void)
	{
	/*设置小车状态*/
	car_variable_p->car_start_stop = CAR_STOP;//--停止
	 /*右轮停止*/
	right_wheel_handle_p->stop();
	/*左轮停止*/
	left_wheel_handle_p->stop();
		/*PID中间变量清零*/
	left_pid_handle_p->pwm_value_reset();
	right_pid_handle_p->pwm_value_reset();
	car_pid_handle_p->car_walk_distance_pid_value_reset();
	car_pid_handle_p->car_walk_yaw_pid_value_reset();
	/*小车中间参数清零*/
  car_handle_p->car_variable_reset();
	upper_cmd_header_p->set_cmd_type(E_UPPER_CMD_STOP);
		return 0;
	}
	/*小车紧急停止*/
static	uint8_t Car_Brake(void)
	{
	/*设置小车状态*/
	car_variable_p->car_start_stop = CAR_STOP;//--停止
	 /*右轮停止*/
	right_wheel_handle_p->brake();
	/*左轮停止*/
	left_wheel_handle_p->brake();
	/*PID中间变量清零*/
	left_pid_handle_p->pwm_value_reset();
	right_pid_handle_p->pwm_value_reset();
	car_pid_handle_p->car_walk_distance_pid_value_reset();
	car_pid_handle_p->car_walk_yaw_pid_value_reset();		
	/*小车中间参数清零*/
  car_handle_p->car_variable_reset();
	upper_cmd_header_p->set_cmd_type(E_UPPER_CMD_BRAKE);
		return 0;
	}

/*接收上位机数据*/
static int8_t Car_Walk_Straight_Line_Updata(int32_t v,uint32_t s)	
 {
                           
 /*更新设定的速度*/
	car_variable_p->car_speed = v;
	/*更新设定距离*/
	car_variable_p->car_distance = s;
	/*删除当前的行驶距离*/
	car_variable_p->now_car_distance = 0;
	 /*获取小车初始角度*/
	car_variable_p->car_start_angle = imu_data_center_p->yaw;
	car_variable_p->last_yaw = car_variable_p->car_start_angle;
	/*获取初始里程*/
	car_variable_p->last_car_distance = 	car_handle_p->read_car_distance();
	car_variable_p->car_start_stop = CAR_START;//--启动
	 return 0;
 }
/*接收上位机数据*/
static int8_t Car_Turn_Updata(int32_t V,float W,float A)	
 {


	car_variable_p->err_add =0;
  /*更新设定的速度*/
	car_variable_p->car_speed = V;
	/*更新设定的角速度*/
	car_variable_p->car_angular_speed = W;
	/*更新设定的角度*/
	car_variable_p->car_angle = A;
	/*删除当前的角度*/
	car_variable_p->now_car_angle = 0;
	/*更新初始姿态*/
	car_variable_p->car_start_angle = imu_data_center_p->yaw;
	car_variable_p->last_yaw = car_variable_p->car_start_angle;
	car_variable_p->car_start_stop = CAR_START;//--启动
	return 0;
 }
 /*接收上位机数据*/
static int8_t Car_Turn_8_Updata(int32_t V,float W,int Q)	
 {


	car_variable_p->err_add =0;
  /*更新设定的速度*/
	car_variable_p->car_speed = V;
	/*更新设定的角速度*/
	car_variable_p->car_angular_speed = W;
	/*更新设定的角度*/
	car_variable_p->car_q = Q*2;
	/*删除当前的角度*/
	car_variable_p->now_car_angle = 0;
	/*更新初始姿态*/
	car_variable_p->car_start_angle = imu_data_center_p->yaw;
	car_variable_p->last_yaw = car_variable_p->car_start_angle;
	car_variable_p->car_start_stop = CAR_START;//--启动
	return 0;
 }
 //===================================================================================
//--小车遥控走直线
static int8_t Car_Remote_Line(void)
	{

	 /*左轮走*/
	 left_wheel_handle_p->start(car_variable_p->car_speed);
   /*右轮走*/
	 right_wheel_handle_p->start(car_variable_p->car_speed);
	 return 0;
	}
//--小车遥控转弯
static int8_t Car_Remote_Turn(void)
{
  int32_t w;
	int32_t vl,vr;
	/*获取角速度*/
	w = car_variable_p->car_angular_speed*100;//--浮点型转整形
	/*计算设定的角速度对应的角度*/
	int32_t angle= w*180/PI;
	/*V=0 原地旋转*/
	/*右轮为内轮，左轮为外轮*/
  vr = car_variable_p->car_speed - angle*PI*WHEEL_SPACING/2/180/100;//--内轮
  vl = car_variable_p->car_speed + angle*PI*WHEEL_SPACING/2/180/100;//--外轮
  /*左轮走*/
	left_wheel_handle_p->start(vr);
  /*右轮走*/
	right_wheel_handle_p->start(vl);
	return 0;
}
//=======================================================================================
//--小车走直线
static int8_t Car_Walk_Straight_Line(void)
	{
		int32_t reference_value, adjusted_value;
		int32_t last_yaw,naw_yaw;

		/*判断启动停止标志*/
	if(car_variable_p->car_start_stop == CAR_STOP)
	{
	 /*是，刹车停机*/
	 car_handle_p->car_stop();
	 return 0;
	}

//--------------------------获取角度数据-----------------------------
	/*获取上一次偏航角*/
	last_yaw = car_variable_p->last_yaw*100;//--浮点型转整形
	/*获取当前偏航角*/
	naw_yaw = imu_data_center_p->yaw*100;//--浮点型转整形
	/*当前偏航角复制给上一次偏航角*/
	car_variable_p->last_yaw = imu_data_center_p->yaw;

//-------------------------------------------------------------------
	/*计算小车直线行走角度误差*/
	car_handle_p->car_walk_straight_line_calculate_angle_difference(360,naw_yaw,last_yaw);
//--------------------------------------------------------------------------
	/*计算角度误差，调整速度偏差量*/
	adjusted_value = car_pid_handle_p->car_walk_yaw_pid(car_variable_p->yaw_add);	
//--------------------------------------------------------------------------
  /*更新小车行走距离*/
	uint32_t car_distance_temp = car_handle_p->read_car_distance();//--获取当前距离
	uint32_t car_distance_temp1 = (car_distance_temp - car_variable_p->last_car_distance)*cos(car_variable_p->yaw_add/100);//--计算小车前进距离
  car_variable_p->last_car_distance = car_distance_temp;//--保存当前里程数	
	car_variable_p->now_car_distance += car_distance_temp1;//--累加前进里程数
//--------------------------------------------------------------------------	
/*如果距离终点相差20cm，开始减速停车*/
 reference_value = car_handle_p->slow_down_and_stop();
	/*更新小车设定速度*/
	car_variable_p->car_speed = reference_value;	  
	/*设置两轮速度*/
	int32_t *buf;
	buf = car_handle_p->car_walk_straight_line_calculate_wheel_speed(car_variable_p->car_speed,adjusted_value);
	/*左偏>0 右偏<0*/
	/*左轮走*/
	left_wheel_handle_p->start(buf[1]);
  /*右轮走*/
	right_wheel_handle_p->start(buf[0]);
	return 0;
	}

//==============================================================================
	//--小车沿某条曲线前进或者后退
	
	/*
	*小车转弯半径公式推导推导1 ,已知转弯半径，计算左右轮速度
	* v1      R
	* --  =  ---  = k , v1 = k*v2
	* v2     R+L
	*     v1+v2
	* v = ------, v1+v2 = 2*v
	*       2
	*      k*2*v          2*v
	* v1 = -----    v2 = ----
	*      1+k            1+k
	* v1:内轮速度  v2:外轮速度  v：小车速度 R：内轮半径   R+L ：外轮半径
	*小车转弯半径公式推导推导2 ，已知线速度和角速度，计算左右轮速度
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
	*弧度 = 角度 * PI /180
	*角度 = 弧度/PI * 180
	*/
	//--速度：左正右负
static uint8_t Car_Turn(void)
 { 
  int32_t w,naw_yaw,last_yaw,set_yaw;
		int32_t vl,vr;
//------------------------------------------------------------
			/*判断启动停止标志*/
	if(car_variable_p->car_start_stop == CAR_STOP)
	{
	 /*是，刹车停机*/
	car_handle_p->car_stop();
	return 0;
	}
//------------------------------------------------------------
	/*获取角速度*/
	w = car_variable_p->car_angular_speed*100;//--浮点型转整形
	/*获取上一次偏航角*/
	last_yaw = car_variable_p->last_yaw*100;//--浮点型转整形
	/*获取当前偏航角*/
	naw_yaw = imu_data_center_p->yaw*100;//--浮点型转整形
	/*当前偏航角复制给上一次偏航角*/
	car_variable_p->last_yaw = imu_data_center_p->yaw;
//------------------------------计算误差都为正值-------------------
	car_handle_p->car_turn_calculate_angle_difference(w,naw_yaw,last_yaw);
//-------------------------------------------------------------------------
	/*计算轮子速度*/
		/*计算设定的角速度对应的角度*/
	   int32_t angle= w*180/PI;
		/*右轮为内轮，左轮为外轮*/
		 vr = car_variable_p->car_speed - angle*PI*WHEEL_SPACING/2/180/100;//--内轮
     vl = car_variable_p->car_speed + angle*PI*WHEEL_SPACING/2/180/100;//--外轮	
	
	/*获取设定角度*/
	set_yaw = car_variable_p->car_angle*100;//--浮点型转整形
	/*判断是否转到指定角度*/
	if(car_variable_p->err_add >= set_yaw)
	{
		//if(queue_handle_p->list_is_null() == 0)//--命令队列有数据
		//	   {
		//		   if(queue_handle_p->get_queue_head_cmd_type() == E_UPPER_CMD_TURN)//--状态一样
			//		   {
						   /*不设置命令更新标志，在此处直接获取数据更新状态*/
			//				upper_cmd_header_p->set_update_cmd_flag(1);//--外部不更新		
			//				 return 0;
			//			 }
			//	 }
			 
			 /*如果状态不一样，等待到达下一点的时候停止，进行状态切换*/
			/*设置命令更新标志*/
			// upper_cmd_header_p->set_update_cmd_flag(1);//--外部更新
	    /*是，刹车停机*/
	   car_handle_p->car_stop();
	 return 0;
	}
	/*不是，继续转*/
	//--------------------减速----------------------------
	//if(queue_handle_p->list_is_null() == 0)//--命令队列有数据
			  // {
				 // if(queue_handle_p->get_queue_head_cmd_type() != E_UPPER_CMD_TURN)//--状态不是转弯
					 //  {
			/*				 int k = 0;
		          if(w<0)k = -w/100;
		         else k = w/100; 
	          if((w<100)&&(w>-100))k=1;//--w绝对值小于1
		        if((set_yaw-car_variable_p->err_add)<=3000)//--差值小于w*30度开始减速
	                { 
				          vr /=2*k;  
				          vl /=2*k; 
	                }  
							 */
						// }
						 
					// }

	 //------------------------------------------------
	 /*左轮走*/
	   left_wheel_handle_p->start(vr);
   /*右轮走*/
	   right_wheel_handle_p->start(vl);
	return 0;
 }

	
	/*读取小车线速度*/
	static int32_t Read_Speed(void)
	{
	 return car_variable_p->now_car_speed;
	}
	/*读取小车角速度*/
	/*读取小车行驶距离*/
	static uint32_t Read_Car_Distance(void)
	{
	/*读取左轮里程*/
	uint32_t left_distance = left_wheel_handle_p->get_walk_distance();
	/*读取右轮里程*/
	//uint32_t right_distance = right_wheel_handle_p->get_walk_distance();
	/*计算小车里程*//*100ms累加一次里程*/
 // return car_variable_p->now_car_distance = (left_distance + right_distance)/2;
		return left_distance;
	}
	 /*读取小车状态命令*/
	static uint8_t Read_Car_Start_Stop(void)
	{
	 return car_variable_p->car_start_stop;
	}


/*小车参数清零*/
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
** Descriptions:        计算小车走直线角度差值 左偏为正 右偏为负，方便PID计算速度调整量
** input parameters:     range:选择IMU角度范围 180:0-180  360:0-360 
**                       naw_angle:当前角度
**                       注意：传入的数据都被放大成100倍整数
**                       last_angle:上一次角度
** Returned value:       角度差值
***************************************************************************/
static uint32_t Car_Walk_Straight_Line_Calculate_Angle_Difference(uint16_t range,int32_t naw_angle , int32_t last_angle)
{
		int32_t err = last_angle-naw_angle;
	if(range == 180)
	{
	//+-180范围
  //--逆时针转，误差为正数
		if((last_angle>900)&&(last_angle<=1800)&&(naw_angle<-900)&&(naw_angle>=-1800))//--逆时针转到+-180度临界值
		{
		err = naw_angle-last_angle+3600;//--计算相邻的偏航值的误差
		}
   //--顺时针转，误差为负数
		else if((last_angle<-900)&&(last_angle>=-1800)&&(naw_angle>900)&&(naw_angle<=1800))//--顺时针过+-180临界值
		{
		err = naw_angle-last_angle-3600;
		}
   car_variable_p->yaw_add+=err;//--累加逆时针偏差值
		
		/*将误差控制在+-180之内*/
		if(car_variable_p->yaw_add>18000)car_variable_p->yaw_add = car_variable_p->yaw_add -18000;
		if(car_variable_p->yaw_add<-18000)car_variable_p->yaw_add = car_variable_p->yaw_add +18000;
	}
  if(range == 360)
	{
		//--0-360范围
		//--逆时针为负，顺时针为正
		if((last_angle>=0)&&(last_angle<=4500)&&(naw_angle<=36000)&&(naw_angle>=31500))//--逆时针转到0-360度临界值
		{
		err = -(naw_angle-last_angle-36000);//--计算相邻的偏航值的误差
		}
		else if((naw_angle>=0)&&(naw_angle<=4500)&&(last_angle<=36000)&&(last_angle>=31500))//--顺时针转到0-360度临界值
		{
		err = -(naw_angle-last_angle+36000);//--计算相邻的偏航值的误差
		}
		
		car_variable_p->yaw_add+=err;//--累加逆时针偏差值
		
		/*将误差控制在+-360之内*/
		if(car_variable_p->yaw_add>36000)car_variable_p->yaw_add = car_variable_p->yaw_add -36000;
		if(car_variable_p->yaw_add<-36000)car_variable_p->yaw_add = car_variable_p->yaw_add +36000;
	}
	return car_variable_p->yaw_add;
}
/***************************************************************************
** Function name:      Car_Walk_Straight_Line_Calculate_Angle_Difference
** Descriptions:       计算小车直线行走左右轮速度
** input parameters:     reference_value :速度基准值
**                        adjusted_value :速度调整值
**                       
** Returned value:       buf[0] :右轮速度 buf[1]:左轮速度
***************************************************************************/
static int32_t* Car_Walk_Straight_Line_Calculate_Wheel_Speed(int32_t reference_value,int32_t adjusted_value)
{
	static int32_t buf[2] = {0};
		if(reference_value>=0)//--前进
		{
		  if(adjusted_value>=0)//--左偏
		  {
		    if(reference_value+adjusted_value>500)//--左轮速度溢出
				{
				 buf[0] =-2*adjusted_value+500;//--目前轮子最大转速500mm/s
				buf[1] =500;			  
				}
				else
				{
				buf[1] = reference_value+adjusted_value;//-- +
        buf[0]=reference_value-adjusted_value;	 //-- -	
				}	
		  }
     else if(adjusted_value<=0)//--右偏	 
		  {
		   if(reference_value-adjusted_value>500)//--右轮速度溢出
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
		else if(reference_value<=0)//--后退
		{
		 if(adjusted_value>=0)//--左偏
		  {
		    if(reference_value-adjusted_value<-500)//--右轮速度溢出
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
     else if(adjusted_value<=0)//--右偏	 
		  {
		   if(reference_value+adjusted_value<-500)//--左轮速度溢出
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
** Descriptions:       直线减速停车
** input parameters:   
**                       
** Returned value:      速度基础量
***************************************************************************/
static uint32_t Slow_Down_And_Stop(void)
{
		/*判断是否走完指定距离*/
		if(car_variable_p->now_car_distance >= car_variable_p->car_distance)
		 {
		   /*是，刹车停机*/
			 /*获取下一个点状态是否和当前状态一样，如果一样，把下一个点里程加到当前里程上*/
	//		 if(queue_handle_p->list_is_null() == 0)//--命令队列有数据
	//		   {
	//			   if(queue_handle_p->get_queue_head_cmd_type() == E_UPPER_CMD_MOVING_LINE)//--状态一样
	//				   {
						   /*不设置命令更新标志，在此处直接获取数据更新状态*/
		//					upper_cmd_header_p->set_update_cmd_flag(1);//--外部不更新	
		//					 return 0;
		//				 }
		//		 }
		//	 
			 /*如果状态不一样，等待到达下一点的时候停止，进行状态切换*/
			/*设置命令更新标志*/
		//	 upper_cmd_header_p->set_update_cmd_flag(1);//--外部更新
			 car_handle_p->car_stop();
			 /*清除PID参数*/
			 return 0;
		 }	
		/*不是，计算误差，调整速度基础量*/
 return car_pid_handle_p->car_walk_distance_pid(car_variable_p->car_speed,
		                                                       car_variable_p->now_car_distance,
		                                                       car_variable_p->car_distance);
	
}
/***************************************************************************
** Function name:      Car_Turn_Calculate_Angle_Difference
** Descriptions:       计算小车转弯角度，全是正数
** input parameters:    w:大于0 逆时针  小于0 顺时针
**                       naw_angle:当前角度
**                       last_angle:上一次角度
**                       注意：传入的数据都被放大成100倍整数
** Returned value:       角度差值
***************************************************************************/
static int32_t Car_Turn_Calculate_Angle_Difference(int32_t w,int32_t naw_angle , int32_t last_angle)
{
	int32_t err;
	if(w>0)//--逆时针
	{
   	err = last_angle-naw_angle;
//--0-360范围
		//--逆时针为负，顺时针为正
		if((last_angle>=0)&&(last_angle<=4500)&&(naw_angle<=36000)&&(naw_angle>=31500))//--逆时针转到0-360度临界值
		{
		err = -(naw_angle-last_angle-36000);//--计算相邻的偏航值的误差
		}
		else if((naw_angle>=0)&&(naw_angle<=4500)&&(last_angle<=36000)&&(last_angle>=31500))//--逆时针转到0-360度临界值
		{
		err = -(naw_angle-last_angle+36000);//--计算相邻的偏航值的误差
		}
		car_variable_p->err_add+=err;//--累加逆时针偏差值
	}
	else if(w<0)//--顺时针
	{
	 
   	err = naw_angle-last_angle;
//--0-360范围
		//--逆时针为负，顺时针为正
		if((last_angle>=0)&&(last_angle<=4500)&&(naw_angle<=36000)&&(naw_angle>=31500))//--逆时针转到0-360度临界值
		{
		err = (naw_angle-last_angle-36000);//--计算相邻的偏航值的误差
		}
		else if((naw_angle>=0)&&(naw_angle<=4500)&&(last_angle<=36000)&&(last_angle>=31500))//--逆时针转到0-360度临界值
		{
		err = (naw_angle-last_angle+36000);//--计算相邻的偏航值的误差
		}
		car_variable_p->err_add+=err;//--累加逆时针偏差值
	}
	return car_variable_p->err_add;
}

//--获取小车启停状态
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
			/*判断启动停止标志*/
	if(car_variable_p->car_start_stop == CAR_STOP)
	{
	 /*是，刹车停机*/
	car_handle_p->car_stop();
	return 0;
	}
//------------------------------------------------------------
	/*获取角速度*/
	w = car_variable_p->car_angular_speed*100;//--浮点型转整形
	/*获取上一次偏航角*/
	last_yaw = car_variable_p->last_yaw*100;//--浮点型转整形
	/*获取当前偏航角*/
	naw_yaw = imu_data_center_p->yaw*100;//--浮点型转整形
	/*当前偏航角复制给上一次偏航角*/
	car_variable_p->last_yaw = imu_data_center_p->yaw;
//------------------------------计算误差都为正值-------------------
	car_handle_p->car_turn_calculate_angle_difference(w,naw_yaw,last_yaw);
//-------------------------------------------------------------------------
	/*计算轮子速度*/
		/*计算设定的角速度对应的角度*/
	   int32_t angle= w*180/PI;
		/*右轮为内轮，左轮为外轮*/
		 vr = car_variable_p->car_speed - angle*PI*WHEEL_SPACING/2/180/100;//--内轮
     vl = car_variable_p->car_speed + angle*PI*WHEEL_SPACING/2/180/100;//--外轮	
	
	/*获取设定角度*/
	set_yaw = car_variable_p->car_angle*100;//--浮点型转整形
	
	/*判断是否转到指定角度*/
	if(car_variable_p->err_add >= 36000)
	{
   //--换向
	car_variable_p->car_angular_speed = -car_variable_p->car_angular_speed;
	car_variable_p->err_add = 0;
	car_variable_p->car_q--;
	}
	
	if(car_variable_p->car_q == 0)
	{
	 car_handle_p->car_stop();
	}
	/*不是，继续转*/
	//--------------------减速----------------------------
//		int k = 0;
//		if(w<0)k = -w/100;
//		 else k = w/100; 
//	  if((w<100)&&(w>-100))k=1;//--w绝对值小于1
//		 if((set_yaw-car_variable_p->err_add)<=3000)//--差值小于w*30度开始减速
//	 { 
//				 vr /=2*k;  
//				 vl /=2*k; 
//	 }  
	 //------------------------------------------------
	 /*左轮走*/
	   left_wheel_handle_p->start(vr);
   /*右轮走*/
	   right_wheel_handle_p->start(vl);
	return 0;
	}
