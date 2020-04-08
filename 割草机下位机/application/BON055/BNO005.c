
/**
  ******************************************************************************
  * @file    bno055.c
  * @author 
  * @version 
  * @date    2018/04/11
  * @brief   
  ******************************************************************************
  */



/* Include Section ----------------------------------------------------*/
#include "bno005.h"


/* data define Section ------------------------------------------------*/

//typedef short int16_t;
//typedef unsigned char uint8_t;


/*config bno055*/ 
uint8_t Bno055_Config(uint8_t position,uint8_t writeFlag);
uint8_t Bno055_Init(uint8_t position);
/*配置校准模式*/ 
uint8_t Bno055_ConfigCalib(uint8_t position);
/*get bno055 calibration status*/ //--获取bno055校准状态
uint8_t Bno055_Calibrate(void);
/*读取偏航角*/
float Read_Bno055_Yaw(void);
/*读取欧拉角数据*/
uint8_t Bno055_Get_Data(float *pitch,float *roll,float *yaw);
/*读取角速度数据*/
uint8_t Bno055_Get_Acc_Data(float *x_axis,float *y_axis,float *z_axis);
/*读取磁力计数据*/
uint8_t Bno055_Get_Mag_Data(float *x_axis,float *y_axis,float *z_axis);
/*读取陀螺仪数据*/
uint8_t Bno055_Get_Gyo_Data(float *x_axis,float *y_axis,float *z_axis);
uint8_t Bno005_Err_Display(uint8_t err_num);

//-----------------------------------------------------
ST_NDOF st_ndof ={
	0,
	0,
	0
};
ST_SENSOR st_acc={
	0,
	0,
	0
}; 
ST_SENSOR st_mag={
	0,
	0,
	0
}; 
ST_SENSOR st_gyo={
	0,
	0,
	0
}; 
//----------------------------------------------------
ST_bno055_handle bno055_handle = {
Bno055_Config,
Bno055_Init,
Bno055_ConfigCalib,
Bno055_Calibrate,
Read_Bno055_Yaw,
Bno055_Get_Data,
Bno055_Get_Acc_Data,
Bno055_Get_Mag_Data,
Bno055_Get_Gyo_Data,
Bno005_Err_Display,

};
ST_bon055_handle_p bon055_handle_p = &bno055_handle;
//---------------------------------------------------
/* extren funciton Section --------------------------------------------*/

/* define Section -----------------------------------------------------*/

#define CHECK_NUM 400 

#define NINE_AXIS_I2C_BUS LPC_I2C0

/********************************************************/
/**\name	I2C ADDRESS DEFINITION FOR BNO055           */
/********************************************************/
/* bno055 I2C Address */
//--7位地址模式  默认地址0x29  
#define BNO055_I2C_ADDR1                (0x28<<1)//--芯片I2C替代地址
#define BNO055_I2C_ADDR2                (0x29<<1)//--芯片I2C默认地址
/***************************************************/
/**\name	REGISTER ADDRESS DEFINITION  */
/***************************************************/
/* Page id register definition*/
#define BNO055_PAGE_ID_ADDR				    (0X07)//--r：当前寄存器页面（一共两个页面）w:修改当前页面 ，0x00 0x01

/* PAGE0 REGISTER DEFINITION START*/
#define BNO055_CHIP_ID_ADDR                 (0x00)//--芯片识别码，只读固定值0XA0
#define BNO055_ACCEL_REV_ID_ADDR			(0x01)//--加速度计装置的芯片id，只读固定值0xfb
#define BNO055_MAG_REV_ID_ADDR              (0x02)//--磁力计装置的芯片id ，只读定值 0x32 
#define BNO055_GYRO_REV_ID_ADDR             (0x03)//--陀螺仪装置的芯片id，只读固定值0x0f
#define BNO055_SW_REV_ID_LSB_ADDR			(0x04)//--当前版本号ID低字节，不同芯片可能不一样，没有固定值
#define BNO055_SW_REV_ID_MSB_ADDR			(0x05)//--当前版本号ID高字节，不同芯片可能不一样，没有固定值
#define BNO055_BL_REV_ID_ADDR				(0X06)//--Bootloader版本

/* Accel data register*/
#define BNO055_ACCEL_DATA_X_LSB_ADDR		(0X08)//--X轴加速度数据低字节，只读，参考数据手册第3.3节，更改输出类型
#define BNO055_ACCEL_DATA_X_MSB_ADDR		(0X09)//--X轴加速度数据高字节，只读
#define BNO055_ACCEL_DATA_Y_LSB_ADDR		(0X0A)//--Y轴加速度数据低字节
#define BNO055_ACCEL_DATA_Y_MSB_ADDR		(0X0B)//--Y轴加速度数据高字节
#define BNO055_ACCEL_DATA_Z_LSB_ADDR		(0X0C)//--Z轴接速度数据低字节
#define BNO055_ACCEL_DATA_Z_MSB_ADDR		(0X0D)//--Z轴接速度数据高字节

/*Mag data register*/
#define BNO055_MAG_DATA_X_LSB_ADDR			(0X0E)//--X轴磁力计数据低字节
#define BNO055_MAG_DATA_X_MSB_ADDR			(0X0F)//--X轴磁力计数据高字节
#define BNO055_MAG_DATA_Y_LSB_ADDR			(0X10)//--Y轴磁力计数据低字节
#define BNO055_MAG_DATA_Y_MSB_ADDR			(0X11)//--Y轴磁力计数据高字节
#define BNO055_MAG_DATA_Z_LSB_ADDR			(0X12)//--Z轴磁力计数据低字节
#define BNO055_MAG_DATA_Z_MSB_ADDR			(0X13)//--Z轴磁力计数据高字节

/*Gyro data registers*/
#define BNO055_GYRO_DATA_X_LSB_ADDR			(0X14)//--X轴陀螺仪数据低字节
#define BNO055_GYRO_DATA_X_MSB_ADDR			(0X15)//--X轴陀螺仪数据高字节
#define BNO055_GYRO_DATA_Y_LSB_ADDR			(0X16)//--Y轴陀螺仪数据低字节
#define BNO055_GYRO_DATA_Y_MSB_ADDR			(0X17)//--Y轴陀螺仪数据高字节
#define BNO055_GYRO_DATA_Z_LSB_ADDR			(0X18)//--Z轴陀螺仪数据低字节
#define BNO055_GYRO_DATA_Z_MSB_ADDR			(0X19)//--Z轴陀螺仪数据高字节

/*Euler data registers*/
#define BNO055_EULER_H_LSB_ADDR			    (0X1A)//--heading/yaw 偏航角数据低字节，只读，参考数据手册第3.3节，更改输出类型
#define BNO055_EULER_H_MSB_ADDR			    (0X1B)//--heading/yaw 偏航角数据高字节
                                      
#define BNO055_EULER_R_LSB_ADDR			    (0X1C)//--roll 翻滚角数据低字节
#define BNO055_EULER_R_MSB_ADDR			    (0X1D)//--roll 翻滚角数据高字节
                                            
#define BNO055_EULER_P_LSB_ADDR			    (0X1E)//--pitch 俯仰角数据低字节
#define BNO055_EULER_P_MSB_ADDR			    (0X1F)//--pitch 俯仰角数据高字节

/*Quaternion data registers*/
#define BNO055_QUATERNION_DATA_W_LSB_ADDR	(0X20)//--W四元数低字节
#define BNO055_QUATERNION_DATA_W_MSB_ADDR	(0X21)//--W四元数高字节
#define BNO055_QUATERNION_DATA_X_LSB_ADDR	(0X22)//--X四元数低字节
#define BNO055_QUATERNION_DATA_X_MSB_ADDR	(0X23)//--X四元数高字节
#define BNO055_QUATERNION_DATA_Y_LSB_ADDR	(0X24)//--Y四元数低字节
#define BNO055_QUATERNION_DATA_Y_MSB_ADDR	(0X25)//--Y四元数高字节
#define BNO055_QUATERNION_DATA_Z_LSB_ADDR	(0X26)//--Z四元数低字节
#define BNO055_QUATERNION_DATA_Z_MSB_ADDR	(0X27)//--Z四元数高字节

/* Linear acceleration data registers*/
/*线性加速就是从起步到急速都是持续不断的动力推进感*/
/*加速度是随着时间的变化而变化，它的变化是一次函数变化规律*/
/*a = a。+ kt     其中k为某一个常数。a。为被始加速度状态*/
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR	(0X28)//--X轴线性加速数据低字节
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR	(0X29)//--X轴线性加速数据高字节
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR	(0X2A)//--Y轴线性加速数据低字节
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR	(0X2B)//--Y轴线性加速数据高字节
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR	(0X2C)//--Z轴线性加速数据低字节
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR	(0X2D)//--Z轴线性加速数据高字节

/*Gravity data registers*/
#define BNO055_GRAVITY_DATA_X_LSB_ADDR		(0X2E)//--X轴重力矢量数据的低字节
#define BNO055_GRAVITY_DATA_X_MSB_ADDR		(0X2F)//--X轴重力矢量数据的高字节
#define BNO055_GRAVITY_DATA_Y_LSB_ADDR		(0X30)//--Y轴重力矢量数据的低字节
#define BNO055_GRAVITY_DATA_Y_MSB_ADDR		(0X31)//--Y轴重力矢量数据的高字节
#define BNO055_GRAVITY_DATA_Z_LSB_ADDR		(0X32)//--Z轴重力矢量数据的低字节
#define BNO055_GRAVITY_DATA_Z_MSB_ADDR		(0X33)//--Z轴重力矢量诗句的高字节

/* Temperature data register*/
#define BNO055_TEMP_ADDR					(0X34)//--温度数据，只读

/* Status registers*/
#define BNO055_CALIB_STAT_ADDR				(0X35)
/*
* <7:6> 当前系统校准状态，取决于所有传感器的状态，只读读数：3表示完全校准，0表示完全未校准
* <5:4> 陀螺仪当前校准状态
* <3:2> 加速度计当前校准状态
* <1:0> 磁力计的当前校准状态
**/
#define BNO055_SELFTEST_RESULT_ADDR			(0X36)
/*
* bit0 微控制器自检结果 1表示测试通过，0表示测试未通过
* bit1 陀螺仪自检结果  1表示测试通过  0表示测试未通过
* bit2 磁力计自检结果  1表示测试通过 0表示测试未通过
* bit3 加速度计自检结果  1表示测试通过 0表示测试未通过
*/
#define BNO055_INTR_STAT_ADDR				(0X37)
/*
* bit7 加速度计的状态无运动或慢动作中断  1表示中断触发  0表示没有中断触发
* bit6 任何运动的加速度计状态 1表示中断触发  0表示没有中断触发
* bit5 加速度计高G中断状态  1表示中断触发  0表示没有中断触发
* bit3 陀螺仪高速率中断的状态  1表示中断触发  0表示没有中断触发
* bit2 陀螺仪的任何状态中断  1表示中断触发  0表示没有中断触发 
*/
#define BNO055_SYS_CLK_STAT_ADDR			(0X38)//-- bit0  0表示可以自由配置CLK（外部或内部） 1表示它处于配置状态
#define BNO055_SYS_STAT_ADDR				(0X39)
/*
* 0  系统空闲
* 1  系统错误
* 2  初始化外围设备
* 3  系统初始化
* 4  执行自检
* 5 运行传感器融合算法
* 6 系统运行时没有融合算法
*/
/* System states*/
/*对应0x39地址寄存器的值*/
#define SYSTEM_IDLE                         (0) 
#define SYSTEM_ERROR                        (1)
#define INITIALIZING_PERIPHERALS            (2)
#define SYSTEM_INITIALIZATION               (3)
#define EXECUTING_SELFTEST                  (4)
#define SENSOR_FUSION_ALGORITHM_RUN         (5)
#define SYSTEM_RUN_WITHOUT_FUSION_ALGORITHM (6)

#define BNO055_SYS_ERR_ADDR					(0X3A)//--如果0x39寄存器出现0x01系统错误，则从该寄存器读取错误状态
/*
* 0  没有错误
* 1  外设初始化错误
* 2  系统初始化错误
* 3  自检失败
* 4  注册地图值超出范围
* 5  注册地图地址超出范围
* 6 注册映射写入错误
* 7 BNO低功耗模式不适用于所选的操作模式
* 8 无法适用加速度计电源模式
* 9 Fusion算法配置错误 传感器配置错误
*/

/* Unit selection register*/
#define BNO055_UNIT_SEL_ADDR				(0X3B)//--单位统一
/*
* bit7  ORI_Android_Win  读：当前选择的定向模式  写：选择方向模式 0：windows方向  1：Android方向  细节参考3.6.2
* bit5  TEMP_Unit        读：当前选择的温度单位  写：选择温度单位 0：摄氏度  1：华氏度
* bit3  EUL_Unit         读：当前选择的欧拉角单位 写：选择欧拉角单位 0：度  1：弧度
* bit2  GRY_Unit         读：当前选定的角速度单位 写：选择角速度单位  0：dps 度/秒  1:rps 转/分钟（单位描述不一定准确，待查证）
* bit1  ACC_Unit         读：当前选定的加速度单位 写：选择加速度单位  0：m/s2   1:mg
*/
#define BNO055_DATA_SELECT_ADDR				(0X3C) //--BNO055数据手册上没有

/* Mode registers*/
#define BNO055_OPR_MODE_ADDR				(0X3D)//--<3:0> 读：当前选定的操作模式 写：选择操作模式  细节见3.3部分
#define BNO055_PWR_MODE_ADDR				(0X3E)//--<1:0> 读：当前选的电源模式 写 选择电源模式   细节见0部分

#define BNO055_SYS_TRIGGER_ADDR				(0X3F)
/* 
*  bit7  0:使用内部振荡器  1：使用外部振荡器
*  bit6  置1重置所有中断状态位，和 int 输出..
*  bit5  置1重置系统
*  bit0  置1触发自检
*/
#define BNO055_TEMP_SOURCE_ADDR				(0X40)//--温度源  详情见3.6.5.8节
/* Axis remap registers*/
#define BNO055_AXIS_MAP_CONFIG_ADDR			(0X41) //--<5:4>重映射Z轴值 <3:2>重映射Y轴值 <1:0>重映射X轴值 详情见3.4节
#define BNO055_AXIS_MAP_SIGN_ADDR			(0X42)//--bit2:重映射X轴符号 bit1:重映射Y轴符号 bit0 :重映射Z轴符号  //--详情见3.4节

/* SIC registers*/
#define BNO055_SIC_MATRIX_0_LSB_ADDR		(0X43)//---数据手册没找到
#define BNO055_SIC_MATRIX_0_MSB_ADDR		(0X44)//---数据手册没找到
#define BNO055_SIC_MATRIX_1_LSB_ADDR		(0X45)//---数据手册没找到
#define BNO055_SIC_MATRIX_1_MSB_ADDR		(0X46)//---数据手册没找到
#define BNO055_SIC_MATRIX_2_LSB_ADDR		(0X47)//---数据手册没找到
#define BNO055_SIC_MATRIX_2_MSB_ADDR		(0X48)//---数据手册没找到
#define BNO055_SIC_MATRIX_3_LSB_ADDR		(0X49)//---数据手册没找到
#define BNO055_SIC_MATRIX_3_MSB_ADDR		(0X4A)//---数据手册没找到
#define BNO055_SIC_MATRIX_4_LSB_ADDR		(0X4B)//---数据手册没找到
#define BNO055_SIC_MATRIX_4_MSB_ADDR		(0X4C)//---数据手册没找到
#define BNO055_SIC_MATRIX_5_LSB_ADDR		(0X4D)//---数据手册没找到
#define BNO055_SIC_MATRIX_5_MSB_ADDR		(0X4E)//---数据手册没找到
#define BNO055_SIC_MATRIX_6_LSB_ADDR		(0X4F)//---数据手册没找到
#define BNO055_SIC_MATRIX_6_MSB_ADDR		(0X50)//---数据手册没找到
#define BNO055_SIC_MATRIX_7_LSB_ADDR		(0X51)//---数据手册没找到
#define BNO055_SIC_MATRIX_7_MSB_ADDR		(0X52)//---数据手册没找到
#define BNO055_SIC_MATRIX_8_LSB_ADDR		(0X53)//---数据手册没找到
#define BNO055_SIC_MATRIX_8_MSB_ADDR		(0X54)//--数据手册没找到

/* Accelerometer Offset registers*/
#define BNO055_ACCEL_OFFSET_X_LSB_ADDR		(0X55)//--X轴加速度计补偿地位  //--详情见3.6.4节
#define BNO055_ACCEL_OFFSET_X_MSB_ADDR		(0X56)//--X轴加速度计补偿高位
#define BNO055_ACCEL_OFFSET_Y_LSB_ADDR		(0X57)//--Y轴加速度补偿
#define BNO055_ACCEL_OFFSET_Y_MSB_ADDR		(0X58)
#define BNO055_ACCEL_OFFSET_Z_LSB_ADDR		(0X59)//--Z轴加速度补偿
#define BNO055_ACCEL_OFFSET_Z_MSB_ADDR		(0X5A)

/* Magnetometer Offset registers*/
#define BNO055_MAG_OFFSET_X_LSB_ADDR		(0X5B)//--X轴磁力计补偿 //--详情见3.6.4节
#define BNO055_MAG_OFFSET_X_MSB_ADDR		(0X5C)
#define BNO055_MAG_OFFSET_Y_LSB_ADDR		(0X5D)//--Y轴磁力计补偿
#define BNO055_MAG_OFFSET_Y_MSB_ADDR		(0X5E)
#define BNO055_MAG_OFFSET_Z_LSB_ADDR		(0X5F)//--Z轴磁力计补偿
#define BNO055_MAG_OFFSET_Z_MSB_ADDR		(0X60)

/* Gyroscope Offset registers*/
#define BNO055_GYRO_OFFSET_X_LSB_ADDR		(0X61)//--X轴陀螺仪补偿 //--详情见3.6.4节
#define BNO055_GYRO_OFFSET_X_MSB_ADDR		(0X62)
#define BNO055_GYRO_OFFSET_Y_LSB_ADDR		(0X63)//--Y轴陀螺仪补偿
#define BNO055_GYRO_OFFSET_Y_MSB_ADDR		(0X64)
#define BNO055_GYRO_OFFSET_Z_LSB_ADDR		(0X65)//--Z轴陀螺仪补偿
#define BNO055_GYRO_OFFSET_Z_MSB_ADDR		(0X66)

/* Radius registers*/
#define	BNO055_ACCEL_RADIUS_LSB_ADDR		(0X67)//--加速度半径
#define	BNO055_ACCEL_RADIUS_MSB_ADDR		(0X68)//--
#define	BNO055_MAG_RADIUS_LSB_ADDR			(0X69)//--磁力计半径
#define	BNO055_MAG_RADIUS_MSB_ADDR			(0X6A)
/* PAGE0 REGISTERS DEFINITION END*/

/* PAGE1 REGISTERS DEFINITION START*/
/* Configuration registers*/
#define BNO055_ACCEL_CONFIG_ADDR			(0X08)
/*
* <7:5> 读：当前的加速度电源模式  ，只有在传感器模式下才能写入  详情见3.5.2节
* <4:3> 读：当前加速度频率
* <2:0> 读：当前选择的加速度范围
**/
#define BNO055_MAG_CONFIG_ADDR				(0X09)
/*
* <6:5> 读：当前的磁力计电源模式  ，只有在传感器模式下才能写入  详情见3.5.2节
* <4:3> 读：当前磁力计频率
* <2:0> 读：当前选择的磁力计范围
**/
#define BNO055_GYRO_CONFIG_ADDR				(0X0A)//--<5:3>陀螺仪带宽 <2:0> 陀螺仪范围  只有在传感器模式下才能写入  详情见3.5.3节
#define BNO055_GYRO_MODE_CONFIG_ADDR		(0X0B)//--<2:0>当前陀螺及电源模式
#define BNO055_ACCEL_SLEEP_CONFIG_ADDR		(0X0C) 
/*
*<4:1>写入：加速度计低功耗模式的睡眠时间只能在没有运行融合库的传感器操作模式下配置。在睡眠阶段之后，可以设置持续时间。
*bit0 :写0：使用事件驱动的时基模式  1：使用等距采样时基模式
*/
#define BNO055_GYRO_SLEEP_CONFIG_ADDR		(0X0D)
/*
* <5:3>陀螺仪可以在高级电源模式下配置，以优化功耗。自动睡眠持续时间的可能配置 详情看手册
* <2:0>陀螺仪可以在高级电源模式下配置，以优化功耗  睡眠持续时间的可能配置  详情看手册
*/
#define BNO055_MAG_SLEEP_CONFIG_ADDR		(0x0E)//--数据手册没找到

/* Interrupt registers*/
#define BNO055_INT_MASK_ADDR				(0X0F)//--屏蔽各种中断，具体查看数据手册
#define BNO055_INT_ADDR						(0X10)//--开启关闭各种中断，具体查看数据手册
#define BNO055_ACCEL_ANY_MOTION_THRES_ADDR	(0X11)//--设置中断阈值，具体查看手册
#define BNO055_ACCEL_INTR_SETTINGS_ADDR		(0X12)//--选择哪个加速度计触发哪个中断，具体查看数据手册
#define BNO055_ACCEL_HIGH_G_DURN_ADDR		(0X13)//--高G触发中断延时设置
#define BNO055_ACCEL_HIGH_G_THRES_ADDR		(0X14)//--阈值使用高G触发
#define BNO055_ACCEL_NO_MOTION_THRES_ADDR	(0X15)//--用于慢动作或误动作中断的阈值
#define BNO055_ACCEL_NO_MOTION_SET_ADDR		(0X16)//--该寄存器功能取决于是否选择了慢动作或无动作中断
#define BNO055_GYRO_INTR_SETING_ADDR		(0X17)//--选择未经过滤(过滤)的数据用于高速中断
#define BNO055_GYRO_HIGHRATE_X_SET_ADDR		(0X18)//--X轴高速滞后 / 高速率阈值用于陀螺仪X轴
#define BNO055_GYRO_DURN_X_ADDR				(0X19)//--X高速速率持续时间
#define BNO055_GYRO_HIGHRATE_Y_SET_ADDR		(0X1A)//--Y轴高速滞后 / 高速率阈值用于陀螺仪Y轴
#define BNO055_GYRO_DURN_Y_ADDR				(0X1B)//--Y高速速率持续时间
#define BNO055_GYRO_HIGHRATE_Z_SET_ADDR		(0X1C)//--Z轴高速滞后 / 高速率阈值用于陀螺仪z轴
#define BNO055_GYRO_DURN_Z_ADDR				(0X1D)//--z高速速率持续时间
#define BNO055_GYRO_ANY_MOTION_THRES_ADDR	(0X1E)//--任何运动阈值都是陀螺仪的任何运动中断
#define BNO055_GYRO_ANY_MOTION_SET_ADDR		(0X1F)//--

/* Power mode*///--电源模式
#define BNO055_POWER_MODE_NORMAL	        (0X00)
#define BNO055_POWER_MODE_LOWPOWER	        (0X01)
#define BNO055_POWER_MODE_SUSPEND	        (0X02)
/*Euler division factor*/  //--欧拉除法因子   详细见3.6.5.4节表3-29         
#define BNO055_EULER_DIV_DEG		        (16.0)//-- 1 DPS =16 LSB
#define BNO055_ACC_DIV_DEG		            (100.0)//-- 1m/s2 = 100 LSB
#define BNO055_EULER_DIV_RAD		        (900.0)//-- 1 RPS = 900 LSB
/* Operation mode settings*///--操作模式设置见数据手册3.3节表3-5
#define BNO055_OPERATION_MODE_CONFIG		(0X00)//--配置模式
#define BNO055_OPERATION_MODE_ACCONLY		(0X01)//--非融合模式
#define BNO055_OPERATION_MODE_MAGONLY		(0X02)//--非融合模式
#define BNO055_OPERATION_MODE_GYRONLY		(0X03)//--非融合模式
#define BNO055_OPERATION_MODE_ACCMAG		(0X04)//--非融合模式
#define BNO055_OPERATION_MODE_ACCGYRO		(0X05)//--非融合模式
#define BNO055_OPERATION_MODE_MAGGYRO		(0X06)//--非融合模式
#define BNO055_OPERATION_MODE_AMG			(0X07)//--非融合模式
#define BNO055_OPERATION_MODE_IMUPLUS		(0X08)//--融合模式
#define BNO055_OPERATION_MODE_COMPASS		(0X09)//--融合模式
#define BNO055_OPERATION_MODE_M4G			(0X0A)//--融合模式
#define BNO055_OPERATION_MODE_NDOF_FMC_OFF	(0X0B)//--融合模式
#define BNO055_OPERATION_MODE_NDOF			(0X0C)//--融合模式


typedef struct{
   int16_t x;
   int16_t y;
   int16_t z;  
}Axis_sTypeDef;//--轴

typedef struct{
 Axis_sTypeDef  acc_offset;//--加速度设置
 Axis_sTypeDef  mag_offset;//--磁力计设置
 Axis_sTypeDef  gyr_offset;//--陀螺仪设置
           int16_t  acc_radius;
           int16_t  mag_radius;
}CalOut_sTypeDef;


CalOut_sTypeDef  calData;

ST_CalibValue cal;

//*****rewrite Section---start*************
//#define FLASH_USED_ADDRESS  (0x70000)
//#define FLASH_CHECK_VALUE   (0X3456)             
//extern void erase_user_flash(void); 
//extern void write_data(uint32_t flash_address,uint32_t * flash_data_buf, uint32_t count);
//extern void delay_ms(unsigned int ms);
//                        
///*save calibration data to flash*/
//uint8_t bno055_SaveData(void)
//{    
//    
//    union{
//        struct{
//          uint16_t  chk;
//           int16_t  acc_x;            
//           int16_t  acc_y;            
//           int16_t  acc_z;            
//           int16_t  gyr_x;            
//           int16_t  gyr_y;            
//           int16_t  gyr_z;            
//           int16_t  acc_rad;            
//        }value;
//        
//        uint8_t buf[16];
//    }data;
//    
//    data.value.chk     =  FLASH_CHECK_VALUE;
//    data.value.acc_x   =  cal.acc_offset.data_x;
//    data.value.acc_y   =  cal.acc_offset.data_y;
//    data.value.acc_z   =  cal.acc_offset.data_z;
//    data.value.gyr_x   =  cal.gyr_offset.data_x;
//    data.value.gyr_y   =  cal.gyr_offset.data_y;
//    data.value.gyr_z   =  cal.gyr_offset.data_z;
//    data.value.acc_rad =  cal.acc_radius;
//    
//    erase_user_flash();    
//    write_data(FLASH_USED_ADDRESS,(uint32_t *)(&(data.buf[0])),256);   
//    return 0;
//}

///*get calibration data saved to flash*/
//uint8_t bno055_GetData(void)
//{    
//    if(*(uint16_t *)(FLASH_USED_ADDRESS) != FLASH_CHECK_VALUE)
//        return 1;
//    
//    cal.acc_offset.data_x = *(int16_t *)(FLASH_USED_ADDRESS +  2);
//    cal.acc_offset.data_y = *(int16_t *)(FLASH_USED_ADDRESS +  4);
//    cal.acc_offset.data_z = *(int16_t *)(FLASH_USED_ADDRESS +  6);
//    cal.gyr_offset.data_x = *(int16_t *)(FLASH_USED_ADDRESS +  8);
//    cal.gyr_offset.data_y = *(int16_t *)(FLASH_USED_ADDRESS + 10);
//    cal.gyr_offset.data_z = *(int16_t *)(FLASH_USED_ADDRESS + 12);
//    cal.acc_radius        = *(int16_t *)(FLASH_USED_ADDRESS + 14);  
//    
//    return 0;
//}
//*****rewrite Section---end*************                        

/*config bno055*/ 
uint8_t Bno055_Config(uint8_t position,uint8_t writeFlag)
{
    uint8_t regCmdCh1,regCmdCh2;
    uint8_t system_statues;
	int16_t retry = 0;
    while(1)
    {
			//--读取系统启动状态寄存器
		if(HAL_I2C_Mem_Read(&hi2c1, //--哪个I2C
	                      BNO055_I2C_ADDR2, //--设备地址
	                      BNO055_SYS_STAT_ADDR, //--寄存器地址
	                      I2C_MEMADD_SIZE_8BIT, //--单个字节长度
                   	    &system_statues, //--存储地址
	                      1, //--读取长度
                   	    0xff))//--超时长度
            return ERROR_I2C_READ;

        switch(system_statues)//--查看系统状态
        {
        case SYSTEM_IDLE: //--系统空闲
                 if(0 == position)
                 {
                     regCmdCh1 = 0X21; //--0010 0100  坐标系方向
                     regCmdCh2 = 0X04; //--0000 0011  轴正方向
                 }
                 else if(1 == position)
                 {
                     regCmdCh1 = 0X24;//--0010 0001   坐标系
                     regCmdCh2 = 0X00;//--0000 0001   轴正方向 
                 }
                 else if(2 == position)
                 {
                     regCmdCh1 = 0X24;//--0010 0001   坐标系
                     regCmdCh2 = 0X06;//--0000 0001   轴正方向 
                 }
                 else if(3 == position)
                 {
                     regCmdCh1 = 0X21;//--0010 0001   坐标系
                     regCmdCh2 = 0X02;//--0000 0001   轴正方向 
                 }
                 else if(4 == position)
                 {
                     regCmdCh1 = 0X24;//--0010 0001   坐标系
                     regCmdCh2 = 0X03;//--0000 0001   轴正方向 
                 }
                 else if(5 == position)
                 {
                     regCmdCh1 = 0X21;//--0010 0001   坐标系
                     regCmdCh2 = 0X01;//--0000 0001   轴正方向 
                 }
                 else if(6 == position)
                 {
                     regCmdCh1 = 0X21;//--0010 0001   坐标系
                     regCmdCh2 = 0X07;//--0000 0001   轴正方向 
                 }
                 else if(7 == position)
                 {
                     regCmdCh1 = 0X24;//--0010 0001   坐标系
                     regCmdCh2 = 0X05;//--0000 0001   轴正方向 
                 }

				 //--重新配置坐标系参考轴
				 //--默认：x x x x x x x x  AXIS_MAP_CONFIG
				 //--      x x 1 0 0 1 0 0   0X24                            
				 //--          --- --- ---
				 //--           Z   Y   X   芯片1脚小圆点处，长轴方向是Y轴，短轴方向是X轴
				 //--      x x 1 0 0 0 0 1  0X21
         //--          --- --- ---
         //--		    Z	X	Y 
								 
								 
		         //--默认：X X X X X X X X  AXIS_MAP_SIGN
				 //--      X X X X X 0 0 0       0:小圆点默认左上角  X轴正方向朝前  Y轴正方向朝左  Z轴始终朝上
				 //--  0：轴朝向是默然方向  1：轴朝向是默认方向的反方向   						 
 if(HAL_I2C_Mem_Write(&hi2c1, //--哪个I2C
	                      BNO055_I2C_ADDR2, //--设备地址
	                      BNO055_AXIS_MAP_CONFIG_ADDR, //--寄存器地址
	                      I2C_MEMADD_SIZE_8BIT, //--单个字节长度
                   	     &regCmdCh1, //--存储地址
	                      1, //--写长度
                   	    0xff))//--超时长度        
       return ERROR_I2C_WRITE;
					
     
 if(HAL_I2C_Mem_Write(&hi2c1, //--哪个I2C
	                      BNO055_I2C_ADDR2, //--设备地址
	                      BNO055_AXIS_MAP_SIGN_ADDR, //--寄存器地址
	                      I2C_MEMADD_SIZE_8BIT, //--单个字节长度
                   	     &regCmdCh2, //--存储地址
	                      1, //--写长度
                   	    0xff))//--超时长度          
 				return ERROR_I2C_WRITE;
					
          //--BNO055支持三种不同的电源模式：正常模式，低功耗模式， 挂起模式
				 //--可以通过写入PWR_MODE寄存器来选择电源模式，默认为正常模式
         regCmdCh1 = BNO055_POWER_MODE_NORMAL;//--详情看手册
 if(HAL_I2C_Mem_Write(&hi2c1, //--哪个I2C
	                      BNO055_I2C_ADDR2, //--设备地址
	                      BNO055_PWR_MODE_ADDR, //--寄存器地址
	                      I2C_MEMADD_SIZE_8BIT, //--单个字节长度
                   	     &regCmdCh1, //--存储地址
	                      1, //--写长度
                   	    0xff))//--超时长度   
          return ERROR_I2C_WRITE;
                 
				 //--传感器校准数据   //--芯片出厂已经校准过了，可以考虑忽略这部分代码
         if(writeFlag)//write calibration data(acc and gyr)to bno055//--详情见3.6.4
             { 
							 //--加速度补偿
							 if(HAL_I2C_Mem_Write(&hi2c1, //--哪个I2C
	                      BNO055_I2C_ADDR2, //--设备地址
	                      BNO055_ACCEL_OFFSET_X_LSB_ADDR, //--寄存器地址
	                      I2C_MEMADD_SIZE_8BIT, //--单个字节长度
                   	    (uint8_t *)(&(cal.acc_offset)), //--存储地址
	                      6, //--写长度
                   	    0xff))//--超时长度          
                    return ERROR_I2C_WRITE; 
						  //--陀螺仪补偿	 
				  if(HAL_I2C_Mem_Write(&hi2c1, //--哪个I2C
	                      BNO055_I2C_ADDR2, //--设备地址
	                      BNO055_GYRO_OFFSET_X_LSB_ADDR, //--寄存器地址
	                      I2C_MEMADD_SIZE_8BIT, //--单个字节长度
                   	    (uint8_t *)(&(cal.gyr_offset)), //--存储地址
	                      6, //--写长度
                   	    0xff))//--超时长度 
                     return ERROR_I2C_WRITE;  
					  //--加速度半径
					 if(HAL_I2C_Mem_Write(&hi2c1, //--哪个I2C
	                      BNO055_I2C_ADDR2, //--设备地址
	                      BNO055_ACCEL_RADIUS_LSB_ADDR, //--寄存器地址
	                      I2C_MEMADD_SIZE_8BIT, //--单个字节长度
                   	    (uint8_t *)(&(cal.acc_radius)), //--存储地址
	                      2, //--写长度
                   	    0xff))//--超时长度 
                     return ERROR_I2C_WRITE;                      
              }
                 
				 //--在IMU模式中，BNO055在空间中的相对方向是根据加速度计和陀螺仪数据计算的。 计算很快（即高输出数据速率）
    
							//regCmdCh1 = BNO055_OPERATION_MODE_IMUPLUS;//--IMU模式
								regCmdCh1 = BNO055_OPERATION_MODE_NDOF;//--
	if(HAL_I2C_Mem_Write(&hi2c1, //--哪个I2C
	                      BNO055_I2C_ADDR2, //--设备地址
	                      BNO055_OPR_MODE_ADDR, //--寄存器地址
	                      I2C_MEMADD_SIZE_8BIT, //--单个字节长度
                   	     &regCmdCh1, //--存储地址
	                      1, //--写长度
                   	    0xff))//--超时长度   
               return ERROR_I2C_WRITE;
                 
              return NO_ERROR;
              //break;
        case SYSTEM_ERROR:  //--系统错误

                regCmdCh1 = 0X20;
	if(HAL_I2C_Mem_Write(&hi2c1, //--哪个I2C
	                      BNO055_I2C_ADDR2, //--设备地址
	                      BNO055_SYS_TRIGGER_ADDR, //--寄存器地址
	                      I2C_MEMADD_SIZE_8BIT, //--单个字节长度
                   	     &regCmdCh1, //--存储地址
	                      1, //--写长度
                   	    0xff))//--超时长度   
                    return ERROR_I2C_WRITE;
                break;
        case INITIALIZING_PERIPHERALS: //--初始化外围设备
                break;
        case SYSTEM_INITIALIZATION: //--自检失败 
                break;
        case EXECUTING_SELFTEST:   //--注册寄存器地图值超出范围				
                break;	
        case SENSOR_FUSION_ALGORITHM_RUN: //--注册寄存器地图地址超出范围
                return NO_ERROR;
        case SYSTEM_RUN_WITHOUT_FUSION_ALGORITHM: //--注册映射写入错误
                break;
        default:     
                return ERROR_UNKNOWN; //--其他错误（还有3种错误）
                                                     
        }
        
        retry++;//--没有读到正确的系统状态，说明有可能模块还没准备好，继续重读
        if(retry>CHECK_NUM)
        {
            return ERROR_RETRY_OVER;//--重读超时
        }
	 }
}
                        
                        
/**
 * @brief Initialize bno055, set operater register in fusion IMU mode.
 */ 
uint8_t Bno055_Init(uint8_t position)
{     
 // return bno055_config(position,1);
	
//	return Bno055_Config(position,0);
	return Bno055_ConfigCalib(position);
}

 
/*配置校准模式*/ 

uint8_t Bno055_ConfigCalib(uint8_t position)
{ 
    uint8_t regCmdCh;
    
    //复位 bno055(可以使用复位脚去复位 bno055)  
    regCmdCh = 0X20;//--0010 0000   //--0x3f地址
		if(HAL_I2C_Mem_Write(&hi2c1, //--哪个I2C
	                      BNO055_I2C_ADDR2, //--设备地址
	                      BNO055_SYS_TRIGGER_ADDR, //--寄存器地址
	                      I2C_MEMADD_SIZE_8BIT, //--单个字节长度
                   	     &regCmdCh, //--存储地址
	                      1, //--写长度
                   	    0xf))//--超时长度   
        return ERROR_I2C_WRITE;
    
    //delay---need to rewrite
    HAL_Delay(5000);
    
    //config bno055
    return Bno055_Config(position,0);
}

/*get bno055 calibration status*/ //--获取bno055校准状态
uint8_t Bno055_Calibrate(void)
{ ST_CalibSta *status;
    uint8_t regCmdCh;
	 //--0x35 获取当前校准状态
    if(HAL_I2C_Mem_Read(&hi2c1, //--哪个I2C
	                      BNO055_I2C_ADDR2, //--设备地址
	                      BNO055_CALIB_STAT_ADDR, //--寄存器地址
	                      I2C_MEMADD_SIZE_8BIT, //--单个字节长度
                   	    (uint8_t*)(status), //--存储地址
	                      1, //--读取长度
                   	    0xff))//--超时长度
        return ERROR_I2C_READ;
    
    if((status->CalibStaACC<3)||(status->CalibStaGYR<3))//--小于3，没有完全校准
        return CALIBRATING;//--需要校准
    
    //config bno055 config-mode     //--选择CONIG_MODE的操作模式
	  //--BNO055_OPR_MODE_ADDR 选择操作模式
	  //--此模式用于配置BNO055，其中所有输出数据重置为零并且传感器融合停止。 这是唯一可以更改
    //-所有可写寄存器映射条目的模式。 （此规则的例外情况是中断寄存器（INT和INT_MSK）和
    //--操作模式寄存器（OPR_MODE），可以在任何操作模式下修改。）
    //-如上所述，此模式是上电或复位后的默认操作模式。 必须选择任何其他模式才能读取任何传感器数据
    regCmdCh = 0;//--0为配置模式
		//--0x3D  //--进入配置模式
		if(HAL_I2C_Mem_Write(&hi2c1, //--哪个I2C
	                      BNO055_I2C_ADDR2, //--设备地址
	                      BNO055_OPR_MODE_ADDR, //--寄存器地址
	                      I2C_MEMADD_SIZE_8BIT, //--单个字节长度
                   	     &regCmdCh, //--存储地址
	                      1, //--写长度
                   	    0xf))//--超时长度   
        return ERROR_I2C_WRITE;
    
    //delay---need to rewrite 
   HAL_Delay(50); //--需要时间去修改，等一会
     
     //--读取校准的acc偏移量数据
	
	   //--配置加速度计偏移需要6个字节（3轴
     //--X，Y和Z中的每一个都有2个字节）。 仅当用户写入最后一个字节（即ACC_OFFSET_Z_MSB）时才会进行配置
     //--连续读6个字节
		if(HAL_I2C_Mem_Read(&hi2c1, //--哪个I2C
	                      BNO055_I2C_ADDR2, //--设备地址
	                      BNO055_ACCEL_OFFSET_X_LSB_ADDR, //--寄存器地址
	                      I2C_MEMADD_SIZE_8BIT, //--单个字节长度
                   	    (uint8_t*)(&cal.acc_offset), //--存储地址
	                      6, //--读取长度
                   	    0xff))//--超时长度
        return ERROR_I2C_READ;
    
     //--读取校准的GyR偏移数据
			if(HAL_I2C_Mem_Read(&hi2c1, //--哪个I2C
	                      BNO055_I2C_ADDR2, //--设备地址
	                      BNO055_GYRO_OFFSET_X_LSB_ADDR, //--寄存器地址
	                      I2C_MEMADD_SIZE_8BIT, //--单个字节长度
                   	   (uint8_t*)(&cal.gyr_offset), //--存储地址
	                      6, //--读取长度
                   	    0xff))//--超时长度
        return ERROR_I2C_READ;

     //--读取校准的acc半径数据
			if(HAL_I2C_Mem_Read(&hi2c1, //--哪个I2C
	                      BNO055_I2C_ADDR2, //--设备地址
	                      BNO055_ACCEL_OFFSET_X_LSB_ADDR, //--寄存器地址
	                      I2C_MEMADD_SIZE_8BIT, //--单个字节长度
                   	   (uint8_t*)(&cal.acc_radius), //--存储地址
	                      2, //--读取长度
                   	    0xff))//--超时长度
        return ERROR_I2C_READ;
    
    
    return CALIB_SUCCESS;
}
float Read_Bno055_Yaw(void)
{
  float pitch,roll,yaw;
	Bno055_Get_Data(&pitch,&roll,&yaw);
	return yaw;
}

/*读取欧拉角数据*/
uint8_t Bno055_Get_Data(float *pitch,float *roll,float *yaw)
{ 
	
    //ST_NDOF st_ndof ;
	 if(HAL_I2C_Mem_Read(&hi2c1, //--哪个I2C
	                      BNO055_I2C_ADDR2, //--设备地址
	                      BNO055_EULER_H_LSB_ADDR, //--寄存器地址
	                      I2C_MEMADD_SIZE_8BIT, //--单个字节长度
                   	    (uint8_t*)&st_ndof, //--存储地址
	                      sizeof(ST_NDOF), //--读取长度
                   	    0xff))//--超时长度
        return ERROR_I2C_READ;
                    
    *pitch = st_ndof.pitch/ BNO055_EULER_DIV_DEG;   //--1 RPS = 900 LSB
    *roll  = st_ndof.roll/  BNO055_EULER_DIV_DEG;
    *yaw   = st_ndof.heading/ BNO055_EULER_DIV_DEG;
    
    return NO_ERROR;
}
/*读取加速度数据*/
uint8_t Bno055_Get_Acc_Data(float *x_axis,float *y_axis,float *z_axis)
{ 	
    //ST_SENSOR st_acc ; 
	  if(HAL_I2C_Mem_Read(&hi2c1, //--哪个I2C
	                      BNO055_I2C_ADDR2, //--设备地址
	                      BNO055_ACCEL_DATA_X_LSB_ADDR, //--寄存器地址
	                      I2C_MEMADD_SIZE_8BIT, //--单个字节长度
                   	    (uint8_t*)&st_acc, //--存储地址
	                      sizeof(ST_SENSOR), //--读取长度
                   	    0xff))//--超时长度
        return ERROR_I2C_READ;
                        
    *x_axis = st_acc.data_x/ BNO055_ACC_DIV_DEG; //--1m/s2 = 100 LSB
    *y_axis = st_acc.data_y/ BNO055_ACC_DIV_DEG;
    *z_axis = st_acc.data_z/ BNO055_ACC_DIV_DEG;

    return NO_ERROR;
}

/*读取磁力计数据*/
uint8_t Bno055_Get_Mag_Data(float *x_axis,float *y_axis,float *z_axis)
{ 	
   // ST_SENSOR st_mag ; 
   if(HAL_I2C_Mem_Read(&hi2c1, //--哪个I2C
	                      BNO055_I2C_ADDR2, //--设备地址
	                      BNO055_ACCEL_DATA_X_LSB_ADDR, //--寄存器地址
	                      I2C_MEMADD_SIZE_8BIT, //--单个字节长度
                   	    (uint8_t*)&st_mag, //--存储地址
	                      sizeof(ST_SENSOR), //--读取长度
                   	    0xff))//--超时长度  
        return ERROR_I2C_READ;
                    
    *x_axis = st_mag.data_x/ BNO055_EULER_DIV_DEG; //--1 RPS = 900 LSB
    *y_axis = st_mag.data_y/ BNO055_EULER_DIV_DEG;
    *z_axis = st_mag.data_z/ BNO055_EULER_DIV_DEG;

    return NO_ERROR;
}

/*读取角速度数据*/
uint8_t Bno055_Get_Gyo_Data(float *x_axis,float *y_axis,float *z_axis)
{ 
	
    //ST_SENSOR st_gyo ;  
   if(HAL_I2C_Mem_Read(&hi2c1, //--哪个I2C
	                      BNO055_I2C_ADDR2, //--设备地址
	                      BNO055_GYRO_DATA_X_LSB_ADDR, //--寄存器地址
	                      I2C_MEMADD_SIZE_8BIT, //--单个字节长度
                   	    (uint8_t*)&st_gyo, //--存储地址
	                      sizeof(ST_SENSOR), //--读取长度
                   	    0xff))//--超时长度  
         return ERROR_I2C_READ;                    
                        
    *x_axis = st_gyo.data_x/ BNO055_EULER_DIV_DEG; //--1 RPS = 900 LSB
    *y_axis = st_gyo.data_y/ BNO055_EULER_DIV_DEG;
    *z_axis = st_gyo.data_z/ BNO055_EULER_DIV_DEG;

    return NO_ERROR;
}


uint8_t Bno005_Err_Display(uint8_t err_num)
{
switch(err_num)
{
	case NO_ERROR:
	//printf("-->NO_ERROR\r\n");
	return NO_ERROR;
  //break;		
  case ERROR_I2C_READ:
  //printf("-->ERROR_I2C_READ\r\n");
	return ERROR_I2C_READ;
  //break;	
  case ERROR_I2C_WRITE:
 // printf("-->ERROR_I2C_WRITE\r\n");
	return ERROR_I2C_WRITE;
 // break;	
	case ERROR_INPUT_PARA:
 //printf("-->ERROR_INPUT_PARA\r\n");
	return ERROR_INPUT_PARA;
 // break;	
  case ERROR_RETRY_OVER:
  //printf("-->ERROR_RETRY_OVER\r\n");
	return ERROR_RETRY_OVER;
 // break;	
	case ERROR_UNKNOWN:
  //printf("-->ERROR_UNKNOWN\r\n");
	return ERROR_UNKNOWN;
 // break;	
  case  CALIBRATING:
 //printf("-->CALIBRATING\r\n");
	return CALIBRATING;
 // break;	
         
}
return 0;
}


/*-- File end --*/
