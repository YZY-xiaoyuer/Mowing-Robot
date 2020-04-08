#ifndef __BNO055_H_
#define __BNO055_H_


/* Include Section ----------------------------------------------------*/
#include "i2c.h"
#include "stdio.h"
/* define Section -----------------------------------------------------*/
/* return error code*/
#define NO_ERROR           (0)
#define ERROR_I2C_READ     (1)
#define ERROR_I2C_WRITE    (2)
#define ERROR_INPUT_PARA   (3)
#define ERROR_RETRY_OVER   (4)
#define ERROR_UNKNOWN      (5)

#define CALIB_SUCCESS      (0)
#define CALIBRATING        (6)
#define CALIB_SAVE_ERO     (7)

/* data define Section ------------------------------------------------*/
typedef struct {
    int16_t  heading;
    int16_t  roll;
    int16_t  pitch;
}ST_NDOF;	

typedef struct {
	  int16_t  data_x;
    int16_t  data_y;
    int16_t  data_z;
}ST_SENSOR;	

typedef struct{
    uint8_t  CalibStaMAG:2;//--位域
    uint8_t  CalibStaACC:2;
    uint8_t  CalibStaGYR:2;
    uint8_t  CalibStaSYS:2;
}ST_CalibSta;//--一个字节八位，每个变量只占了两位

typedef struct{
  ST_SENSOR  acc_offset;
  ST_SENSOR  gyr_offset;  
    int16_t  acc_radius;
}ST_CalibValue;


typedef struct _BNO055_handle{
/*config bno055*/ 
uint8_t (*bno055_config)(uint8_t position,uint8_t writeFlag);
uint8_t (*bno055_init)(uint8_t position);
/*配置校准模式*/ 
uint8_t (*bno055_configCalib)(uint8_t position);
/*get bno055 calibration status*/ //--获取bno055校准状态
uint8_t (*bno055_calibrate)(void);
/*读取偏航角*/
float (*read_bno055_yaw)(void);
/*读取欧拉角数据*/
uint8_t (*bno055_get_data)(float *pitch,float *roll,float *yaw);
/*读取角速度数据*/
uint8_t (*bno055_get_acc_data)(float *x_axis,float *y_axis,float *z_axis);
/*读取磁力计数据*/
uint8_t (*bno055_get_mag_data)(float *x_axis,float *y_axis,float *z_axis);
/*读取陀螺仪数据*/
uint8_t (*bno055_get_gyo_data)(float *x_axis,float *y_axis,float *z_axis);
uint8_t (*bno005_err_display)(uint8_t err_num);
	
}ST_bno055_handle,*ST_bon055_handle_p;
extern ST_bno055_handle bno055_handle;
extern ST_bon055_handle_p bon055_handle_p;



#endif
