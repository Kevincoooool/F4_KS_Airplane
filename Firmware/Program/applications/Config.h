#ifndef _config_H
#define _config_H

#include "stm32f4xx.h"

#define USE_USB_HID
#define USE_NRF24l01
#define USE_WIFI
#define ACC_CAL_EN

#define OFFSET_AV_NUM 10
#define ACC_ADJ_EN //允许校准加速度计

#define MOTOR_NUM 4
#define MAX_ANGLE     25.0f

#define MAX_SPEED_ROL 200  //角度每秒
#define MAX_SPEED_PIT 200  //角度每秒
#define MAX_SPEED_YAW 250  //角度每秒

#define MAX_ROLLING_SPEED 1600  //角度每秒

#define MAX_SPEED 50 //最大水平速度，厘米每秒 cm/s

#define MAX_Z_SPEED_UP 350 //厘米每秒 cm/s
#define MAX_Z_SPEED_DW 250 //厘米每秒 cm/s
#define MAX_EXP_XY_ACC   500 //厘米每平方秒 cm/ss

#define CTRL_1_INTE_LIM 250 //角速度环积分限幅 ：输出

#define FINAL_P 			0.1f  //电机输出量比例
#define ANGULAR_VELOCITY_PID_INTE_D_LIM 300/FINAL_P  
#define X_PROPORTION_X_Y 1.0f //proportion
#define ROLL_ANGLE_KP 10.0f   //翻滚角度kp

#define MAX_THR_SET    85  //最大油门百分比 %
#define THR_INTE_LIM_SET   70  //油门积分百分比 % 

#define MAX_THR       MAX_THR_SET/FINAL_P   
#define THR_INTE_LIM   THR_INTE_LIM_SET/0.3  

#define THR_START      30  //油门起调量百分比 %

#define LAND_ACC              500  //着陆加速度检测
#define LAND_ACC_DELTA        300  //着陆加速度变化量检测


#define BARO_FIX -0                          //气压速度积分修正起调值/CM厘米
#endif
