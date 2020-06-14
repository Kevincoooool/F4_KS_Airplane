
#include "drv_icm20689.h"
#include "drv_spi.h"
#include "mymath.h"
#include "drv_led.h"
#include "Filter.h"
#include "imu.h"
#include "stdlib.h"
#include <stdio.h>
#include "LevenbergMarquardt.h"
#define MPU_RA_XG_OFFS_TC       0x00    //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_YG_OFFS_TC       0x01    //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_ZG_OFFS_TC       0x02    //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_RA_X_FINE_GAIN      0x03    //[7:0] X_FINE_GAIN
#define MPU_RA_Y_FINE_GAIN      0x04    //[7:0] Y_FINE_GAIN
#define MPU_RA_Z_FINE_GAIN      0x05    //[7:0] Z_FINE_GAIN
#define MPU_RA_XA_OFFS_H        0x06    //[15:0] XA_OFFS
#define MPU_RA_XA_OFFS_L_TC     0x07
#define MPU_RA_YA_OFFS_H        0x08    //[15:0] YA_OFFS
#define MPU_RA_YA_OFFS_L_TC     0x09
#define MPU_RA_ZA_OFFS_H        0x0A    //[15:0] ZA_OFFS
#define MPU_RA_ZA_OFFS_L_TC     0x0B
#define MPU_RA_PRODUCT_ID       0x0C    // Product ID Register
#define MPU_RA_XG_OFFS_USRH     0x13    //[15:0] XG_OFFS_USR
#define MPU_RA_XG_OFFS_USRL     0x14
#define MPU_RA_YG_OFFS_USRH     0x15    //[15:0] YG_OFFS_USR
#define MPU_RA_YG_OFFS_USRL     0x16
#define MPU_RA_ZG_OFFS_USRH     0x17    //[15:0] ZG_OFFS_USR
#define MPU_RA_ZG_OFFS_USRL     0x18
#define MPU_RA_SMPLRT_DIV       0x19
#define MPU_RA_CONFIG           0x1A
#define MPU_RA_GYRO_CONFIG      0x1B
#define MPU_RA_ACCEL_CONFIG     0x1C
#define MPU_RA_ACCEL_CONFIG2    0x1D
#define MPU_RA_FF_DUR           0x1E
#define MPU_RA_MOT_THR          0x1F
#define MPU_RA_MOT_DUR          0x20
#define MPU_RA_ZRMOT_THR        0x21
#define MPU_RA_ZRMOT_DUR        0x22
#define MPU_RA_FIFO_EN          0x23
#define MPU_RA_I2C_MST_CTRL     0x24
#define MPU_RA_I2C_SLV0_ADDR    0x25
#define MPU_RA_I2C_SLV0_REG     0x26
#define MPU_RA_I2C_SLV0_CTRL    0x27
#define MPU_RA_I2C_SLV1_ADDR    0x28
#define MPU_RA_I2C_SLV1_REG     0x29
#define MPU_RA_I2C_SLV1_CTRL    0x2A
#define MPU_RA_I2C_SLV2_ADDR    0x2B
#define MPU_RA_I2C_SLV2_REG     0x2C
#define MPU_RA_I2C_SLV2_CTRL    0x2D
#define MPU_RA_I2C_SLV3_ADDR    0x2E
#define MPU_RA_I2C_SLV3_REG     0x2F
#define MPU_RA_I2C_SLV3_CTRL    0x30
#define MPU_RA_I2C_SLV4_ADDR    0x31
#define MPU_RA_I2C_SLV4_REG     0x32
#define MPU_RA_I2C_SLV4_DO      0x33
#define MPU_RA_I2C_SLV4_CTRL    0x34
#define MPU_RA_I2C_SLV4_DI      0x35
#define MPU_RA_I2C_MST_STATUS   0x36
#define MPU_RA_INT_PIN_CFG      0x37
#define MPU_RA_INT_ENABLE       0x38
#define MPU_RA_DMP_INT_STATUS   0x39
#define MPU_RA_INT_STATUS       0x3A
#define MPU_RA_ACCEL_XOUT_H     0x3B
#define MPU_RA_ACCEL_XOUT_L     0x3C
#define MPU_RA_ACCEL_YOUT_H     0x3D
#define MPU_RA_ACCEL_YOUT_L     0x3E
#define MPU_RA_ACCEL_ZOUT_H     0x3F
#define MPU_RA_ACCEL_ZOUT_L     0x40
#define MPU_RA_TEMP_OUT_H       0x41
#define MPU_RA_TEMP_OUT_L       0x42
#define MPU_RA_GYRO_XOUT_H      0x43
#define MPU_RA_GYRO_XOUT_L      0x44
#define MPU_RA_GYRO_YOUT_H      0x45
#define MPU_RA_GYRO_YOUT_L      0x46
#define MPU_RA_GYRO_ZOUT_H      0x47
#define MPU_RA_GYRO_ZOUT_L      0x48
#define MPU_RA_EXT_SENS_DATA_00 0x49
#define MPU_RA_MOT_DETECT_STATUS    0x61
#define MPU_RA_I2C_SLV0_DO      0x63
#define MPU_RA_I2C_SLV1_DO      0x64
#define MPU_RA_I2C_SLV2_DO      0x65
#define MPU_RA_I2C_SLV3_DO      0x66
#define MPU_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU_RA_SIGNAL_PATH_RESET    0x68
#define MPU_RA_MOT_DETECT_CTRL      0x69
#define MPU_RA_USER_CTRL        0x6A
#define MPU_RA_PWR_MGMT_1       0x6B
#define MPU_RA_PWR_MGMT_2       0x6C
#define MPU_RA_BANK_SEL         0x6D
#define MPU_RA_MEM_START_ADDR   0x6E
#define MPU_RA_MEM_R_W          0x6F
#define MPU_RA_DMP_CFG_1        0x70
#define MPU_RA_DMP_CFG_2        0x71
#define MPU_RA_FIFO_COUNTH      0x72
#define MPU_RA_FIFO_COUNTL      0x73
#define MPU_RA_FIFO_R_W         0x74
#define MPU_RA_WHO_AM_I         0x75

#define MPU_SMPLRT_DIV      0       // 8000Hz

#define BIT_GYRO                    3
#define BIT_ACC                     2
#define BIT_TEMP                    1

#define MPU_LPF_256HZ       0
#define MPU_LPF_188HZ       1
#define MPU_LPF_98HZ        2
#define MPU_LPF_42HZ        3
#define MPU_LPF_20HZ        4
#define MPU_LPF_10HZ        5
#define MPU_LPF_5HZ         6

#define MPU_A_2mg                ((float)0.00006103f)  //g/LSB
#define MPU_A_4mg                ((float)0.00012207f)  //g/LSB
#define MPU_A_8mg                ((float)0.00024414f)  //g/LSB

#define MPU_G_s250dps            ((float)0.0076296f)  //dps/LSB
#define MPU_G_s500dps            ((float)0.0152592f)  //dps/LSB
#define MPU_G_s1000dps           ((float)0.0305185f)  //dps/LSB
#define MPU_G_s2000dps           ((float)0.0610370f)  //dps/LSB


/**********************************************************************************************************
*函 数 名: ICM20689_Detect
*功能说明: 检测ICM20689是否存在
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
bool ICM20689_Detect(void)
{
    uint8_t who_am_i;

    Spi_GyroMultiRead(MPU_RA_WHO_AM_I, &who_am_i, 1);

    if(who_am_i == 0x98)
        return true;
    else
        return false;
}

/**********************************************************************************************************
*函 数 名: ICM20689_Init
*功能说明: ICM20689寄存器配置初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
u8 Drv_Icm20689Reg_Init(void)
{
    Spi_GyroSingleWrite(MPU_RA_PWR_MGMT_1, 0x80);
    Delay_ms(10);

    Spi_GyroSingleWrite(MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
    Delay_ms(10);

    Spi_GyroSingleWrite(MPU_RA_PWR_MGMT_1, 0x00);
    Delay_ms(10);

    Spi_GyroSingleWrite(MPU_RA_USER_CTRL, 0x10);
    Delay_ms(10);

    Spi_GyroSingleWrite(MPU_RA_PWR_MGMT_2, 0x00);
    Delay_us(50);

    //陀螺仪采样率0x00(1000Hz)   采样率 = 陀螺仪的输出率 / (1 + SMPLRT_DIV)
    Spi_GyroSingleWrite(MPU_RA_SMPLRT_DIV, (1000/1000 - 1));
    Delay_us(50);

    //i2c旁路模式
    // INT_PIN_CFG   -- INT_LEVEL_HIGH, INT_OPEN_DIS, LATCH_INT_DIS, INT_RD_CLEAR_DIS, FSYNC_INT_LEVEL_HIGH, FSYNC_INT_DIS, I2C_BYPASS_EN, CLOCK_DIS
    Spi_GyroSingleWrite(MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 1 << 1 | 0 << 0);
    Delay_us(50);

    //低通滤波频率
    Spi_GyroSingleWrite(MPU_RA_CONFIG, MPU_LPF_20HZ);
    Delay_us(50);

    //陀螺仪自检及测量范围，典型值0x18(不自检，2000deg/s) (0x10 1000deg/s) (0x10 1000deg/s) (0x08 500deg/s)
    Spi_GyroSingleWrite(MPU_RA_GYRO_CONFIG, 0x18);
    Delay_us(50);

    //加速度自检、测量范围(不自检，+-8G)
    Spi_GyroSingleWrite(MPU_RA_ACCEL_CONFIG, 2 << 3);
    Delay_us(50);

    //加速度低通滤波设置
    Spi_GyroSingleWrite(MPU_RA_ACCEL_CONFIG2, MPU_LPF_10HZ);

    Delay_ms(5);
	//icm20689_INT_Config();
	/*设置重心相对传感器的偏移量*/
	Center_Pos_Set();
	
	//sensor.acc_z_auto_CALIBRATE = 1; //开机自动对准Z轴
	//sensor.gyr_CALIBRATE = 2;//开机自动校准陀螺仪
	return 1;
}
ACCELEROMETER_t acc;
enum ORIENTATION_STATUS orientationStatus;
/**********************************************************************************************************
*函 数 名: ImuOrientationDetect
*功能说明: 检测传感器放置方向
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ImuOrientationDetect(Vector3f_t acc)
{
    const float CONSTANTS_ONE_G = 4096;
    const float accel_err_thr = 400;

    // [ g, 0, 0 ]
    if (fabsf(acc.x - CONSTANTS_ONE_G) < accel_err_thr &&
            fabsf(acc.y) < accel_err_thr &&
            fabsf(acc.z) < accel_err_thr)
    {
        orientationStatus = ORIENTATION_LEFT;
    }
    // [ -g, 0, 0 ]
    if (fabsf(acc.x + CONSTANTS_ONE_G) < accel_err_thr &&
            fabsf(acc.y) < accel_err_thr &&
            fabsf(acc.z) < accel_err_thr)
    {
        orientationStatus = ORIENTATION_RIGHT;
    }
    // [ 0, g, 0 ]
    if (fabsf(acc.x) < accel_err_thr &&
            fabsf(acc.y - CONSTANTS_ONE_G) < accel_err_thr &&
            fabsf(acc.z) < accel_err_thr)
    {
        orientationStatus = ORIENTATION_FRONT;
    }
    // [ 0, -g, 0 ]
    if (fabsf(acc.x) < accel_err_thr &&
            fabsf(acc.y + CONSTANTS_ONE_G) < accel_err_thr &&
            fabsf(acc.z) < accel_err_thr)
    {
        orientationStatus = ORIENTATION_BACK;
    }
    // [ 0, 0, g ]
    if (fabsf(acc.x) < accel_err_thr &&
            fabsf(acc.y) < accel_err_thr &&
            fabsf(acc.z - CONSTANTS_ONE_G) < accel_err_thr)
    {
        orientationStatus = ORIENTATION_UP;
    }
    // [ 0, 0, -g ]
    if (fabsf(acc.x) < accel_err_thr &&
            fabsf(acc.y) < accel_err_thr &&
            fabsf(acc.z + CONSTANTS_ONE_G) < accel_err_thr)
    {
        orientationStatus = ORIENTATION_DOWN;
    }
}
/**********************************************************************************************************
*函 数 名: GetImuOrientation
*功能说明: 获取传感器放置状态
*形    参: 无
*返 回 值: 状态
**********************************************************************************************************/
enum ORIENTATION_STATUS GetImuOrientation(void)
{
    return orientationStatus;
}
/**********************************************************************************************************
*函 数 名: AccCalibration
*功能说明: 加速度校准
*形    参: 加速度原始数据
*返 回 值: 无
**********************************************************************************************************/  
Vector3f_t new_offset;
Vector3f_t new_scale;
Vector3f_t samples[6];
void AccCalibration(Vector3f_t accRaw)
{
    static uint16_t samples_count = 0;
    static uint8_t orientationCaliFlag[6];
    static uint8_t currentOrientation;

    //static Vector3f_t samples[6];
    static uint8_t caliFlag = 0;
    static uint32_t caliCnt = 0;

   if(sensor.acc_CALIBRATE == 0)
       return;

    /*********************************检测IMU放置方向************************************/
    if(GetImuOrientation() == ORIENTATION_UP && !orientationCaliFlag[ORIENTATION_UP])
    {
        //判断IMU是否处于静止状态
        if(flag.motionless == 1)
            caliCnt++;
        else
            caliCnt = 0;

        if(caliCnt > 1000)
        {
            caliFlag = 1;
            orientationCaliFlag[ORIENTATION_UP] = 1;
            samples_count = 0;
            acc.cali.step++;
            currentOrientation = ORIENTATION_UP;
            //mavlink发送检测提示
           // KS_DT_SendString("Up!Please wait 3s!");
        }
    }

    if(GetImuOrientation() == ORIENTATION_DOWN && !orientationCaliFlag[ORIENTATION_DOWN])
    {
        //判断IMU是否处于静止状态
        if(flag.motionless == 1)
            caliCnt++;
        else
            caliCnt = 0;

        if(caliCnt > 1000)
        {
            caliFlag = 1;
            orientationCaliFlag[ORIENTATION_DOWN] = 1;
            samples_count = 0;
            acc.cali.step++;
            currentOrientation = ORIENTATION_DOWN;
            //mavlink发送检测提示
           // KS_DT_SendString("Down!  Please wait 3s!");
        }
    }

    if(GetImuOrientation() == ORIENTATION_FRONT && !orientationCaliFlag[ORIENTATION_FRONT])
    {
        //判断IMU是否处于静止状态
        if(flag.motionless == 1)
            caliCnt++;
        else
            caliCnt = 0;

        if(caliCnt > 1000)
        {
            caliFlag = 1;
            orientationCaliFlag[ORIENTATION_FRONT] = 1;
            samples_count = 0;
            acc.cali.step++;
            currentOrientation = ORIENTATION_FRONT;
            //mavlink发送检测提示
            //KS_DT_SendString("Front! Please wait 3s!");
        }
    }

    if(GetImuOrientation() == ORIENTATION_BACK && !orientationCaliFlag[ORIENTATION_BACK])
    {
        //判断IMU是否处于静止状态
        if(flag.motionless == 1)
            caliCnt++;
        else
            caliCnt = 0;

        if(caliCnt > 1000)
        {
            caliFlag = 1;
            orientationCaliFlag[ORIENTATION_BACK] = 1;
            samples_count = 0;
            acc.cali.step++;
            currentOrientation = ORIENTATION_BACK;
            //mavlink发送检测提示
           // KS_DT_SendString("Back! Please wait 3s!");
        }
    }

    if(GetImuOrientation() == ORIENTATION_LEFT && !orientationCaliFlag[ORIENTATION_LEFT])
    {
        //判断IMU是否处于静止状态
        if(flag.motionless == 1)
            caliCnt++;
        else
            caliCnt = 0;

        if(caliCnt > 1000)
        {
            caliFlag = 1;
            orientationCaliFlag[ORIENTATION_LEFT] = 1;
            samples_count = 0;
            acc.cali.step++;
            currentOrientation = ORIENTATION_LEFT;
            //mavlink发送检测提示
            //KS_DT_SendString("Left! Please wait 3s!");
        }
    }

    if(GetImuOrientation() == ORIENTATION_RIGHT && !orientationCaliFlag[ORIENTATION_RIGHT])
    {
        //判断IMU是否处于静止状态
        if(flag.motionless == 1)
            caliCnt++;
        else
            caliCnt = 0;

        if(caliCnt > 1000)
        {
            caliFlag = 1;
            orientationCaliFlag[ORIENTATION_RIGHT] = 1;
            samples_count = 0;
            acc.cali.step++;
            currentOrientation = ORIENTATION_RIGHT;
            //mavlink发送检测提示
            //KS_DT_SendString("Right! Please wait 3s!");
        }
    }
    /****************************************************************************************/

    //分别采集加速度计六个方向的数据，顺序随意，每个方向取500个样本求平均值
    if(caliFlag)
    {
//			char progress[20];
//						sprintf(progress, "%d", (int)(samples_count * 1.6f)); 
        if(samples_count < 500)
        {
            samples[acc.cali.step - 1].x += accRaw.x;
            samples[acc.cali.step - 1].y += accRaw.y;
            samples[acc.cali.step - 1].z += accRaw.z;
            samples_count++;
        }
        else if(samples_count == 500)
        {
            samples[acc.cali.step - 1].x /= 500;
            samples[acc.cali.step - 1].y /= 500;
            samples[acc.cali.step - 1].z /= 500;
            samples_count++;

            caliFlag = 0;
            caliCnt  = 0;

            //bsklink发送当前校准步骤
            //MessageSensorCaliFeedbackEnable(ACC, acc.cali.step, acc.cali.success);
            
            //mavlink发送当前校准步骤
            switch(currentOrientation)
            {
            case ORIENTATION_UP:
                
               // sprintf(progress, "%d", (int)(acc.cali.step * 1.6f)); 
                //KS_DT_SendString("UP Done!");
                break;
            case ORIENTATION_DOWN:
                //sprintf(progress, "%d", (int)(acc.cali.step * 1.6f)); 
                // KS_DT_SendString("DOWN Done!");
                break;
            case ORIENTATION_FRONT:
                //sprintf(progress, "%d", (int)(acc.cali.step * 1.6f));  
                // KS_DT_SendString("FRONT Done!");
                break;
            case ORIENTATION_BACK:
                //sprintf(progress, "%d", (int)(acc.cali.step * 1.6f)); 
                // KS_DT_SendString("BACK Done!");
                break;
            case ORIENTATION_LEFT:
                //sprintf(progress, "%d", (int)(acc.cali.step * 1.6f)); 
                // KS_DT_SendString("LEFT Done!");
                break;
            case ORIENTATION_RIGHT:
                //sprintf(progress, "%d", (int)(acc.cali.step * 1.6f)); 
               // KS_DT_SendString("RIGHT Done!");
                break;
            default:
                break;
            }
        }
    }

    if(acc.cali.step == 6 && samples_count == 501)
    {
        //计算方程解初值
        float initBeta[6];
        initBeta[0] = 0;
        initBeta[1] = 0;
        initBeta[2] = 0;
        initBeta[3] = 1;
        initBeta[4] = 1;
        initBeta[5] = 1;

        //LM法求解传感器误差方程最优解
        LevenbergMarquardt(samples, &new_offset, &new_scale, initBeta, 1);

        //判断校准参数是否正常
//        if(fabsf(new_scale.x-1.0f) > 0.1f || fabsf(new_scale.y-1.0f) > 0.1f || fabsf(new_scale.z-1.0f) > 0.1f)
//        {
//            acc.cali.success = false;
//        }
////        else if(fabsf(new_offset.x) > (1 * 0.35f) || fabsf(new_offset.y) > (1 * 0.35f) || fabsf(new_offset.z) > (1 * 0.6f))
////        {
//            acc.cali.success = false;
//        }
//        else
//        {
        acc.cali.success = true;
//        }

        for(u8 i=0; i<6; i++)
        {
            samples[i].x = 0;
            samples[i].y = 0;
            samples[i].z = 0;
        }

        if(acc.cali.success)
        {
            save.acc_offset[X] = new_offset.x;
            save.acc_offset[Y] = new_offset.y;
            save.acc_offset[Z] = new_offset.z;
            save.acc_scale[X] = new_scale.x;
            save.acc_scale[Y] = new_scale.y;
            save.acc_scale[Z] = new_scale.z;
        
            sensor.acc_CALIBRATE = 0;
					//KS_DT_SendString("ACC init OK!");
					//LED_STA.calAcc = 0;
					//data_save();
            //mavlink发送校准结果
            //KS_DT_SendString("Success");
        }
        else
        {
            //mavlink发送校准结果
            //KS_DT_SendString("Failed");
        }

        //发送校准结果
       // MessageSensorCaliFeedbackEnable(ACC, acc.cali.step, acc.cali.success);
        //sensor.acc_CALIBRATE = 0;
        acc.cali.step = 0;
        for(uint8_t i=0; i<6; i++)
            orientationCaliFlag[i] = 0;
    }
}

/**********************************************************************************************************
*函 数 名: ICM20689_ReadAcc
*功能说明: ICM20689读取加速度传感器，并转化为标准单位
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
u8 mpu_buffer[14];
_center_pos_st center_pos;
_sensor_st sensor;
void Drv_Icm20689_Read()
{
    Spi_GyroMultiRead(MPU_RA_ACCEL_XOUT_H, mpu_buffer, 14);
}

/**********************************************************************************************************
*函 数 名: ICM20689_ReadGyro
*功能说明: ICM20689读取陀螺仪传感器，并转化为标准单位
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
s32 sensor_val[6];
s32 sensor_val_rot[6];
s32 sensor_val_ref[6];
//float sensor_val_lpf[2][6];


s32 sum_temp[7]={0,0,0,0,0,0,0};
s32 acc_auto_sum_temp[3];
s16 acc_z_auto[4];

u16 acc_sum_cnt = 0,gyro_sum_cnt = 0,acc_z_auto_cnt;

s16 g_old[VEC_XYZ];
float g_d_sum[VEC_XYZ] = {500,500,500};

void mpu_auto_az()
{
	if(sensor.acc_z_auto_CALIBRATE)
	{
		acc_z_auto_cnt++;
		
		acc_auto_sum_temp[0] += sensor_val_ref[A_X];
		acc_auto_sum_temp[1] += sensor_val_ref[A_Y];
		acc_auto_sum_temp[2] += sensor_val_rot[A_Z];
		
		if(acc_z_auto_cnt>=OFFSET_AV_NUM)
		{
			sensor.acc_z_auto_CALIBRATE = 0;
			acc_z_auto_cnt = 0;

			for(u8 i = 0;i<3;i++)
			{			
				acc_z_auto[i] = acc_auto_sum_temp[i]/OFFSET_AV_NUM;
				
				acc_auto_sum_temp[i] = 0;
			}
			
			acc_z_auto[3] = my_sqrt( 4096*4096 - (my_pow(acc_z_auto[0]) + my_pow(acc_z_auto[1])) );
			
			save.acc_offset[Z] = acc_z_auto[2] - acc_z_auto[3];
			

		}
		
	}
}

//静止检测
void motionless_check(u8 dT_ms)
{
	u8 t = 0;

	for(u8 i = 0;i<3;i++)
	{
		g_d_sum[i] += abs(sensor.Gyro_Original[i] - g_old[i]) ;
		
		g_d_sum[i] -= dT_ms ;	
		
		g_d_sum[i] = LIMIT(g_d_sum[i],0,500);
		
		if( g_d_sum[i] > 400)
		{
			t++;
		}
		
		g_old[i] = sensor.Gyro_Original[i];
	}

	if(t>=2)
	{
		flag.motionless = 0;	
	}
	else
	{
		flag.motionless = 1;
	}

}

void IMU_Data_Offset()
{
	static u8 off_cnt;

	
	if(sensor.gyr_CALIBRATE || sensor.acc_CALIBRATE || sensor.acc_z_auto_CALIBRATE)
	{	



///////////////////复位校准值///////////////////////////		
		if(flag.motionless == 0 || sensor_val[A_Z]<1000)
		{
				gyro_sum_cnt = 0;
				acc_sum_cnt=0;
				acc_z_auto_cnt = 0;
				
				for(u8 j=0;j<3;j++)
				{
					acc_auto_sum_temp[j] = sum_temp[G_X+j] = sum_temp[A_X+j] = 0;
				}
				sum_temp[TEM] = 0;
		}

		

		
///////////////////////////////////////////////////////////
		off_cnt++;			
		if(off_cnt>=10)
		{	
			off_cnt=0;

			
			
			if(sensor.gyr_CALIBRATE)
			{
				//LED_STA.calGyr = 1;
				gyro_sum_cnt++;
				
				for(u8 i = 0;i<3;i++)
				{
					sum_temp[G_X+i] += sensor.Gyro_Original[i];
				}
				if( gyro_sum_cnt >= OFFSET_AV_NUM )
				{
					for(u8 i = 0;i<3;i++)
					{
						save.gyro_offset[i] = (float)sum_temp[G_X+i]/OFFSET_AV_NUM;
						
						sum_temp[G_X + i] = 0;
					}
					gyro_sum_cnt =0;
					if(sensor.gyr_CALIBRATE == 1)
					{
						if(sensor.acc_CALIBRATE == 0)
						{
							save_pid_en = 1;
							data_save();
						}
					}
					save_pid_en = 1;
					sensor.gyr_CALIBRATE = 0;
					//KS_DT_SendString("GYR init OK!");
					//LED_STA.calGyr = 0;
				}
			}
			
			if(sensor.acc_CALIBRATE == 1)
			{
				//LED_STA.calAcc = 1;
				acc_sum_cnt++;
				
				sum_temp[A_X] += sensor_val_rot[A_X];
				sum_temp[A_Y] += sensor_val_rot[A_Y];
				sum_temp[A_Z] += sensor_val_rot[A_Z] - 4096;// - 65535/16;   // +-8G
				sum_temp[TEM] += sensor.Tempreature;

				if( acc_sum_cnt >= OFFSET_AV_NUM )
				{
					for(u8 i=0 ;i<3;i++)
					{
						save.acc_offset[i] = sum_temp[A_X+i]/OFFSET_AV_NUM;
						
						sum_temp[A_X + i] = 0;
					}

					acc_sum_cnt =0;
					sensor.acc_CALIBRATE = 0;
					//KS_DT_SendString("ACC init OK!");
					//LED_STA.calAcc = 0;
					save_pid_en = 1;
					data_save();
				}	
			}
		}
	}
}



	

s16 roll_gz_comp;
float wh_matrix[VEC_XYZ][VEC_XYZ] = 
{
	{1,0,0},
	{0,1,0},
	{0,0,1}

};

void Center_Pos_Set()
{
	center_pos.center_pos_cm[X] = 2.0f;//+0.0f;
	center_pos.center_pos_cm[Y] = 0;//-0.0f;
	center_pos.center_pos_cm[Z] = 0;//+0.0f;
}
Vector3f_t AccRaw;
static float gyr_f1[VEC_XYZ],acc_f1[VEC_XYZ];
#define MPU_WINDOW_NUM 5
#define MPU_STEEPEST_NUM 5

#define MPU_WINDOW_NUM_ACC 20
#define MPU_STEEPEST_NUM_ACC 20

_steepest_st steepest_ax;
_steepest_st steepest_ay;
_steepest_st steepest_az;
_steepest_st steepest_gx;
_steepest_st steepest_gy;
_steepest_st steepest_gz;

s32 steepest_ax_arr[MPU_WINDOW_NUM_ACC ];
s32 steepest_ay_arr[MPU_WINDOW_NUM_ACC ];
s32 steepest_az_arr[MPU_WINDOW_NUM_ACC ];
s32 steepest_gx_arr[MPU_WINDOW_NUM ];
s32 steepest_gy_arr[MPU_WINDOW_NUM ];
s32 steepest_gz_arr[MPU_WINDOW_NUM ];
void Sensor_Data_Prepare(u8 dT_ms)
{	
	float hz = 0 ;
	if(dT_ms != 0) hz = 1000/dT_ms;
//	Drv_Icm20689_Read();
	
//	sensor_rotate_func(dT);
	
	/*静止检测*/
	motionless_check(dT_ms);
			
	IMU_Data_Offset(); //校准函数

	/*读取buffer原始数据*/
	sensor.Acc_Original[X] = -(s16)((((u16)mpu_buffer[0]) << 8) | mpu_buffer[1]);//>>1;// + 2 *sensor.Tempreature_C;// + 5 *sensor.Tempreature_C;
	sensor.Acc_Original[Y] = -(s16)((((u16)mpu_buffer[2]) << 8) | mpu_buffer[3]);//>>1;// + 2 *sensor.Tempreature_C;// + 5 *sensor.Tempreature_C;
	sensor.Acc_Original[Z] = -(s16)((((u16)mpu_buffer[4]) << 8) | mpu_buffer[5]);//>>1;// + 4 *sensor.Tempreature_C;// + 7 *sensor.Tempreature_C;
	AccRaw.x = sensor.Acc_Original[X];
	AccRaw.y = sensor.Acc_Original[Y];
	AccRaw.z = sensor.Acc_Original[Z];
	sensor.Gyro_Original[X] = (s16)((((u16)mpu_buffer[ 8]) << 8) | mpu_buffer[ 9]) ;
	sensor.Gyro_Original[Y] = (s16)((((u16)mpu_buffer[10]) << 8) | mpu_buffer[11]) ;
	sensor.Gyro_Original[Z] = (s16)((((u16)mpu_buffer[12]) << 8) | mpu_buffer[13]) ;

	sensor.Tempreature = ((((int16_t)mpu_buffer[6]) << 8) | mpu_buffer[7]); //tempreature
	/*icm20602温度*/
	sensor.Tempreature_C = sensor.Tempreature/326.8f + 25 ;//sensor.Tempreature/340.0f + 36.5f;
	

	/*得出校准后的数据*/
	for(u8 i=0;i<3;i++)
	{ 
		
		sensor_val[A_X+i] = sensor.Acc_Original[i] ;

		sensor_val[G_X+i] = sensor.Gyro_Original[i] - save.gyro_offset[i];
		//sensor_val[G_X+i] = (sensor_val[G_X+i] >>2) <<2;
	}
	
	/*可将整个传感器坐标进行旋转*/
//	for(u8 j=0;j<3;j++)
//	{
//		float t = 0;
//		
//		for(u8 i=0;i<3;i++)
//		{
//			
//			t += sensor_val[A_X + i] *wh_matrix[j][i]; 
//		}
//		
//		sensor_val_rot[A_X + j] = t;
//	}

//	for(u8 j=0;j<3;j++)
//	{
//		float t = 0;
//		
//		for(u8 i=0;i<3;i++)
//		{
//			
//			t += sensor_val[G_X + i] *wh_matrix[j][i]; 
//		}
//		
//		sensor_val_rot[G_X + j] = t;
//	}	

	/*赋值*/
	for(u8 i = 0;i<6;i++)
	{
		sensor_val_rot[i] = sensor_val[i];
	}

	/*数据坐标转90度*/
	sensor_val_ref[G_X] =  sensor_val_rot[G_Y] ;
	sensor_val_ref[G_Y] = -sensor_val_rot[G_X] ;
	sensor_val_ref[G_Z] =  sensor_val_rot[G_Z];

	
	sensor_val_ref[A_X] =  (sensor_val_rot[A_Y] - save.acc_offset[Y] ) ;
	sensor_val_ref[A_Y] = -(sensor_val_rot[A_X] - save.acc_offset[X] ) ;
	sensor_val_ref[A_Z] =  (sensor_val_rot[A_Z] - save.acc_offset[Z] ) ;
	
	/*单独校准z轴模长*/
	mpu_auto_az();

//======================================================================
	
	/*软件滤波*/
	for(u8 i=0;i<3;i++)
	{	
		//0.24f，1ms ，50hz截止; 0.15f,1ms,28hz; 0.1f,1ms,18hz
		gyr_f1[X +i] += 0.15f *(sensor_val_ref[G_X + i] - sensor.Gyro[X +i]);
		acc_f1[X +i] += 0.05f *(sensor_val_ref[A_X + i] - sensor.Acc[X +i]);
//		LPF_1_(100,dT_ms*1e-3f,sensor_val_ref[G_X + i],sensor.Gyro[X +i]);
//		LPF_1_(100,dT_ms*1e-3f,sensor_val_ref[A_X + i],sensor.Acc[X +i]);
				
	}
//	steepest_descend(steepest_ax_arr ,MPU_WINDOW_NUM_ACC ,&steepest_ax ,MPU_STEEPEST_NUM_ACC,(s32) sensor_val_ref[A_X]);
//	steepest_descend(steepest_ay_arr ,MPU_WINDOW_NUM_ACC ,&steepest_ay ,MPU_STEEPEST_NUM_ACC,(s32) sensor_val_ref[A_Y]);
//	steepest_descend(steepest_az_arr ,MPU_WINDOW_NUM_ACC ,&steepest_az ,MPU_STEEPEST_NUM_ACC,(s32) sensor_val_ref[A_Z]);
//	steepest_descend(steepest_gx_arr ,MPU_WINDOW_NUM ,&steepest_gx ,MPU_STEEPEST_NUM,(s32) sensor_val_ref[G_X]);
//	steepest_descend(steepest_gy_arr ,MPU_WINDOW_NUM ,&steepest_gy ,MPU_STEEPEST_NUM,(s32) sensor_val_ref[G_Y]);
//	steepest_descend(steepest_gz_arr ,MPU_WINDOW_NUM ,&steepest_gz ,MPU_STEEPEST_NUM,(s32) sensor_val_ref[G_Z]);

//	gyr_f1[X] = (float)steepest_gx.now_out;
//	gyr_f1[Y] = (float)steepest_gy.now_out;
//	gyr_f1[Z] = (float)steepest_gz.now_out;
//	acc_f1[X] = (float)steepest_ax.now_out;
//	acc_f1[Y] = (float)steepest_ay.now_out;
//	acc_f1[Z] = (float)steepest_az.now_out;
	
			/*旋转加速度补偿*/
//======================================================================
	
	for(u8 i=0;i<3;i++)
	{	
		center_pos.gyro_rad_old[i] = center_pos.gyro_rad[i];
		center_pos.gyro_rad[i] =  gyr_f1[X + i] *RANGE_PN2000_TO_RAD;//0.001065f;
		center_pos.gyro_rad_acc[i] = hz *(center_pos.gyro_rad[i] - center_pos.gyro_rad_old[i]);
	}
	
	center_pos.linear_acc[X] = +center_pos.gyro_rad_acc[Z] *center_pos.center_pos_cm[Y] - center_pos.gyro_rad_acc[Y] *center_pos.center_pos_cm[Z];
	center_pos.linear_acc[Y] = -center_pos.gyro_rad_acc[Z] *center_pos.center_pos_cm[X] + center_pos.gyro_rad_acc[X] *center_pos.center_pos_cm[Z];
	center_pos.linear_acc[Z] = +center_pos.gyro_rad_acc[Y] *center_pos.center_pos_cm[X] - center_pos.gyro_rad_acc[X] *center_pos.center_pos_cm[Y];
	
//======================================================================
	/*赋值*/
	for(u8 i=0;i<3;i++)
	{

		
		sensor.Gyro[X+i] = gyr_f1[i];//sensor_val_ref[G_X + i];
		
		sensor.Acc[X+i] = acc_f1[i] - center_pos.linear_acc[i]/RANGE_PN8G_TO_CMSS;//sensor_val_ref[A_X+i];//
	}
	
	/*转换单位*/
		for(u8 i =0 ;i<3;i++)
		{
			/*陀螺仪转换到度每秒，量程+-2000度*/
			sensor.Gyro_deg[i] = sensor.Gyro[i] *0.06103f ;//  /65535 * 4000; +-2000度 0.061
			/*陀螺仪再低通一次*/
			sensor.Gyro_deg_lpf[i] += 0.12f *(sensor.Gyro_deg[i] - sensor.Gyro_deg_lpf[i]);
			/*陀螺仪转换到弧度度每秒，量程+-2000度*/
			sensor.Gyro_rad[i] = sensor.Gyro[i] *RANGE_PN2000_TO_RAD ;//  0.001065264436f //微调值 0.0010652f

			
			/*加速度计转换到厘米每平方秒，量程+-8G*/
			sensor.Acc_cmss[i] = (sensor.Acc[i] *RANGE_PN8G_TO_CMSS );//   /65535 * 16*981; +-8G
		
		}
		sensor.my_acc_z = -imu_data.Sin_Roll* sensor.Acc_cmss[X]
                              + imu_data.Sin_Pitch *imu_data.Cos_Roll * sensor.Acc_cmss[Y]
                                 + imu_data.Cos_Pitch * imu_data.Cos_Roll *sensor.Acc_cmss[Z];
		sensor.my_acc_z *=9.80f/4096.0f;
		sensor.my_acc_z -=2.30f;
		sensor.my_acc_z *= 100;
}


