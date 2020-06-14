#ifndef SPL06_01_H
#define SPL06_01_H

#include <stm32f4xx.h>

#define s32 int32
#define int16 short
#define int32 int
#define uint8 unsigned char
#define HW_ADR 0x77 //0x76 //0x77 01  0x76  00   7bit  0xec
#define CONTINUOUS_PRESSURE     1
#define CONTINUOUS_TEMPERATURE  2
#define CONTINUOUS_P_AND_T      3
#define PRESSURE_SENSOR     0
#define TEMPERATURE_SENSOR  1


struct spl0601_calib_param_t {	
    int16 c0;
    int16 c1;
    int32 c00;
    int32 c10;
    int16 c01;
    int16 c11;
    int16 c20;
    int16 c21;
    int16 c30;       
};

struct spl0601_t {	
    struct spl0601_calib_param_t calib_param;/**<calibration data*/	
    uint8 chip_id; /**<chip id*/	
    int32 i32rawPressure;
    int32 i32rawTemperature;
    int32 i32kP;    
    int32 i32kT;
};

extern float temperature,temperature2;

u8 Drv_Spl0601_Init ( void );
void spl0601_rateset(uint8 iSensor, uint8 u8OverSmpl, uint8 u8SmplRate);
void spl0601_start_temperature(void);
void spl0601_start_pressure(void);
void spl0601_start_continuous(uint8 mode);
void spl0601_get_raw_temp(void);
void spl0601_get_raw_pressure(void);
float spl0601_get_temperature(void);
float spl0601_get_pressure(void);
float user_spl0601_get(void);
float Drv_Spl0601_Read ( void );
#endif

