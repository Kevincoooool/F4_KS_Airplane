
#include "drv_ist8310.h"
#include "drv_i2c_soft.h"
#include "include.h"

/* IST8310的IIC地址选择
  CAD1  |  CAD0  |  地址 | 模拟IIC地址
------------------------------
  GND   |   GND  |  0CH  | 18H
  GND   |   VDD  |  0DH  | 1AH
  VDD   |   GND  |  0EH  | 1CH
  VDD   |   VDD  |  0FH  | 1EH
  如果CAD1和CAD0都悬空, 地址为0EH
 */
#define IST8310_ADDRESS                 0x0E<<1

#define IST8310_REG_HX_L                0x03
#define IST8310_REG_HX_H                0x04
#define IST8310_REG_HY_L                0x05
#define IST8310_REG_HY_H                0x06
#define IST8310_REG_HZ_L                0x07
#define IST8310_REG_HZ_H                0x08
#define IST8310_REG_WHOAMI              0x00
#define IST8310_REG_CNTRL1              0x0A
#define IST8310_REG_CNTRL2              0x0B
#define IST8310_REG_AVERAGE             0x41
#define IST8310_REG_PDCNTL              0x42
#define IST8310_ODR_SINGLE              0x01
#define IST8310_ODR_10_HZ               0x03
#define IST8310_ODR_20_HZ               0x05
#define IST8310_ODR_50_HZ               0x07
#define IST8310_ODR_100_HZ              0x06
#define IST8310_ODR_200_HZ              0x0B
#define IST8310_CHIP_ID                 0x0E
#define IST8310_AVG_16                  0x24
#define IST8310_PULSE_DURATION_NORMAL   0xC0
#define IST8310_CNTRL2_RESET            0x01
#define IST8310_CNTRL2_DRPOL            0x04
#define IST8310_CNTRL2_DRENA            0x08

#define IST8310_MAG_TO_GAUSS            0.0015f


/**********************************************************************************************************
*函 数 名: IST8310_WriteReg
*功能说明: 往IST8310的寄存器写入一个字节的数据
*形    参: 寄存器地址 写入数据
*返 回 值: 无
**********************************************************************************************************/
void IST8310_WriteReg(u8 REG_Address,u8 REG_data)
{
    IIC_Write_1Byte(IST8310_ADDRESS, REG_Address, REG_data);
}

/**********************************************************************************************************
*函 数 名: IST8310_ReadReg
*功能说明: 读取IST8310寄存器的数据
*形    参: 寄存器地址
*返 回 值: 寄存器数据
**********************************************************************************************************/
uint8_t IST8310_ReadReg(u8 REG_Address)
{
	uint8_t REG1_data;
	
    IIC_Read_1Byte(IST8310_ADDRESS, REG_Address, &REG1_data);
	return REG1_data;
}

/**********************************************************************************************************
*函 数 名: IST8310_Detect
*功能说明: 检测IST8310是否存在
*形    参: 无
*返 回 值: 存在状态
**********************************************************************************************************/
u8 chip_id;
u8 IST8310_Detect(void)
{
	chip_id = IST8310_ReadReg(0x40);
    if(IST8310_ReadReg(IST8310_REG_WHOAMI) == IST8310_CHIP_ID)
        return 1;
    else
        return 0;
}

/**********************************************************************************************************
*函 数 名: IST8310_Init
*功能说明: IST8310寄存器配置初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void IST8310_Init(void)
{
    
	IST8310_WriteReg(IST8310_REG_CNTRL1, IST8310_ODR_200_HZ);
	IST8310_WriteReg(IST8310_REG_CNTRL2, 0x24);
    Delay_ms(5);
}

/**********************************************************************************************************
*函 数 名: IST8310_Update
*功能说明: IST8310数据更新
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static u8 IST8310_buf[6];	
void Mag_Get(s16 mag_va[3])
{
    s16 t[3];

	t[0] = ((((int16_t)IST8310_buf[1]) << 8) | IST8310_buf[0]) ;
	t[1] = ((((int16_t)IST8310_buf[3]) << 8) | IST8310_buf[2]) ;
	t[2] = ((((int16_t)IST8310_buf[5]) << 8) | IST8310_buf[4]) ;

	/*转换坐标轴为ANO坐标*/
	mag_va[0] = -t[1];
	mag_va[1] = +t[0];
	mag_va[2] =  t[2];
}

/**********************************************************************************************************
*函 数 名: IST8310_Read
*功能说明: 读取地磁传感器数据,并转换为标准单位
*形    参: 读出数据指针
*返 回 值: 无
**********************************************************************************************************/
void Drv_IST8310_Read(void)
{	
	
	IIC_Read_nByte(IST8310_ADDRESS,IST8310_REG_HX_L,6,IST8310_buf);

}

