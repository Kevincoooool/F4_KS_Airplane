
#include "Param.h"
#include "Stick.h"
#include "Drv_MPU6050.h"

#define FIRSTINITFLAH  			0x44

struct param Param;

void Param_Init(void)//恢复默认参数
{
	Param.NRF_Channel= 0;
	Param.OffSet_Rol = 0;
	Param.OffSet_Pit = 0;
	Param.OffSet_Yaw = 0;
	Param.OffSet_Thr = 0;
	Param.FirstInitFlag = FIRSTINITFLAH;
	
	Flash_Write((u8 *)(&Param),sizeof(Param));
}

void Param_SAVE(void)
{
	Flash_Write((u8 *)(&Param),sizeof(Param));
}

void Param_READ(void)
{
	Flash_Read((u8 *)(&Param),sizeof(Param));
	if(Param.FirstInitFlag != FIRSTINITFLAH)//板子从未初始化
	{
		offset = 1;
		Param_Init();
	}
}










