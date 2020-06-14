
#ifndef __KS_MOTIONCAL_H
#define __KS_MOTIONCAL_H

#include "include.h"
#include "filter.h"

/* Exported types ------------------------------------------------------------*/
extern _fix_inte_filter_st wcz_spe_fus,wcz_hei_fus;
extern s32 wcz_ref_height,wcz_ref_speed;
extern float wcz_acc_deadzone,wcz_acc_use;	
/* Exported constants --------------------------------------------------------*/
void WCZ_Data_Calc(u8 dT_ms);
void WCZ_Data_Reset(void);
void WCZ_Acc_Get_Task(void);
void WCZ_Acc_Get_Task(void);

typedef struct{
	float High;//处理后的高度
	float Speed;//高度
}_st_Height;

extern _st_Height Height;
void Strapdown_INS_High(float high);

#endif

