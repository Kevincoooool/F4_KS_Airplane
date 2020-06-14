
#ifndef _PLAYER_LOCCTRL_H_
#define _PLAYER_LOCCTRL_H_

#include "include.h"
#include "Alt_Ctrl.h"

extern float ct_loc_out[VEC_XYZ];

extern PID_val_t loc_val_2[VEC_XYZ];
extern float exp_[VEC_XYZ],fb_[VEC_XYZ];
extern u8 ct_loc_hold[VEC_XYZ];

void Loc_1level_Ctrl(float dT_s);
void Loc_1level_PID_Init(void);

void Loc_2level_Ctrl(float dT_s);
void Loc_2level_PID_Init(void);
void Fixed_Point_Ctrl(float x,float y);
#endif
