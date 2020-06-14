
#ifndef __ALT_CTRL_H
#define __ALT_CTRL_H

#include "include.h"
#include "PID.h"

extern float ct_val_thr;
extern float err_i_comp;
extern float exp_h;
extern PID_arg_t alt_arg_high;
extern PID_val_t alt_val_high;

extern PID_arg_t alt_arg_high_s;
extern PID_val_t alt_val_high_s;

void Alt_1level_Ctrl(float dT_s);
void Alt_1level_PID_Init(void);

void Alt_2level_PID_Init(void);
void Alt_2level_Ctrl(float dT_s);

#endif
