/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NAVIGATE_H
#define __NAVIGATE_H
/* Includes ------------------------------------------------------------------*/
#include "Data.h"
#include "Filter.h"

#include "mathTool.h"
/* Exported types ------------------------------------------------------------*/

typedef struct
{
	float ref_acc_w[VEC_XYZ];
	float ref_speed_w_pre[VEC_XYZ];
	float ref_speed_w_pre_old[VEC_XYZ];
	
  float ref_speed_w[VEC_XYZ];
	float ref_speed_w_old[VEC_XYZ];
	float ref_speed_h[VEC_XYZ];
	
	_inte_fix_filter_st acc_if[VEC_XYZ];
	
	_fix_inte_filter_st speed_fi[VEC_XYZ];
	_inte_fix_filter_st speed_if[VEC_XYZ];
	
	float acc_out_w[VEC_XYZ];
	float speed_out_w_pre[VEC_XYZ];
	float speed_out_w[VEC_XYZ];
	float speed_out_h[VEC_XYZ];
} _w_speed_fus_st;
extern _w_speed_fus_st w_speed_fus;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void wxy_speed_fusing(float dT,u8 est_en,u8 fix_en);
void wxy_fus_task(float dT);

enum
{
    FLOW_VEL_X = 0,
    FLOW_VEL_Y,
    FLOW_VEL_Z,
    BARO_VEL,
    TOF_VEL
};

typedef struct {
    Vector3f_t accel;
    Vector3f_t accel_bias;
    
    Vector3f_t velocity;
    float      velMeasure[6];
    
    Vector3f_t position;
    Vector3f_t posMeasure;
} NAVGATION_t;
extern NAVGATION_t nav;
void NavigationInit(void);
void VelocityEstimate(void);
void PositionEstimate(void);

void AltCovarianceSelfAdaptation(void);
void PosCovarianceSelfAdaptation(void);

Vector3f_t GetCopterAccel(void);
Vector3f_t GetAccelBias(void);
Vector3f_t GetCopterVelocity(void);
float* GetCopterVelMeasure(void);
Vector3f_t GetCopterPosition(void);
Vector3f_t GetCopterPosMeasure(void);

void NavigationReset(void);
#endif

