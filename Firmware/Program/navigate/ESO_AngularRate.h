#pragma once

//角速度带模型ESO

#include <stdbool.h>

//观测量延时
#define ESO_AngularRate_his_length 4

typedef struct
{
	float beta1;
	float beta2;
	
	float T;
	float invT;
	float z_inertia;
	float z1;
	float z2;
	float u;
	float his_z1[ ESO_AngularRate_his_length ];

	float h;

	bool err_sign;
	float err_continues_time;
	
	float b;
}ESO_AngularRate;

static inline void init_ESO_AngularRate( ESO_AngularRate* eso , float T , float b , float beta1 , float beta2 )
{
	eso->beta1 = beta1;
	eso->beta2 = beta2;
	
	eso->z1 = eso->z2 = eso->z_inertia = 0;
	eso->err_continues_time = 0;	
	
	eso->T = T;	eso->invT = 1.0f / T;
	eso->b = b;
	for( unsigned char i = 0 ; i < ESO_AngularRate_his_length ; ++i )
		eso->his_z1[i] = 0;
}

static inline void ESO_AngularRate_update_u( ESO_AngularRate* eso , float u )
{
	eso->u = u;
	eso->z_inertia += eso->h * eso->invT * ( eso->b*eso->u - eso->z_inertia );
	eso->z1 += eso->h * ( eso->z_inertia + eso->z2 );
}

static inline float ESO_AngularRate_run( ESO_AngularRate* eso , const float v , const float h )
{
	float err = v - eso->his_z1[0];
	
	//更新误差持续时间
	if( (err > 0) ^ eso->err_sign )
	{
		eso->err_continues_time = 0;
		eso->err_sign = err > 0;
	}
	else               
		eso->err_continues_time += h;
	
	float max_beta1_scale = 0.9f / eso->beta1;
	float err_continues_time3 = eso->err_continues_time*eso->err_continues_time*eso->err_continues_time;
	float beta1_scale = 1 + 500 * err_continues_time3;	
	float beta2_scale = 1 + 50 * err_continues_time3;		
	if( beta1_scale > 15 )
		beta1_scale = 15;
	if( beta2_scale > 15 )
		beta2_scale = 15;
	if( beta1_scale > max_beta1_scale )
		beta1_scale = max_beta1_scale;
	
	//修正
	float z1_correction = beta1_scale*eso->beta1*err;
	float z2_correction = beta2_scale*eso->beta2*err;
	float filter_dt = h;
	for( unsigned char k = 0 ; k < ESO_AngularRate_his_length - 1 ; ++k )
	{
		eso->his_z1[ k ] = eso->his_z1[ k + 1 ] + z1_correction + filter_dt*z2_correction;
		filter_dt += h;
	}
	eso->z2 += z2_correction;
	eso->z1 += z1_correction + filter_dt*z2_correction;
	eso->his_z1[ ESO_AngularRate_his_length - 1 ] = eso->z1;
	
	eso->h = h;
	return eso->z2;
}

static inline float ESO_AngularRate_get_EsAngularRate( ESO_AngularRate* eso )
{
	return eso->z1;
}
static inline float ESO_AngularRate_get_EsDisturbance( ESO_AngularRate* eso )
{
	return eso->z2;
}
static inline float ESO_AngularRate_get_EsAngularAcceleration( ESO_AngularRate* eso )
{
	return eso->z2 + eso->z_inertia;
}
static inline float ESO_AngularRate_get_EsMainPower( ESO_AngularRate* eso )
{
	return eso->z_inertia;
}

