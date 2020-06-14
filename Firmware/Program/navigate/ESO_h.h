#pragma once

//高度无参ESO
//朱文杰 20180102
//请勿用于商业用途
//！！抄袭必究！！

#include <stdbool.h>
#include "RingBuf.h"
#include "AC_Math.h"

//观测量延时
#define ESO_h_his_length 8

typedef struct
{
	float beta;
	
	float T;
	float invT;
	float z_inertia;
	float z1;
	float z2;
	float u;
	
	float buf[8];
	RingBuf_float his_zin;

	float h;

	bool err_sign;
	float err_continues_time;
}ESO_h;

static inline void ESO_h_init( ESO_h* eso , const float T , const float beta )
{
	RingBuf_float_init( &eso->his_zin , eso->buf , ESO_h_his_length );
	eso->T = T;
	eso->invT = 1.0f / T;
	eso->beta = beta;
	eso->z1 = eso->u = 0;
	eso->z2 =1.0f;
	for( unsigned char i = 0 ; i < ESO_h_his_length ; ++i )
		eso->buf[i] = 0;
}

static inline void ESO_h_update_u( ESO_h* eso , float u )
{
	eso->u = u;
	eso->z_inertia += eso->h * eso->invT * ( eso->u - eso->z_inertia );
}

static inline float ESO_h_run( ESO_h* eso , float acc , float h )
{
	if( eso->z2 < 1.0f )
		eso->z2 = 1.0f;
	float b = constG / eso->z2;
	float his_zinertial = RingBuf_float_GetHis( &eso->his_zin , ESO_h_his_length - 1 );
	if( his_zinertial > 0.1f )
	{
		float actual_z2 = his_zinertial / ( acc + constG ) * constG;
		float err = actual_z2 - eso->z2;
		if( (err > 0) ^ eso->err_sign )
		{
			eso->err_continues_time = 0;
			eso->err_sign = err > 0;
		}
		else
			eso->err_continues_time += h;
		
		if( err > 10 )
			err = 10;
		else if( err < -10 )
			err = -10;
		float beta_scale = eso->beta * ( 1 + 300*eso->err_continues_time*eso->err_continues_time*eso->err_continues_time );
		if( beta_scale > 0.2f )
			beta_scale = 0.2f;
		eso->z2 += beta_scale * err;
	}
	if( eso->z2 < 1.0f )
		eso->z2 = 1.0f;
	RingBuf_float_push( &eso->his_zin , eso->z_inertia );
	
	eso->h = h;
	return eso->z2;
}