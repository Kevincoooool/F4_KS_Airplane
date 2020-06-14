/*
	TD4·ÇÏßÐÔÂË²¨Æ÷

*/

#pragma once

typedef struct
{
	unsigned char tracking_mode;
	
	float x1;
	float x2;
	float x3;
	float x4;
	
	float P1;
	float P2;
	float P3;
	float P4;
	
	float r2p , r2n , r3p , r3n , r4p , r4n;
}TD4;

static inline void TD4_reset( TD4* filter )
{
	filter->x1 = filter->x2 = filter->x3 = filter->x4 = 0;
}

static inline void TD4_setP( TD4* filter , float P )
{
	filter->P1 = filter->P2 = filter->P3 = filter->P4 = P;
}

static inline void TD4_init( TD4* filter , float P1 , float P2 , float P3 , float P4 )
{
	filter->P1 = P1;
	filter->P2 = P2;
	filter->P3 = P3;
	filter->P4 = P4;
	filter->r2p = filter->r2n = filter->r3p = filter->r3n = filter->r4p = filter->r4n = 1e12;
	TD4_reset( filter );
}

static inline float TD4_track4( TD4* filter , const float expect , const float h )
{
	filter->tracking_mode = 4;
	
	float e1 = expect - filter->x1;
	float e1_1 = -filter->x2;
	float e1_2 = -filter->x3;
	float e1_3 = -filter->x4;
	float T2 = filter->P1 * e1;
	float P1 = 0;
	if( T2 > filter->r2p )
		T2 = filter->r2p;
	else if( T2 < -filter->r2n )
		T2 = -filter->r2n;
	else
		P1 = filter->P1;
	float T2_1 = P1 * e1_1;
	float T2_2 = P1 * e1_2;
	float T2_3 = P1 * e1_3;
	
	float e2 = T2 - filter->x2;
	float e2_1 = T2_1-filter->x3;
	float e2_2 = T2_2-filter->x4;
	float T3 = filter->P2 * e2;
	float P2 = 0;
	if( T3 > filter->r3p )
		T3 = filter->r3p;
	else if( T3 < -filter->r3n )
		T3 = -filter->r3n;
	else
		P2 = filter->P2;
	T3 += T2_1;
	float T3_1 = P2 * e2_1 + T2_2;
	float T3_2 = P2 * e2_2 + T2_3;
	
	float e3 = T3 - filter->x3;
	float e3_1 = T3_1-filter->x4;
	float T4 = filter->P3 * e3;
	float P3 = 0;
	if( T4 > filter->r4p )
		T4 = filter->r4p;
	else if( T4 < -filter->r4n )
		T4 = -filter->r4n;
	else
		P3 = filter->P3;
	T4 += T3_1;
	float T4_1 = P3 * e3_1 + T3_2;
	
	float e4 = T4 - filter->x4;
	float T5 = filter->P4 * e4 + T4_1;
	
	filter->x1 += h*filter->x2;
	filter->x2 += h*filter->x3;
	filter->x3 += h*filter->x4;
	filter->x4 += h*T5;
	
	return filter->x1;
	
	
//	float Tv1 = filter->P1*(-filter->x2);
//	float Tv2 = filter->P1*(-filter->x3);
//	float Tv3 = filter->P1*(-filter->x4);
//	float Ta1 = filter->P2*( Tv1 - filter->x3 ) + Tv2;
//	float Ta2 = filter->P2*( Tv2 - filter->x4 ) + Tv3;
//	float Tj1 = filter->P3*( Ta1 - filter->x4 ) + Ta2;
//	
//	float Tx2 = filter->P1*( expect - filter->x1 );
//	float Tx3 = filter->P2*( Tx2 - filter->x2 ) + Tv1;
//	float Tx4 = filter->P3*( Tx3 - filter->x3 ) + Ta1;
//	float Tx5 = filter->P4*( Tx4 - filter->x4 ) + Tj1;
//	
//	filter->x1 += h*filter->x2;
//	filter->x2 += h*filter->x3;
//	filter->x3 += h*filter->x4;
//	filter->x4 += h*Tx5;
//	
//	return filter->x1;
}

static inline float TD4_track3( TD4* filter , const float expect , const float h )
{
	filter->tracking_mode = 3;
	
	float e2 = expect - filter->x2;
	float e2_1 = -filter->x3;
	float e2_2 = -filter->x4;
	float T3 = filter->P2 * e2;
	float P2 = 0;
	if( T3 > filter->r3p )
		T3 = filter->r3p;
	else if( T3 < -filter->r3n )
		T3 = -filter->r3n;
	else
		P2 = filter->P2;
	float T3_1 = P2 * e2_1;
	float T3_2 = P2 * e2_2;
	
	float e3 = T3 - filter->x3;
	float e3_1 = T3_1-filter->x4;
	float T4 = filter->P3 * e3;
	float P3 = 0;
	if( T4 > filter->r4p )
		T4 = filter->r4p;
	else if( T4 < -filter->r4n )
		T4 = -filter->r4n;
	else
		P3 = filter->P3;
	T4 += T3_1;
	float T4_1 = P3 * e3_1 + T3_2;
	
	float e4 = T4 - filter->x4;
	float T5 = filter->P4 * e4 + T4_1;
	
	filter->x2 += h*filter->x3;
	filter->x3 += h*filter->x4;
	filter->x4 += h*T5;
	
	return filter->x2;
	
	
	
	
//	float Ta1 = filter->P2*( - filter->x3 );
//	float Ta2 = filter->P2*( - filter->x4 );
//	float Tj1 = filter->P3*( Ta1 - filter->x4 ) + Ta2;
//	
//	float Tx3 = filter->P2*( expect - filter->x2 );
//	float Tx4 = filter->P3*( Tx3 - filter->x3 ) + Ta1;
//	float Tx5 = filter->P4*( Tx4 - filter->x4 ) + Tj1;
//	
//	filter->x2 += h*filter->x3;
//	filter->x3 += h*filter->x4;
//	filter->x4 += h*Tx5;
//	
//	return filter->x2;
}


