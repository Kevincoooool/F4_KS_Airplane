#include "Navigate.h"
#include "Math.h"
#include "Imu.h"
#include "kalman3.h"
#include "kalmanVel.h"
#include "include.h"
#include "opticalflow.h"
#include "malloc.h"	
#include "MotionCal.h"
NAVGATION_t nav;
KalmanVel_t kalmanVel;
Kalman_t kalmanPos;

static void KalmanVelInit(void);
static void KalmanPosInit(void);

/**********************************************************************************************************
*º¯ Êı Ãû: NavigationInit
*¹¦ÄÜËµÃ÷: µ¼º½²ÎÊı³õÊ¼»¯
*ĞÎ    ²Î: ÎŞ
*·µ »Ø Öµ: ÎŞ
**************************************************************************************************/
void NavigationInit(void)
{
    KalmanVelInit();
    KalmanPosInit();
}

/**********************************************************************************************************
*º¯ Êı Ãû: VelocityEstimate
*¹¦ÄÜËµÃ÷: ·ÉĞĞËÙ¶È¹À¼Æ ÈÚºÏ¼ÓËÙ¶È¡¢GPS¡¢ÆøÑ¹¼Æ¼°TOFµÈ¶à¸ö´«¸ĞÆ÷µÄÊı¾İ
*          ËÙ¶ÈµÄ¼ÆËã¾ùÔÚ»úÌå×ø±êÏµÏÂ½øĞĞ£¬ËùÒÔGPSËÙ¶ÈÔÚ²ÎÓëÈÚºÏÊ±ĞèÒªÏÈ×ª»»µ½»úÌå×ø±êÏµ
*ĞÎ    ²Î: ÎŞ
*·µ »Ø Öµ: ÎŞ
**************************************************************************************************/
void VelocityEstimate(void)
{
    static uint64_t previousT;
    float deltaT;
    static uint32_t count;
    static bool fuseFlag;
        
    ////¼ÆËãÊ±¼ä¼ä¸ô£¬ÓÃÓÚ»ı·Ö
    deltaT = (GetSysTime_us() - previousT) * 1e-6f;
    deltaT = ConstrainFloat(deltaT, 0.0005, 0.005);
    previousT = GetSysTime_us();

    //»ñÈ¡ÔË¶¯¼ÓËÙ¶È
    nav.accel.x = -imu_data.h_acc[Y]*1.5f;
    nav.accel.y = -imu_data.h_acc[X]*1.5f;
    nav.accel.z = imu_data.w_acc[Z]*1e-3f;
                                                                                           
    //¼ÓËÙ¶ÈÊı¾İ¸üĞÂÆµÂÊ1KHz£¬¶øÆøÑ¹Êı¾İ¸üĞÂÆµÂÊÖ»ÓĞ25Hz£¬GPSÊı¾İÖ»ÓĞ10Hz
    //ÕâÀï½«ÆøÑ¹ÓëGPS²ÎÓëÈÚºÏµÄÆµÂÊÇ¿ÖÆÍ³Ò»Îª25Hz
    if(count++ % 40 == 0)
    {
        nav.velMeasure[0] = user_flow_x*10;           //¹âÁ÷XÖáËÙ¶È
        nav.velMeasure[1] = user_flow_y*10;           //¹âÁ÷YÖáËÙ¶È
        nav.velMeasure[2] = wcz_ref_speed;                  //ZÖáÆøÑ¹¼ÆËÙ¶È
        nav.velMeasure[3] = 0;              //ZÖáÆøÑ¹¼ÆËÙ¶È
        nav.velMeasure[4] = 0;                              //TOF¼¤¹âËÙ¶È
        nav.velMeasure[5] = 0; 
        
        KalmanVelUseMeasurement(&kalmanVel, TOF_VEL, false);
        
        fuseFlag = true;
    }
    else
    {
        fuseFlag = false;
    }
    

    KalmanVelUpdate(&kalmanVel, &nav.velocity, &nav.accel_bias, nav.accel, nav.velMeasure, deltaT, fuseFlag);

}


void PositionEstimate(void)
{
    static uint64_t previousT;
    float deltaT;
    Vector3f_t input;
    static uint32_t count;
    static bool fuseFlag;


    deltaT = (GetSysTime_us() - previousT) * 1e-6;
    deltaT = ConstrainFloat(deltaT, 0.0005, 0.002);
    previousT = GetSysTime_us();


    if(count++ % 40 == 0)
    {	
		nav.posMeasure.x = mini_flow.x_i;
		nav.posMeasure.y = mini_flow.y_i;
		nav.posMeasure.z = wcz_ref_height;
        fuseFlag = true;
    }
    else
    {
        fuseFlag = false;
    }
    //ËÙ¶È»ı·Ö
    input.x = nav.velocity.x * deltaT;
    input.y = nav.velocity.y * deltaT;
    input.z = nav.velocity.z * deltaT;

    //Î»ÖÃ¸üĞÂ
    KalmanUpdate(&kalmanPos, input, nav.posMeasure, fuseFlag);
    nav.position = kalmanPos.state;
}


/**********************************************************************************************************
*º¯ Êı Ãû: KalmanVelInit
*¹¦ÄÜËµÃ÷: ·ÉĞĞËÙ¶È¹À¼ÆµÄ¿¨¶ûÂü½á¹¹Ìå³õÊ¼»¯
*ĞÎ    ²Î: R¹Ì¶¨£¬QÔ½´ó£¬´ú±íÔ½ĞÅÈÎ²àÁ¿Öµ£¬QÎŞÇî´ú±íÖ»ÓÃ²âÁ¿Öµ£»·´Ö®£¬QÔ½Ğ¡´ú±íÔ½ĞÅÈÎÄ£ĞÍÔ¤²âÖµ£¬QÎªÁãÔòÊÇÖ»ÓÃÄ£ĞÍÔ¤²â¡£
*·µ »Ø Öµ: ÎŞ q 0.2 r 5 »¹À«ÒÔ
**************************************************************************************************************************************************/
static void KalmanVelInit(void)
{
    float qMatInit[6][6] = {{0.1, 0, 0, 0, 0, 0},
                            {0, 0.1, 0, 0, 0, 0},
                            {0, 0, 0.05, 0, 0, 0},      
                            {0.03, 0, 0, 0, 0, 0},
                            {0, 0.03, 0, 0, 0, 0},
                            {0, 0, 0.03, 0, 0, 0}};

    float rMatInit[6][6] = {{200, 0, 0, 0, 0, 0},          //ËÙ¶ÈxÖáÊı¾İÔëÉù·½²î
                            {0, 200, 0, 0, 0, 0},          //ËÙ¶ÈyÖáÊı¾İÔëÉù·½²î
                            {0, 0, 200, 0, 0, 0},          //GPSËÙ¶ÈzÖáÊı¾İÔëÉù·½²î       
                            {0, 0, 0, 2500, 0, 0},         //ÆøÑ¹ËÙ¶ÈÊı¾İÔëÉù·½²î
                            {0, 0, 0, 0, 2000, 0},         //TOFËÙ¶ÈÊı¾İÔëÉù·½²î
                            {0, 0, 0, 0, 0, 500000}};      //zÖáËÙ¶È¸ßÍ¨ÂË²¨ÏµÊı

    float pMatInit[6][6] = {{20, 0, 0, 0, 0, 0},
                            {0, 20, 0, 0, 0, 0},
                            {0, 0, 5, 0, 0, 0},      
                            {2, 0, 0, 2, 0, 0},
                            {0, 2, 0, 0, 2, 0},
                            {0, 0, 6, 0, 0, 2}};    //Ôö´óĞ­·½²îPµÄ³õÖµ£¬¿ÉÒÔÌá¸ß³õÊ¼»¯Ê±biasµÄÊÕÁ²ËÙ¶È

    float hMatInit[6][6] = {{1, 0, 0, 0, 0, 0},
                            {0, 1, 0, 0, 0, 0},
                            {0, 0, 1, 0, 0, 0},      
                            {0, 0, 1, 0, 0, 0},
                            {0, 0, 1, 0, 0, 0},
                            {0, 0, 1, 0, 0, 0}};    //h[5][2]:ËÙ¶ÈzÖáÔö¼ÓÉÙĞí¸ßÍ¨ÂË²¨Ğ§¹û½

    float fMatInit[6][6] = {{1, 0, 0, 0, 0, 0},
                            {0, 1, 0, 0, 0, 0},
                            {0, 0, 1, 0, 0, 0},      
                            {0, 0, 0, 1, 0, 0},
                            {0, 0, 0, 0, 1, 0},
                            {0, 0, 0, 0, 0, 1}};
    
    float bMatInit[6][6] = {{1, 0, 0, 0, 0, 0},
                            {0, 1, 0, 0, 0, 0},
                            {0, 0, 1, 0, 0, 0},      
                            {0, 0, 0, 0, 0, 0},
                            {0, 0, 0, 0, 0, 0},
                            {0, 0, 0, 0, 0, 0}};
    
	//³õÊ¼»¯¿¨¶ûÂüÂË²¨Æ÷µÄÏà¹Ø¾ØÕó
    KalmanVelQMatSet(&kalmanVel, qMatInit);
    KalmanVelRMatSet(&kalmanVel, rMatInit);
    KalmanVelCovarianceMatSet(&kalmanVel, pMatInit);
    KalmanVelObserveMapMatSet(&kalmanVel, hMatInit);
    KalmanVelStateTransMatSet(&kalmanVel, fMatInit);
    KalmanVelBMatSet(&kalmanVel, bMatInit);
                            
    ////×´Ì¬»¬¶¯´°¿Ú£¬ÓÃÓÚ½â¾ö¿¨¶ûÂü×´Ì¬¹À¼ÆÁ¿Óë¹Û²âÁ¿Ö®¼äµÄÏàÎ»²îÎÊÌâ
	kalmanVel.slidWindowSize = 250;
    kalmanVel.stateSlidWindow = mymalloc(kalmanVel.slidWindowSize * sizeof(Vector3f_t));
    kalmanVel.fuseDelay[FLOW_VEL_X] = 50;    //ËÙ¶ÈxÖáÊı¾İÑÓ³Ù²ÎÊı£º0.22s
    kalmanVel.fuseDelay[FLOW_VEL_Y] = 50;    //ËÙ¶ÈyÖáÊı¾İÑÓ³Ù²ÎÊı£º0.22s
    kalmanVel.fuseDelay[FLOW_VEL_Z] = 50;    //ËÙ¶ÈzÖáÊı¾İÑÓ³Ù²ÎÊı£º0.22s
    kalmanVel.fuseDelay[BARO_VEL]  = 50;     //ÆøÑ¹ËÙ¶ÈÊı¾İÑÓ³Ù²ÎÊı£º0.1s
    kalmanVel.fuseDelay[TOF_VEL]   = 30;     //TOFËÙ¶ÈÊı¾İÑÓ³Ù²ÎÊı£º
}

/**********************************************************************************************************
*º¯ Êı Ãû: KalmanPosInit
*¹¦ÄÜËµÃ÷: Î»ÖÃ¹À¼ÆµÄ¿¨¶ûÂü½á¹¹Ìå³õÊ¼»¯
*ĞÎ    ²Î: ÎŞ
*·µ »Ø Öµ: ÎŞ
*****************************************************************************************************************/
static void KalmanPosInit(void)
{
    float qMatInit[9] = {0.5, 0, 0, 0, 0.5, 0, 0, 0, 0.5};
    float rMatInit[9] = {20, 0,  0, 0,20, 0, 0, 0, 50};
    float pMatInit[9] = {10, 0, 0, 0, 10, 0, 0, 0, 10};
    float fMatInit[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    float hMatInit[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    float bMatInit[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    //³õÊ¼»¯¿¨¶ûÂüÂË²¨Æ÷µÄÏà¹Ø¾ØÕó  
	KalmanQMatSet(&kalmanPos, qMatInit);
    KalmanRMatSet(&kalmanPos, rMatInit);
    KalmanBMatSet(&kalmanPos, bMatInit);
    KalmanCovarianceMatSet(&kalmanPos, pMatInit);
    KalmanStateTransMatSet(&kalmanPos, fMatInit);
    KalmanObserveMapMatSet(&kalmanPos, hMatInit);

    //×´Ì¬»¬¶¯´°¿Ú£¬ÓÃÓÚ½â¾ö¿¨¶ûÂü×´Ì¬¹À¼ÆÁ¿Óë¹Û²âÁ¿Ö®¼äµÄÏàÎ»²îÎÊÌâ
    kalmanPos.slidWindowSize = 200;
    kalmanPos.statusSlidWindow = mymalloc(kalmanPos.slidWindowSize * sizeof(kalmanPos.state));
    kalmanPos.fuseDelay.x = 20;    //0.05sÑÓÊ±
    kalmanPos.fuseDelay.y = 20;    //0.05sÑÓÊ±
    kalmanPos.fuseDelay.z = 100;    //0.05sÑÓÊ±
}

/**********************************************************************************************************
*º¯ Êı Ãû: NavigationReset
*¹¦ÄÜËµÃ÷: µ¼º½Ïà¹ØÊı¾İ¸´Î»
*ĞÎ    ²Î: ÎŞ
*·µ »Ø Öµ: ÎŞ
***************************************************************************************************/
void NavigationReset(void)
{
    kalmanVel.state[0] = 0;
    kalmanVel.state[1] = 0;
    kalmanVel.state[2] = 0;

//    if(GpsGetFixStatus())
//    {
    kalmanPos.state.x = mini_flow.x_i;
    kalmanPos.state.y = mini_flow.y_i;
//    }
//    else
//    {
//        kalmanPos.state.x = 0;
//        kalmanPos.state.y = 0;
//    }
    kalmanPos.state.z = baro_height;
}

/**********************************************************************************************************
*º¯ Êı Ãû: GetCopterAccel
*¹¦ÄÜËµÃ÷: »ñÈ¡·ÉĞĞ¼ÓËÙ¶È
*ĞÎ    ²Î: ÎŞ
*·µ »Ø Öµ: ¼ÓËÙ¶ÈÖµ
************************************************************************************/
Vector3f_t GetCopterAccel(void)
{
    return nav.accel;
}

/**********************************************************************************************************
*º¯ Êı Ãû: GetAccelBias
*¹¦ÄÜËµÃ÷: »ñÈ¡¼ÓËÙ¶Èbias
*ĞÎ    ²Î: ÎŞ
*·µ »Ø Öµ: ¼ÓËÙ¶ÈbiasÖµ
**********************************************************************************************************/
Vector3f_t GetAccelBias(void)
{
    return nav.accel_bias;
}

/**********************************************************************************************************
*º¯ Êı Ãû: GetCopterVelocity
*¹¦ÄÜËµÃ÷: »ñÈ¡·ÉĞĞËÙ¶È¹À¼ÆÖµ
*ĞÎ    ²Î: ÎŞ
*·µ »Ø Öµ: ËÙ¶ÈÖµ
**********************************************************************************************************/
Vector3f_t GetCopterVelocity(void)
{
    return nav.velocity;
}

/**********************************************************************************************************
*º¯ Êı Ãû: GetCopterVelMeasure
*¹¦ÄÜËµÃ÷: »ñÈ¡·ÉĞĞËÙ¶È²âÁ¿Öµ
*ĞÎ    ²Î: ÎŞ
*·µ »Ø Öµ: ËÙ¶ÈÖµ
**********************************************************************************************************/
float* GetCopterVelMeasure(void)
{
    return nav.velMeasure;
}

/**********************************************************************************************************
*º¯ Êı Ãû: GetCopterPosition
*¹¦ÄÜËµÃ÷: »ñÈ¡Î»ÖÃ¹À¼ÆÖµ
*ĞÎ    ²Î: ÎŞ
*·µ »Ø Öµ: Î»ÖÃÖµ
**********************************************************************************************************/
Vector3f_t GetCopterPosition(void)
{
    return nav.position;
}

/**********************************************************************************************************
*º¯ Êı Ãû: GetCopterPosMeasure
*¹¦ÄÜËµÃ÷: »ñÈ¡Î»ÖÃ²âÁ¿Öµ
*ĞÎ    ²Î: ÎŞ
*·µ »Ø Öµ: ËÙ¶ÈÖµ
**********************************************************************************************************/
Vector3f_t GetCopterPosMeasure(void)
{
    return nav.posMeasure;
}
