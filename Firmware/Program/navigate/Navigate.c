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
*�� �� ��: NavigationInit
*����˵��: ����������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**************************************************************************************************/
void NavigationInit(void)
{
    KalmanVelInit();
    KalmanPosInit();
}

/**********************************************************************************************************
*�� �� ��: VelocityEstimate
*����˵��: �����ٶȹ��� �ںϼ��ٶȡ�GPS����ѹ�Ƽ�TOF�ȶ��������������
*          �ٶȵļ�����ڻ�������ϵ�½��У�����GPS�ٶ��ڲ����ں�ʱ��Ҫ��ת������������ϵ
*��    ��: ��
*�� �� ֵ: ��
**************************************************************************************************/
void VelocityEstimate(void)
{
    static uint64_t previousT;
    float deltaT;
    static uint32_t count;
    static bool fuseFlag;
        
    ////����ʱ���������ڻ���
    deltaT = (GetSysTime_us() - previousT) * 1e-6f;
    deltaT = ConstrainFloat(deltaT, 0.0005, 0.005);
    previousT = GetSysTime_us();

    //��ȡ�˶����ٶ�
    nav.accel.x = -imu_data.h_acc[Y]*1.5f;
    nav.accel.y = -imu_data.h_acc[X]*1.5f;
    nav.accel.z = imu_data.w_acc[Z]*1e-3f;
                                                                                           
    //���ٶ����ݸ���Ƶ��1KHz������ѹ���ݸ���Ƶ��ֻ��25Hz��GPS����ֻ��10Hz
    //���ｫ��ѹ��GPS�����ںϵ�Ƶ��ǿ��ͳһΪ25Hz
    if(count++ % 40 == 0)
    {
        nav.velMeasure[0] = user_flow_x*10;           //����X���ٶ�
        nav.velMeasure[1] = user_flow_y*10;           //����Y���ٶ�
        nav.velMeasure[2] = wcz_ref_speed;                  //Z����ѹ���ٶ�
        nav.velMeasure[3] = 0;              //Z����ѹ���ٶ�
        nav.velMeasure[4] = 0;                              //TOF�����ٶ�
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
    //�ٶȻ���
    input.x = nav.velocity.x * deltaT;
    input.y = nav.velocity.y * deltaT;
    input.z = nav.velocity.z * deltaT;

    //λ�ø���
    KalmanUpdate(&kalmanPos, input, nav.posMeasure, fuseFlag);
    nav.position = kalmanPos.state;
}


/**********************************************************************************************************
*�� �� ��: KalmanVelInit
*����˵��: �����ٶȹ��ƵĿ������ṹ���ʼ��
*��    ��: R�̶���QԽ�󣬴���Խ���β���ֵ��Q�������ֻ�ò���ֵ����֮��QԽС����Խ����ģ��Ԥ��ֵ��QΪ������ֻ��ģ��Ԥ�⡣
*�� �� ֵ: �� q 0.2 r 5 ������
**************************************************************************************************************************************************/
static void KalmanVelInit(void)
{
    float qMatInit[6][6] = {{0.1, 0, 0, 0, 0, 0},
                            {0, 0.1, 0, 0, 0, 0},
                            {0, 0, 0.05, 0, 0, 0},      
                            {0.03, 0, 0, 0, 0, 0},
                            {0, 0.03, 0, 0, 0, 0},
                            {0, 0, 0.03, 0, 0, 0}};

    float rMatInit[6][6] = {{200, 0, 0, 0, 0, 0},          //�ٶ�x��������������
                            {0, 200, 0, 0, 0, 0},          //�ٶ�y��������������
                            {0, 0, 200, 0, 0, 0},          //GPS�ٶ�z��������������       
                            {0, 0, 0, 2500, 0, 0},         //��ѹ�ٶ�������������
                            {0, 0, 0, 0, 2000, 0},         //TOF�ٶ�������������
                            {0, 0, 0, 0, 0, 500000}};      //z���ٶȸ�ͨ�˲�ϵ��

    float pMatInit[6][6] = {{20, 0, 0, 0, 0, 0},
                            {0, 20, 0, 0, 0, 0},
                            {0, 0, 5, 0, 0, 0},      
                            {2, 0, 0, 2, 0, 0},
                            {0, 2, 0, 0, 2, 0},
                            {0, 0, 6, 0, 0, 2}};    //����Э����P�ĳ�ֵ��������߳�ʼ��ʱbias�������ٶ�

    float hMatInit[6][6] = {{1, 0, 0, 0, 0, 0},
                            {0, 1, 0, 0, 0, 0},
                            {0, 0, 1, 0, 0, 0},      
                            {0, 0, 1, 0, 0, 0},
                            {0, 0, 1, 0, 0, 0},
                            {0, 0, 1, 0, 0, 0}};    //h[5][2]:�ٶ�z�����������ͨ�˲�Ч���

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
    
	//��ʼ���������˲�������ؾ���
    KalmanVelQMatSet(&kalmanVel, qMatInit);
    KalmanVelRMatSet(&kalmanVel, rMatInit);
    KalmanVelCovarianceMatSet(&kalmanVel, pMatInit);
    KalmanVelObserveMapMatSet(&kalmanVel, hMatInit);
    KalmanVelStateTransMatSet(&kalmanVel, fMatInit);
    KalmanVelBMatSet(&kalmanVel, bMatInit);
                            
    ////״̬�������ڣ����ڽ��������״̬��������۲���֮�����λ������
	kalmanVel.slidWindowSize = 250;
    kalmanVel.stateSlidWindow = mymalloc(kalmanVel.slidWindowSize * sizeof(Vector3f_t));
    kalmanVel.fuseDelay[FLOW_VEL_X] = 50;    //�ٶ�x�������ӳٲ�����0.22s
    kalmanVel.fuseDelay[FLOW_VEL_Y] = 50;    //�ٶ�y�������ӳٲ�����0.22s
    kalmanVel.fuseDelay[FLOW_VEL_Z] = 50;    //�ٶ�z�������ӳٲ�����0.22s
    kalmanVel.fuseDelay[BARO_VEL]  = 50;     //��ѹ�ٶ������ӳٲ�����0.1s
    kalmanVel.fuseDelay[TOF_VEL]   = 30;     //TOF�ٶ������ӳٲ�����
}

/**********************************************************************************************************
*�� �� ��: KalmanPosInit
*����˵��: λ�ù��ƵĿ������ṹ���ʼ��
*��    ��: ��
*�� �� ֵ: ��
*****************************************************************************************************************/
static void KalmanPosInit(void)
{
    float qMatInit[9] = {0.5, 0, 0, 0, 0.5, 0, 0, 0, 0.5};
    float rMatInit[9] = {20, 0,  0, 0,20, 0, 0, 0, 50};
    float pMatInit[9] = {10, 0, 0, 0, 10, 0, 0, 0, 10};
    float fMatInit[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    float hMatInit[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    float bMatInit[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    //��ʼ���������˲�������ؾ���  
	KalmanQMatSet(&kalmanPos, qMatInit);
    KalmanRMatSet(&kalmanPos, rMatInit);
    KalmanBMatSet(&kalmanPos, bMatInit);
    KalmanCovarianceMatSet(&kalmanPos, pMatInit);
    KalmanStateTransMatSet(&kalmanPos, fMatInit);
    KalmanObserveMapMatSet(&kalmanPos, hMatInit);

    //״̬�������ڣ����ڽ��������״̬��������۲���֮�����λ������
    kalmanPos.slidWindowSize = 200;
    kalmanPos.statusSlidWindow = mymalloc(kalmanPos.slidWindowSize * sizeof(kalmanPos.state));
    kalmanPos.fuseDelay.x = 20;    //0.05s��ʱ
    kalmanPos.fuseDelay.y = 20;    //0.05s��ʱ
    kalmanPos.fuseDelay.z = 100;    //0.05s��ʱ
}

/**********************************************************************************************************
*�� �� ��: NavigationReset
*����˵��: ����������ݸ�λ
*��    ��: ��
*�� �� ֵ: ��
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
*�� �� ��: GetCopterAccel
*����˵��: ��ȡ���м��ٶ�
*��    ��: ��
*�� �� ֵ: ���ٶ�ֵ
************************************************************************************/
Vector3f_t GetCopterAccel(void)
{
    return nav.accel;
}

/**********************************************************************************************************
*�� �� ��: GetAccelBias
*����˵��: ��ȡ���ٶ�bias
*��    ��: ��
*�� �� ֵ: ���ٶ�biasֵ
**********************************************************************************************************/
Vector3f_t GetAccelBias(void)
{
    return nav.accel_bias;
}

/**********************************************************************************************************
*�� �� ��: GetCopterVelocity
*����˵��: ��ȡ�����ٶȹ���ֵ
*��    ��: ��
*�� �� ֵ: �ٶ�ֵ
**********************************************************************************************************/
Vector3f_t GetCopterVelocity(void)
{
    return nav.velocity;
}

/**********************************************************************************************************
*�� �� ��: GetCopterVelMeasure
*����˵��: ��ȡ�����ٶȲ���ֵ
*��    ��: ��
*�� �� ֵ: �ٶ�ֵ
**********************************************************************************************************/
float* GetCopterVelMeasure(void)
{
    return nav.velMeasure;
}

/**********************************************************************************************************
*�� �� ��: GetCopterPosition
*����˵��: ��ȡλ�ù���ֵ
*��    ��: ��
*�� �� ֵ: λ��ֵ
**********************************************************************************************************/
Vector3f_t GetCopterPosition(void)
{
    return nav.position;
}

/**********************************************************************************************************
*�� �� ��: GetCopterPosMeasure
*����˵��: ��ȡλ�ò���ֵ
*��    ��: ��
*�� �� ֵ: �ٶ�ֵ
**********************************************************************************************************/
Vector3f_t GetCopterPosMeasure(void)
{
    return nav.posMeasure;
}
