#include "vl53l0x_platform.h"

/** Maximum buffer size to be used in i2c */
#define VL53L0X_MAX_I2C_XFER_SIZE   64 /* Maximum buffer size to be used in i2c */
#define DECL_I2C_BUFFER  uint8_t LocBuffer[VL53L0X_MAX_I2C_XFER_SIZE];
#define VL53L0X_GetLocalBuffer(Dev, n_byte)  LocBuffer

#define VL53L0X_I2C_USER_VAR         /* none but could be for a flag var to get/pass to mutex interruptible  return flags and try again */
#define VL53L0X_GetI2CAccess(Dev)    /* todo mutex acquire */
#define VL53L0X_DoneI2CAcces(Dev)    /* todo mutex release */


VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV Dev){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    return Status;
}

VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV Dev){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    return Status;
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count){

    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int = 0;
	uint8_t deviceAddress;

    if (count>=VL53L0X_MAX_I2C_XFER_SIZE){
        Status = VL53L0X_ERROR_INVALID_PARAMS;
    }

	deviceAddress = Dev->I2cDevAddr;

	status_int = VL53L0X_write_multi(deviceAddress, index, pdata, count);

	if (status_int != 0)
		Status = VL53L0X_ERROR_CONTROL_INTERFACE;

    return Status;
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count){
    VL53L0X_I2C_USER_VAR
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;
	uint8_t deviceAddress;

    if (count>=VL53L0X_MAX_I2C_XFER_SIZE){
        Status = VL53L0X_ERROR_INVALID_PARAMS;
    }

    deviceAddress = Dev->I2cDevAddr;

	status_int = VL53L0X_read_multi(deviceAddress, index, pdata, count);

	if (status_int != 0)
		Status = VL53L0X_ERROR_CONTROL_INTERFACE;

    return Status;
}


VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;
	uint8_t deviceAddress;

    deviceAddress = Dev->I2cDevAddr;

	status_int = VL53L0X_write_byte(deviceAddress, index, data);

	if (status_int != 0)
		Status = VL53L0X_ERROR_CONTROL_INTERFACE;

    return Status;
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;
	uint8_t deviceAddress;

    deviceAddress = Dev->I2cDevAddr;

	status_int = VL53L0X_write_word(deviceAddress, index, data);

	if (status_int != 0)
		Status = VL53L0X_ERROR_CONTROL_INTERFACE;

    return Status;
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;
	uint8_t deviceAddress;

    deviceAddress = Dev->I2cDevAddr;

	status_int = VL53L0X_write_dword(deviceAddress, index, data);

	if (status_int != 0)
		Status = VL53L0X_ERROR_CONTROL_INTERFACE;

    return Status;
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;
    uint8_t deviceAddress;
    uint8_t data;

    deviceAddress = Dev->I2cDevAddr;

    status_int = VL53L0X_read_byte(deviceAddress, index, &data);

    if (status_int != 0)
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;

    if (Status == VL53L0X_ERROR_NONE) {
        data = (data & AndData) | OrData;
        status_int = VL53L0X_write_byte(deviceAddress, index, data);

        if (status_int != 0)
            Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    return Status;
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;
    uint8_t deviceAddress;

    deviceAddress = Dev->I2cDevAddr;

    status_int = VL53L0X_read_byte(deviceAddress, index, data);

    if (status_int != 0)
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;

    return Status;
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;
    uint8_t deviceAddress;

    deviceAddress = Dev->I2cDevAddr;

    status_int = VL53L0X_read_word(deviceAddress, index, data);

    if (status_int != 0)
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;

    return Status;
}

VL53L0X_Error  VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data){
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;
    uint8_t deviceAddress;

    deviceAddress = Dev->I2cDevAddr;

    status_int = VL53L0X_read_dword(deviceAddress, index, data);

    if (status_int != 0)
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;

    return Status;
}

#define VL53L0X_POLLINGDELAY_LOOPNB  250
VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev){
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    volatile uint32_t i;

    for(i=0;i<VL53L0X_POLLINGDELAY_LOOPNB;i++){
        //Do nothing
        ;;
    }

    return status;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "vl53l0x_api.h"
VL53L0X_Dev_t vl53l0x_dev;
VL53L0X_DeviceInfo_t vl53l0x_dev_info;
VL53L0X_RangingMeasurementData_t vl53l0x_data;
VL53L0X_Error vl53l0x_status;

VL53L0X_Error vl53l0x_measure_init ( VL53L0X_Dev_t *pMyDevice )
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    //FixPoint1616_t LimitCheckCurrent;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;
    // Device Initialization
    Status = VL53L0X_StaticInit ( pMyDevice );
    if ( Status != VL53L0X_ERROR_NONE )
    {
        return Status;
    }
	if(1)
	{
		Status = VL53L0X_SetReferenceSpads ( pMyDevice, 5, 0 );
		if ( Status != VL53L0X_ERROR_NONE )
		{
			return Status;
		}
		Status = VL53L0X_SetRefCalibration ( pMyDevice, 0x1D, 1 );
		if ( Status != VL53L0X_ERROR_NONE )
		{
			return Status;
		}
	}
	else
	{
		//校准SPAD，校准后的两个结果refSpadCount，isApertureSpads
		Status = VL53L0X_PerformRefSpadManagement ( pMyDevice, &refSpadCount, &isApertureSpads );
		if ( Status != VL53L0X_ERROR_NONE )
		{
			return Status;
		}
		//校准温度相关参数，校准后的两个结果VhvSettings，PhaseCal
		Status = VL53L0X_PerformRefCalibration ( pMyDevice, &VhvSettings, &PhaseCal );
		if ( Status != VL53L0X_ERROR_NONE )
		{
			return Status;
		}
	}
    Status = VL53L0X_SetDeviceMode ( pMyDevice, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING ); 
    if ( Status != VL53L0X_ERROR_NONE )
    {
        return Status;
    }

    if ( Status == VL53L0X_ERROR_NONE )
    {
        Status = VL53L0X_SetLimitCheckEnable ( pMyDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1 );
    }
    if ( Status == VL53L0X_ERROR_NONE )
    {
        Status = VL53L0X_SetLimitCheckEnable ( pMyDevice, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1 );
    }

    if ( Status == VL53L0X_ERROR_NONE )
    {
        Status = VL53L0X_SetLimitCheckValue ( pMyDevice, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, ( FixPoint1616_t ) ( 0.1 * 65536 ) );
    }
    if ( Status == VL53L0X_ERROR_NONE )
    {
        Status = VL53L0X_SetLimitCheckValue ( pMyDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, ( FixPoint1616_t ) ( 60 * 65536 ) );
    }
    if ( Status == VL53L0X_ERROR_NONE )
    {
        Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds ( pMyDevice, 33000 );
    }

    if ( Status == VL53L0X_ERROR_NONE )
    {
        Status = VL53L0X_SetVcselPulsePeriod ( pMyDevice, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18 );
    }
    if ( Status == VL53L0X_ERROR_NONE )
    {
        Status = VL53L0X_SetVcselPulsePeriod ( pMyDevice, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14 );
    }

    if ( Status == VL53L0X_ERROR_NONE )
    {
        Status = VL53L0X_StartMeasurement ( pMyDevice );
    }

    return Status;
}

VL53L0X_Error VL53L0X_Init ( void )
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_Dev_t *pMyDevice = &vl53l0x_dev;
    pMyDevice->I2cDevAddr      = 0x52;
    pMyDevice->comms_type      =  1;
    pMyDevice->comms_speed_khz =  400;

    Status = VL53L0X_DataInit ( pMyDevice ); // Data initialization
    if ( Status != VL53L0X_ERROR_NONE )
    {
        return Status;
    }
    Status = VL53L0X_GetDeviceInfo ( pMyDevice, &vl53l0x_dev_info );
    if ( Status != VL53L0X_ERROR_NONE )
    {
        return Status;
    }
    if ( ( vl53l0x_dev_info.ProductRevisionMajor != 1 ) && ( vl53l0x_dev_info.ProductRevisionMinor != 1 ) )
    {
        Status = VL53L0X_ERROR_NOT_SUPPORTED;
        return Status;
    }
    Status = vl53l0x_measure_init ( pMyDevice );
    vl53l0x_status = Status;
    if ( Status != VL53L0X_ERROR_NONE )
    {
        return Status;
    }

    return Status;
}
u8 VL53L0X_IfDataReady ( void )
{
	uint32_t InterruptMask;

	VL53L0X_GetInterruptMaskStatus ( &vl53l0x_dev, &InterruptMask );//IIC通信1字节
    if ( InterruptMask == VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY )
		return 1;
	else
		return 0;
}
u16 VL53L0X_FastRead( void )
{
	u8 datatemp[2];
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	
	Status = VL53L0X_ReadMulti ( &vl53l0x_dev, 0x1E, datatemp, 2 );	//IIC通信2字节
	//VL53L0X_ClearInterruptMask ( &vl53l0x_dev, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY );//IIC通信3字节
	if(Status != VL53L0X_ERROR_NONE)
		return 0xffff;
	return VL53L0X_MAKEUINT16 ( datatemp[1], datatemp[0] );
}

