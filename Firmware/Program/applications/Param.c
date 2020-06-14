
#include "Param.h"
#include "Control.h"
#include "include.h"
struct _save_param_st_pk Param;
_parameter_state_st para_sta;

void Param_Init()
{
	Param.firstintiflag = FIRST_INIT_FLAG;
	Param.hardware = 200;
	Param.software = 100;
	
	Param.auto_take_off_height = 20;
	Param.auto_take_off_speed = 20;
	Param.auto_landing_speed = 20;
	Param.idle_speed_pwm = 20;
	
	Param.PID_rol.kp = 800;
	Param.PID_rol.ki = 0;
	Param.PID_rol.kd = 10;

	Param.PID_pit.kp = 800;
	Param.PID_pit.ki = 0;
	Param.PID_pit.kd = 10;
	
	Param.PID_yaw.kp = 4000;
	Param.PID_yaw.ki = 0;
	Param.PID_yaw.kd = 100;
	
	Param.PID_loc.kp = 3000;
	Param.PID_loc.ki = 0;
	Param.PID_loc.kd = 800;

	Param.PID_high.kp = 2000;
	Param.PID_high.ki = 0;
	Param.PID_high.kd = 100;

	Param.PID_rol_s.kp = 2000;
	Param.PID_rol_s.ki = 800;
	Param.PID_rol_s.kd = 60;
	
	Param.PID_pit_s.kp = 2000;
	Param.PID_pit_s.ki = 800;
	Param.PID_pit_s.kd = 60;
	
	Param.PID_yaw_s.kp = 6000;
	Param.PID_yaw_s.ki = 0;
	Param.PID_yaw_s.kd = 200;

	Param.PID_loc_s.kp = 40;
	Param.PID_loc_s.ki = 0;
	Param.PID_loc_s.kd = 2;
	
	Param.PID_high_s.kp = 2000;
	Param.PID_high_s.ki = 2000;
	Param.PID_high_s.kd = 100;


	Param_Save();
}
static void Parame_Copy_Para2fc()
{

	for(u8 i = 0;i<3;i++)
	{	
		save.acc_offset[i]=Param.acc_offset[i]	;
		save.acc_scale[i]=Param.acc_scale[i]	;
		save.gyro_offset[i]=Param.gyro_offset[i]	;
		save.surface_vec[i]=Param.surface_vec[i]	;
		
		//center_pos参数不需要反向赋值
	}
	Param.gyr_temprea_offset=0;
	Param.acc_temprea_offset=0;
}
static void Parame_Copy_Fc2para()
{

	for(u8 i = 0;i<3;i++)
	{	
		Param.acc_offset[i]	=	save.acc_offset[i];
		Param.acc_scale[i]	=	save.acc_scale[i];
		Param.gyro_offset[i]=	save.gyro_offset[i];
		Param.surface_vec[i]=	save.surface_vec[i];
		
		//center_pos参数不需要反向赋值
	}
	Param.gyr_temprea_offset=0;
	Param.acc_temprea_offset=0;
}
void Param_Read(void)
{
	u8 size,len;
	
	len =  sizeof(Param);
	size = len/(4+(len%4)?1:0); //保存的数据长度
	
	STMFLASH_Read(EEPROM_START_ADDRESS,(u32 *)(&Param),size);
	Parame_Copy_Para2fc();
	if(Param.firstintiflag != FIRST_INIT_FLAG)//板子从未初始化
	{
		
		Param_Init();
		
		sensor.acc_CALIBRATE = 1;	
		sensor.gyr_CALIBRATE = 1;
		save_pid_en = 1;
	}

}
void data_save(void)
{
	para_sta.save_en = !flag.fly_ready;
	para_sta.save_trig = 1;
}

void Param_Save(void)
{
	u32 data_len;
	
	data_len = sizeof(Param);
	data_len = data_len/4+((data_len%4)?1:0);
	
	Parame_Copy_Fc2para();
	
	STMFLASH_Write(EEPROM_START_ADDRESS,(u32*)(&Param),data_len);
	All_PID_Init();
	pid_init();
}

u16 save_pid_en = 0;
void PID_Save_Overtime(u16 ms,u16 dTms)
{
	if(save_pid_en!=0)
	{
		save_pid_en++;
	}
	
	if(save_pid_en>=(ms/dTms))
	{
		Param_Save();
		save_pid_en = 0;
	}

}






