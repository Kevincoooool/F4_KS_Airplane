
#include "show.h"
#include "oledfont.h" 
#include "oled.h"

#define Line1_Begin 29
#define Line2_Begin 5
#define Line3_Begin 5
#define Line4_Begin 30
#define Line5_Begin 0

#define X_Begin 0
#define Y_Begin 51
#define Z_Begin 103

#define Line1_Begin1 0
#define Line2_Begin1 0
#define Line3_Begin1 40
#define Line4_Begin1 0
#define Line5_Begin1 0

#define Y0 0
#define Y1 14
#define Y2 Y1+12
#define Y3 Y2+12
#define Y4 Y3+12
#define Y5 Y4+12

struct _Show Show;

unsigned char i;          //��������
unsigned char Send_Count; //������Ҫ���͵����ݸ���
float Vol;

/**************************************************************************
�������ܣ�OLED��ʾ
��ڲ�������
����  ֵ����
**************************************************************************/
void oled_show(void)
{
	u8 temp;
	int temp1;
	
	static u8 page,page_temp,flash_cnt,show_mode=1;
	
	if(page != page_temp)//�л�ҳ��������
	{
		page_temp = page;
		OLED_Clear();
	}
	///////////////////////////////��һ��///////////////////////////////////
	//��ʾ��ʾ
	if(Show.low_power)//ң�ص͵���
	{
		flash_cnt ++;
		if(flash_cnt>3) 
		{
			flash_cnt=0;
			if(show_mode==0)show_mode=1;
			else show_mode = 0;
		}
		
		for(u8 i=0;i<12;i++) OLED_Show_CH_String(Line1_Begin+i*6,Y0,oled_CH16[i],12,show_mode);
	}
	else if(Show.test_flag&BIT5)//�ɻ��͵���
	{
		flash_cnt ++;
		if(flash_cnt>3) 
		{
			flash_cnt=0;
			if(show_mode==0)show_mode=1;
			else show_mode = 0;
		}
		
		for(u8 i=0;i<12;i++) OLED_Show_CH_String(Line1_Begin+i*6,Y0,oled_CH17[i],12,show_mode);
	}
	else
	{
		OLED_ShowString(Line1_Begin+00,Y0,"KS DIY TC",12,1);                                                
	}
	//////////////////////////////////////////////////////
	if(Show.NRF_Err)	OLED_Show_CH(2,Y0,6 ,12,1);//����ģ�������ʾX
	else				OLED_ShowNumber(2,Y0,Param.NRF_Channel,3,12);//��ʾ�����ŵ�
	
	//��ʾ�ź�ǿ��
	temp = Show.Rc_num/20;
	switch(temp)
	{
		case 0:OLED_Show_CH(Line1_Begin+85,Y0,6 ,12,1);break;
		case 1:OLED_Show_CH(Line1_Begin+85,Y0,7 ,12,1);break;
		case 2:OLED_Show_CH(Line1_Begin+85,Y0,8 ,12,1);break;
		case 3:OLED_Show_CH(Line1_Begin+85,Y0,9 ,12,1);break;
		case 4:OLED_Show_CH(Line1_Begin+85,Y0,10,12,1);break;
		default:OLED_Show_CH(Line1_Begin+85,Y0,11,12,1);break;
	}
	///////////////////////////////�ڶ���///////////////////////////////////
	//��ʾ��ѹ	
	OLED_ShowString(Line2_Begin+00,Y1,"R-V:",12,1);OLED_ShowString(Line2_Begin+36,Y1,".",12,1);
	OLED_ShowNumber(Line2_Begin+30,Y1,Show.Battery_Rc/100,1,12);OLED_ShowNumber(Line2_Begin+42,Y1,Show.Battery_Rc%100,2,12);
	if(Show.Battery_Rc%100<10)	OLED_ShowNumber(Line2_Begin+42,Y1,0,1,12);
	
	OLED_ShowString(Line2_Begin+64,Y1,"F-V:",12,1);OLED_ShowString(Line2_Begin+100,Y1,".",12,1);
	OLED_ShowNumber(Line2_Begin+94,Y1,Show.Battery_Fly/100,1,12);OLED_ShowNumber(Line2_Begin+106,Y1,Show.Battery_Fly%100,2,12);
	if(Show.Battery_Fly%100<10)	OLED_ShowNumber(Line2_Begin+106,Y1,0,1,12);
	///////////////////////////////����������/////////////////////////////////
	//��ʾң������
	OLED_ShowString(Line3_Begin+00,Y2,"THR:",12,1);
	temp = (Rc.THR-1000)/41;
	OLED_Show_progress_bar(temp,12,24,Line4_Begin+6 ,Y2,12,1);
	temp = (Rc.THR-1500)/41;
	OLED_Show_progress_bar(temp,12,12,Line4_Begin+18,Y2,12,1);
	
	OLED_ShowString(Line3_Begin+64,Y2,"ROL:",12,1);
	temp = (Rc.ROL-1000)/41;
	OLED_Show_progress_bar(temp,12,24,Line4_Begin+70,Y2,12,1);
	temp = (Rc.ROL-1500)/41;
	OLED_Show_progress_bar(temp,12,12,Line4_Begin+82,Y2,12,1);
	
	OLED_ShowString(Line3_Begin+00,Y3,"YAW:",12,1);
	temp = (Rc.YAW-1000)/41;
	OLED_Show_progress_bar(temp,12,24,Line4_Begin+6 ,Y3,12,1);
	temp = (Rc.YAW-1500)/41;
	OLED_Show_progress_bar(temp,12,12,Line4_Begin+18,Y3,12,1);
	
	OLED_ShowString(Line3_Begin+64,Y3,"PIT:",12,1);
	temp = (Rc.PIT-1000)/41;
	OLED_Show_progress_bar(temp,12,24,Line4_Begin+70,Y3,12,1);
	temp = (Rc.PIT-1500)/41;
	OLED_Show_progress_bar(temp,12,12,Line4_Begin+82,Y3,12,1);
	///////////////////////////////������///////////////////////////////////
//	if(Show.Rc_num)
//	{
//		page = 1;//��һҳ
		//��ʾ��̬�ǶȺ͸߶�
		if(Show.X<0) temp1 = -Show.X,OLED_ShowString(X_Begin,Y4,"-",12,1);
		else         temp1 = Show.X, OLED_ShowString(X_Begin,Y4,"+",12,1);
		OLED_ShowNumber(X_Begin+8,Y4,temp1/10,3,12);
		OLED_ShowString(X_Begin+28,Y4,".",12,1);
		OLED_ShowNumber(X_Begin+32,Y4,temp1%10,1,12);
		
		if(Show.Y<0) temp1 = -Show.Y,OLED_ShowString(Y_Begin,Y4,"-",12,1);
		else    temp1 = Show.Y, OLED_ShowString(Y_Begin,Y4,"+",12,1);
		OLED_ShowNumber(Y_Begin+8,Y4,temp1/10,3,12);
		OLED_ShowString(Y_Begin+28,Y4,".",12,1);
		OLED_ShowNumber(Y_Begin+32,Y4,temp1%10,1,12);
		
		if(Show.H<0) 			temp1 =-Show.H,  OLED_ShowString(Z_Begin,Y4,"-",12,1);
		else    			temp1 = Show.H,  OLED_ShowString(Z_Begin,Y4,"+",12,1);
		//�����ʾ999����,Ҳ����9.99�ס�
		if(temp1>999) temp1 = 999,OLED_ShowString(Z_Begin,Y4,">",12,1);
		OLED_ShowNumber(Z_Begin+8,Y4,temp1,3,12);
	
	OLED_Refresh_Gram();//��ʼ��ʾ
}
//��������ʾ����
void OLED_Show_progress_bar(u8 temp,u8 chr_star,u8 chr_default,u8 x,u8 y,u8 size,u8 mode)
{
	switch(temp)
	{
		case  0:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case  1:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case  2:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case  3:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case  4:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case  5:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case  6:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case  7:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case  8:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case  9:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case 10:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case 11:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		case 12:OLED_Show_CH(x,y,chr_star+temp,size,size);break;
		
		default:OLED_Show_CH(x,y,chr_default,size,size);break;
	}
}

vs16 set_bit=0,bit_max=4;
u16 set_temp=0x0000;

void OLED_Show_Seting(void)
{
	u8 cnt=0,bit_cnt=0;
	u8 mode,page;
	static u8 page_temp;
	
	for(u8 i=0;i<8;i++) OLED_Show_CH_String(40+i*6,cnt,oled_CH0[i],12,1);

	page = set_bit/4;
	bit_cnt = 4*page;
	if(page_temp!=page)
	{
		page_temp=page;
		OLED_Clear();
	}
	
	switch(page)
	{
		case 0:
			cnt+=13;
			for(u8 i=0;i<10;i++) OLED_Show_CH_String(0+i*6,cnt,oled_CH1[i],12,1);
			if(set_bit==bit_cnt) mode=0;	else mode=1;
			if( set_temp&BIT0 )	for(u8 i=0;i<6;i++) OLED_Show_CH_String(94+i*6,cnt,oled_CH12[i],12,mode);
			else								for(u8 i=0;i<6;i++) OLED_Show_CH_String(94+i*6,cnt,oled_CH13[i],12,mode);
			bit_cnt++;

			cnt+=13;
			for(u8 i=0;i<10;i++) OLED_Show_CH_String(0+i*6,cnt,oled_CH2[i],12,1);
			if(set_bit==bit_cnt) mode=0;	else mode=1;
			if( set_temp&BIT1 )	for(u8 i=0;i<6;i++) OLED_Show_CH_String(94+i*6,cnt,oled_CH12[i],12,mode);
			else								for(u8 i=0;i<6;i++) OLED_Show_CH_String(94+i*6,cnt,oled_CH13[i],12,mode);
			bit_cnt++;

			cnt+=13;
			for(u8 i=0;i<10;i++) OLED_Show_CH_String(0+i*6,cnt,oled_CH3[i],12,1);
			if(set_bit==bit_cnt) mode=0; else mode=1;
			if( set_temp&BIT2 )	for(u8 i=0;i<6;i++) OLED_Show_CH_String(94+i*6,cnt,oled_CH12[i],12,mode);
			else								for(u8 i=0;i<6;i++) OLED_Show_CH_String(94+i*6,cnt,oled_CH13[i],12,mode);
			bit_cnt++;

			cnt+=13;
			for(u8 i=0;i<10;i++) OLED_Show_CH_String(0+i*6,cnt,oled_CH4[i],12,1);
			if(set_bit==bit_cnt)	mode=0; else mode=1;
			if( set_temp&BIT3 )	for(u8 i=0;i<6;i++) OLED_Show_CH_String(94+i*6,cnt,oled_CH12[i],12,mode);
			else								for(u8 i=0;i<6;i++) OLED_Show_CH_String(94+i*6,cnt,oled_CH13[i],12,mode);
			bit_cnt++;
		break;
		
		case 1:
			cnt+=13;
			for(u8 i=0;i<10;i++) OLED_Show_CH_String(0+i*6,cnt,oled_CH14[i],12,1);
			if(set_bit==bit_cnt)	mode=0; else mode=1;
			if( set_temp&BIT4 )	for(u8 i=0;i<6;i++) OLED_Show_CH_String(94+i*6,cnt,oled_CH12[i],12,mode);
			else								for(u8 i=0;i<6;i++) OLED_Show_CH_String(94+i*6,cnt,oled_CH13[i],12,mode);
			bit_cnt++;
		break;
		
		default:break;
	}
	
	OLED_Refresh_Gram();//��ʼ��ʾ
}

vs16 test_bit=0,test_max=4;
u16  test_temp=0;

void OLED_Show_Test(void)
{
	u8 cnt=0,bit_cnt=0;
	u8 mode,page;
	static u8 page_temp;
	
	test_temp = Show.test_flag;
	
	for(u8 i=0;i<10;i++) OLED_Show_CH_String(34+i*6,cnt,oled_CH5[i],12,1);
	
	page = test_bit/4;
	bit_cnt = 4*page;
	if(page_temp!=page)	
	{
		page_temp=page;
		OLED_Clear();
	}
	
	switch(page)
	{
		case 0:
			cnt+=13;
			for(u8 i=0;i<8;i++) OLED_Show_CH_String(0+i*6,cnt,oled_CH6[i],12,1);
			if(test_bit==bit_cnt++) mode=0; else mode=1;
			if( test_temp&BIT0 )for(u8 i=0;i<4;i++) OLED_Show_CH_String(104+i*6,cnt,oled_CH10[i],12,mode);
			else								for(u8 i=0;i<4;i++) OLED_Show_CH_String(104+i*6,cnt,oled_CH11[i],12,mode);
			
			cnt+=13;
			for(u8 i=0;i<8;i++) OLED_Show_CH_String(0+i*6,cnt,oled_CH7[i],12,1);
			if(test_bit==bit_cnt++) mode=0; else mode=1;
			if( test_temp&BIT1 )for(u8 i=0;i<4;i++) OLED_Show_CH_String(104+i*6,cnt,oled_CH10[i],12,mode);
			else								for(u8 i=0;i<4;i++) OLED_Show_CH_String(104+i*6,cnt,oled_CH11[i],12,mode);
			
			cnt+=13;
			for(u8 i=0;i<10;i++) OLED_Show_CH_String(0+i*6,cnt,oled_CH8[i],12,1);
			if(test_bit==bit_cnt++) mode=0; else mode=1;
			if( test_temp&BIT2 )for(u8 i=0;i<4;i++) OLED_Show_CH_String(104+i*6,cnt,oled_CH10[i],12,mode);
			else								for(u8 i=0;i<4;i++) OLED_Show_CH_String(104+i*6,cnt,oled_CH11[i],12,mode);
			
			cnt+=13;
			for(u8 i=0;i<12;i++) OLED_Show_CH_String(0+i*6,cnt,oled_CH9[i],12,1);
			if(test_bit==bit_cnt++) mode=0; else mode=1;
			if( test_temp&BIT3 )for(u8 i=0;i<4;i++) OLED_Show_CH_String(104+i*6,cnt,oled_CH10[i],12,mode);
			else								for(u8 i=0;i<4;i++) OLED_Show_CH_String(104+i*6,cnt,oled_CH11[i],12,mode);
		break;
		
		case 1:
			cnt+=13;
			for(u8 i=0;i<14;i++) OLED_Show_CH_String(0+i*6,cnt,oled_CH15[i],12,1);
			if(test_bit==bit_cnt++) mode=0; else mode=1;
			if( test_temp&BIT4 )for(u8 i=0;i<4;i++) OLED_Show_CH_String(104+i*6,cnt,oled_CH10[i],12,mode);
			else								for(u8 i=0;i<4;i++) OLED_Show_CH_String(104+i*6,cnt,oled_CH11[i],12,mode);
		break;
		
		default:break;
	}

	OLED_Refresh_Gram();//��ʼ��ʾ
}

//ң�˼��
void Gesture_Check0(u16 val,vs16 *set,vs16 max)
{
	static u8 cnt0,cnt1;
	
	if(val>1800) 
	{
		cnt0++;
	}
	else
	{
		if(cnt0>2) (*set)--;
		cnt0 = 0;
	}
	
	if(val<1200) 
	{
		cnt1++;
	}
	else
	{
		if(cnt1>2) (*set)++;
		cnt1 = 0;
	}
	
	if((*set)<0) 				(*set) = max;
	else if((*set)>max) (*set) = 0;
}

void Gesture_Check1(u16 val,u16 *set,vs16 bit)
{
	static u8 cnt0;
	
	if(val>1800 || val<1200) 
	{
		cnt0++;
	}
	else
	{
		if(cnt0>2) 
			*set = REVERSE(*set,bit);
		cnt0 = 0;
	}
}

u8 send_flag=0;

void Gesture_Check(void)
{
	static u8 temp;
	
	if(temp!=Show.windows)
	{
		if(Show.windows==1) set_temp = Show.set_flag;
		if(temp==1 && set_temp!=Show.set_flag) send_flag = 1;
		temp=Show.windows;
	}
	switch(Show.windows)
	{
		case 1:
			Gesture_Check0(Rc.PIT,&set_bit,bit_max);
			if(Show.Connect_Succeed)
			{
				Gesture_Check1(Rc.ROL,&set_temp,set_bit);
			}
		break;
		
		case 2:
			Gesture_Check0(Rc.PIT,&test_bit,test_max);
		break;
		
		default:break;
	}
}

void Show_Duty(void)
{
	static u8 temp;
	
	if(Show.windows!=temp)
	{
		temp = Show.windows;
		OLED_Clear();
	}
	switch(Show.windows)
	{
		case 0:	oled_show();	break;
		case 1:	OLED_Show_Seting();	break;
		case 2:	OLED_Show_Test();	break;
		default:break;
	}
}
