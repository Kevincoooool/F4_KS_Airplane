#include "malloc.h"	   

//�ڴ��(32�ֽڶ���)
__align(32) u8 mem1base[MEM1_MAX_SIZE];													//�ڲ�SRAM�ڴ��
//__align(32) u8 mem2base[MEM2_MAX_SIZE] __attribute__((at(0X68000000)));					//�ⲿSRAM�ڴ��
//__align(32) u8 mem3base[MEM3_MAX_SIZE] __attribute__((at(0X10000000)));					//�ڲ�CCM�ڴ��
//�ڴ�����
u16 mem1mapbase[MEM1_ALLOC_TABLE_SIZE];													//�ڲ�SRAM�ڴ��MAP
//u16 mem2mapbase[MEM2_ALLOC_TABLE_SIZE] __attribute__((at(0X68000000+MEM2_MAX_SIZE)));	//�ⲿSRAM�ڴ��MAP
//u16 mem3mapbase[MEM3_ALLOC_TABLE_SIZE] __attribute__((at(0X10000000+MEM3_MAX_SIZE)));	//�ڲ�CCM�ڴ��MAP
//�ڴ�������	   
const u32 memtblsize[SRAMBANK]={MEM1_ALLOC_TABLE_SIZE};	//�ڴ���С
const u32 memblksize[SRAMBANK]={MEM1_BLOCK_SIZE};					//�ڴ�ֿ��С
const u32 memsize[SRAMBANK]={MEM1_MAX_SIZE};							//�ڴ��ܴ�С


//�ڴ���������
struct _m_mallco_dev mallco_dev=
{
	my_mem_init,						//�ڴ��ʼ��
	my_mem_perused,						//�ڴ�ʹ����
	mem1base,			//�ڴ��
	mem1mapbase,//�ڴ����״̬��
	0,  		 					//�ڴ����δ����
};

//�����ڴ�
//*des:Ŀ�ĵ�ַ
//*src:Դ��ַ
//n:��Ҫ���Ƶ��ڴ泤��(�ֽ�Ϊ��λ)
void mymemcpy(void *des,void *src,u32 n)  
{  
    u8 *xdes=des;
	u8 *xsrc=src; 
    while(n--)*xdes++=*xsrc++;  
}  
//�����ڴ�
//*s:�ڴ��׵�ַ
//c :Ҫ���õ�ֵ
//count:��Ҫ���õ��ڴ��С(�ֽ�Ϊ��λ)
void mymemset(void *s,u8 c,u32 count)  
{  
    u8 *xs = s;  
    while(count--)*xs++=c;  
}	   
//�ڴ�����ʼ��  
//0:�����ڴ��
void my_mem_init(void)  
{  
    mymemset(mallco_dev.memmap[0], 0,memtblsize[0]*2);//�ڴ�״̬����������  
	mymemset(mallco_dev.membase[0], 0,memsize[0]);	//�ڴ��������������  
	mallco_dev.memrdy[0]=1;								//�ڴ�����ʼ��OK  
}  
////��ȡ�ڴ�ʹ����
////0:�����ڴ��
////����ֵ:ʹ����(0~100)
u8 my_mem_perused(void)  
{  
    u32 used=0;  
    u32 i;  
    for(i=0;i<memtblsize[0];i++)  
    {  
        if(mallco_dev.memmap[0][i])used++; 
    } 
    return (used*100)/(memtblsize[0]);  
}  
////�ڴ����(�ڲ�����)
////0:�����ڴ��
////size:Ҫ������ڴ��С(�ֽ�)
////����ֵ:0XFFFFFFFF,�������;����,�ڴ�ƫ�Ƶ�ַ 
u32 my_mem_malloc(u32 size)  
{  
    signed long offset=0;  
    u32 nmemb;	//��Ҫ���ڴ����  
	u32 cmemb=0;//�������ڴ����
    u32 i;  
    if(!mallco_dev.memrdy[0])mallco_dev.init();//δ��ʼ��,��ִ�г�ʼ�� 
    if(size==0)return 0XFFFFFFFF;//����Ҫ����
    nmemb=size/memblksize[0];  	//��ȡ��Ҫ����������ڴ����
    if(size%memblksize[0])nmemb++;  
    for(offset=memtblsize[0]-1;offset>=0;offset--)//���������ڴ������  
    {     
		if(!mallco_dev.memmap[0][offset])cmemb++;//�������ڴ��������
		else cmemb=0;								//�����ڴ������
		if(cmemb==nmemb)							//�ҵ�������nmemb�����ڴ��
		{
            for(i=0;i<nmemb;i++)  					//��ע�ڴ��ǿ� 
            {  
                mallco_dev.memmap[0][offset+i]=nmemb;  
            }  
            return (offset*memblksize[0]);//����ƫ�Ƶ�ַ  
		}
    }  
    return 0XFFFFFFFF;//δ�ҵ����Ϸ����������ڴ��  
}  
//�ͷ��ڴ�(�ڲ�����) 
//0:�����ڴ��
//offset:�ڴ��ַƫ��
//����ֵ:0,�ͷųɹ�;1,�ͷ�ʧ��;  
u8 my_mem_free(u32 offset)  
{  
    int i;  
    if(!mallco_dev.memrdy[0])//δ��ʼ��,��ִ�г�ʼ��
	{
		mallco_dev.init();    
        return 1;//δ��ʼ��  
    }  
    if(offset<memsize[0])//ƫ�����ڴ����. 
    {  
        int index=offset/memblksize[0];			//ƫ�������ڴ�����  
        int nmemb=mallco_dev.memmap[0][index];	//�ڴ������
        for(i=0;i<nmemb;i++)  						//�ڴ������
        {  
            mallco_dev.memmap[0][index+i]=0;  
        }  
        return 0;  
    }else return 2;//ƫ�Ƴ�����.  
}  
////�ͷ��ڴ�(�ⲿ����) 
////0:�����ڴ��
////ptr:�ڴ��׵�ַ 
//void myfree(void *ptr)  
//{  
//	u32 offset;   
//	if(ptr==NULL)return;//��ַΪ0.  
// 	offset=(u32)ptr-(u32)mallco_dev.membase[0];     
//    my_mem_free(offset);	//�ͷ��ڴ�      
//}  
//�����ڴ�(�ⲿ����)
//0:�����ڴ��
//size:�ڴ��С(�ֽ�)
//����ֵ:���䵽���ڴ��׵�ַ.
void *mymalloc(u32 size)  
{  
    u32 offset;   
	offset=my_mem_malloc(size);  	   	 	   
    if(offset==0XFFFFFFFF)return NULL;  
    else return (void*)((u32)mallco_dev.membase[0]+offset);  
}  
//���·����ڴ�(�ⲿ����)
//0:�����ڴ��
//*ptr:���ڴ��׵�ַ
//size:Ҫ������ڴ��С(�ֽ�)
//����ֵ:�·��䵽���ڴ��׵�ַ.
//void *myrealloc(void *ptr,u32 size)  
//{  
//    u32 offset;    
//    offset=my_mem_malloc(size);   	
//    if(offset==0XFFFFFFFF)return NULL;     
//    else  
//    {  									   
//	    mymemcpy((void*)((u32)mallco_dev.membase[0]+offset),ptr,size);	//�������ڴ����ݵ����ڴ�   
//        myfree(ptr);  											  		//�ͷž��ڴ�
//        return (void*)((u32)mallco_dev.membase[0]+offset);  				//�������ڴ��׵�ַ
//    }  
//}












