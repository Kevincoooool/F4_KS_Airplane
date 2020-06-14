#include "include.h"
#include "Drv_SPI.h"

/**********************************************************************************************************
*函 数 名: Spi_GPIO_Init
*功能说明: SPI从机设备CS引脚初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Spi_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_InitStructure.GPIO_Pin = GYRO_CS_PIN;
    GPIO_Init(GYRO_CS_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BARO_CS_PIN;
    GPIO_Init(BARO_CS_GPIO, &GPIO_InitStructure);

    GPIO_SetBits(GYRO_CS_GPIO, GYRO_CS_PIN);
    GPIO_SetBits(BARO_CS_GPIO, BARO_CS_PIN);
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;           //GPIO
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
    GPIO_Init(GPIOB,&GPIO_InitStructure);              //初始化GPIo
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;           //GPIO
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
    GPIO_Init(GPIOD,&GPIO_InitStructure);              //初始化GPIo
	
	SPI_CSN_H();
	
	SPI_CE_H();
	
}
/**********************************************************************************************************
*函 数 名: Spi_Open
*功能说明: SPI初始化
*形    参: 设备号
*返 回 值: 无
**********************************************************************************************************/
void Spi_Open(uint8_t deviceNum)
{

    if(deviceNum == 1)
    {

			GPIO_InitTypeDef GPIO_InitStructure;
			SPI_InitTypeDef  SPI_InitStructure;

			
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);//使能SPI1时钟
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//使能PORTF时钟	

	
		    GPIO_InitStructure.GPIO_Pin = SPI1_PIN_MOSI|SPI1_PIN_MISO|SPI1_PIN_SCK;

			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
			GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化

			GPIO_PinAFConfig(SPI1_GPIO_MOSI, SPI1_PINSOURCE_MOSI, GPIO_AF_SPI1);
			GPIO_PinAFConfig(SPI1_GPIO_MISO, SPI1_PINSOURCE_MISO, GPIO_AF_SPI1);
			GPIO_PinAFConfig(SPI1_GPIO_SCK, SPI1_PINSOURCE_SCK, GPIO_AF_SPI1);


			//这里只针对SPI口初始化
			RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);//复位SPI1
			RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);//停止复位SPI1

			SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
			SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
			SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
			
			SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//串行同步时钟的空闲状态为高电平
			SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
			
			SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
			SPI_InitStructure.SPI_BaudRatePrescaler = SPI1_CLOCKDIV;		//定义波特率预分频的值:波特率预分频值为256
			SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
			SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
			SPI_Init(SPI1, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器

			SPI_Cmd(SPI1, ENABLE); //使能SPI外设

			Spi_SingleWirteAndRead(GYRO_SPI,0xff);//启动传输
    }
    else if(deviceNum == 2)
    {
//        GPIO_PinAFConfig(SPI2_GPIO_MOSI, SPI2_PINSOURCE_MOSI, GPIO_AF_SPI2);
//        GPIO_PinAFConfig(SPI2_GPIO_MISO, SPI2_PINSOURCE_MISO, GPIO_AF_SPI2);
//        GPIO_PinAFConfig(SPI2_GPIO_SCK,  SPI2_PINSOURCE_SCK,  GPIO_AF_SPI2);
//        GPIO_InitStructure.GPIO_Pin = SPI2_PIN_MOSI;
//        GPIO_Init(SPI2_GPIO_MOSI, &GPIO_InitStructure);
//        GPIO_InitStructure.GPIO_Pin =  SPI2_PIN_MISO;
//        GPIO_Init(SPI2_GPIO_MISO, &GPIO_InitStructure);
//        GPIO_InitStructure.GPIO_Pin =  SPI2_PIN_SCK;
//        GPIO_Init(SPI2_GPIO_SCK, &GPIO_InitStructure);

//        SPI_InitStructure.SPI_BaudRatePrescaler = SPI2_CLOCKDIV;
//        SPI_Init(SPI2, &SPI_InitStructure);
//        SPI_Cmd(SPI2, ENABLE);
    }
	else if(deviceNum == 3)
    {
		GPIO_InitTypeDef GPIO_InitStructure;
		SPI_InitTypeDef  SPI_InitStructure;


		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);//使能SPI1时钟
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//使能PORTF时钟	

		//GPIOFA5,6,7初始化设置
		GPIO_InitStructure.GPIO_Pin = SPI3_PIN_SCK|SPI3_PIN_MISO|SPI3_PIN_MOSI;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
		GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化

		GPIO_PinAFConfig(SPI3_GPIO_SCK, SPI3_PINSOURCE_SCK, GPIO_AF_SPI3); //PA5复用为 SPI1
		GPIO_PinAFConfig(SPI3_GPIO_MISO,SPI3_PINSOURCE_MISO,GPIO_AF_SPI3); //PA6复用为 SPI1
		GPIO_PinAFConfig(SPI3_GPIO_MOSI,SPI3_PINSOURCE_MOSI,GPIO_AF_SPI3); //PA7复用为 SPI1


		//这里只针对SPI口初始化
		RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,ENABLE);//复位SPI1
		RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,DISABLE);//停止复位SPI1

		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构

		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//串行同步时钟的空闲状态为高电平
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样

		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI3_CLOCKDIV;		//定义波特率预分频的值:波特率预分频值为256
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
		SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
		SPI_Init(SPI3, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器

		SPI_Cmd(SPI3, ENABLE); //使能SPI外设

		Spi_SingleWirteAndRead(NRF2401_SPI,0xff);//启动传输
    }
}


/**********************************************************************************************************
*函 数 名: Spi_SingleWirteAndRead
*功能说明: SPI单字节读取
*形    参: 设备号 写入的数据
*返 回 值: 读取到的数据
**********************************************************************************************************/
uint8_t Spi_SingleWirteAndRead(uint8_t deviceNum, uint8_t dat)
{
    if(deviceNum == 1)
    {
        while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
        SPI_I2S_SendData(SPI1, dat);
        while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
        return SPI_I2S_ReceiveData(SPI1);
    }
    else if(deviceNum == 2)
    {
        while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
        SPI_I2S_SendData(SPI2, dat);
        while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
        return SPI_I2S_ReceiveData(SPI2);
    }
	 else if(deviceNum == 3)
    {
        while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);
        SPI_I2S_SendData(SPI3, dat);
        while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET);
        return SPI_I2S_ReceiveData(SPI3);
    }
    else
    {
        return 0;
    }
}

/**********************************************************************************************************
*函 数 名: SPI_MultiWriteAndRead
*功能说明: SPI多字节读取
*形    参: 设备号 写入数据缓冲区指针 读出数据缓冲区指针 数据长度
            同时只能写入或者读出，写入时读取缓冲区设置为NULL，读出时反之
*返 回 值: 无
**********************************************************************************************************/
void SPI_MultiWriteAndRead(uint8_t deviceNum, uint8_t *out, uint8_t *in, int len)
{
    uint8_t b;
    if(deviceNum == 1)
    {
        while (len--)
        {
            b = in ? *(in++) : 0xFF;
            while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
            SPI_I2S_SendData(SPI1, b);
            while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
            b = SPI_I2S_ReceiveData(SPI1);
            if (out)
                *(out++) = b;
        }
    }
    else if(deviceNum == 2)
    {
        while (len--)
        {
            b = in ? *(in++) : 0xFF;
            while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
            SPI_I2S_SendData(SPI2, b);
            while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
            b = SPI_I2S_ReceiveData(SPI2);
            if (out)
                *(out++) = b;
        }
    }
	else if(deviceNum == 3)
    {
        while (len--)
        {
            b = in ? *(in++) : 0xFF;
            while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);
            SPI_I2S_SendData(SPI3, b);
            while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET);
            b = SPI_I2S_ReceiveData(SPI3);
            if (out)
                *(out++) = b;
        }
    }
}

/**********************************************************************************************************
*函 数 名: Spi_GyroEnable
*功能说明: 陀螺仪CS脚使能
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Spi_GyroEnable(void)
{
    GPIO_ResetBits(GYRO_CS_GPIO, GYRO_CS_PIN);
}

/**********************************************************************************************************
*函 数 名: Spi_GyroDisable
*功能说明: 陀螺仪CS脚失能
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Spi_GyroDisable(void)
{
    GPIO_SetBits(GYRO_CS_GPIO, GYRO_CS_PIN);
}

/**********************************************************************************************************
*函 数 名: Spi_GyroSingleWrite
*功能说明: 陀螺仪单个寄存器写入
*形    参: 寄存器地址 写入值
*返 回 值: 无
**********************************************************************************************************/
void Spi_GyroSingleWrite(uint8_t reg, uint8_t value)
{
    Spi_GyroEnable();
	Spi_BaroDisable();
    Spi_SingleWirteAndRead(GYRO_SPI, reg);
    Spi_SingleWirteAndRead(GYRO_SPI, value);
    Spi_GyroDisable();
}

/**********************************************************************************************************
*函 数 名: Spi_GyroMultiRead
*功能说明: 陀螺仪多个寄存器读出
*形    参: 寄存器地址 读出缓冲区 读出长度
*返 回 值: 无
**********************************************************************************************************/
void Spi_GyroMultiRead(uint8_t reg,uint8_t *data, uint8_t length)
{
    Spi_GyroEnable();
	Spi_BaroDisable();
    Spi_SingleWirteAndRead(GYRO_SPI, reg | 0x80);
    SPI_MultiWriteAndRead(GYRO_SPI, data, 0, length);
    Spi_GyroDisable();
}

/**********************************************************************************************************
*函 数 名: Spi_BaroEnable
*功能说明: 气压计CS脚使能
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Spi_BaroEnable(void)
{
    GPIO_ResetBits(BARO_CS_GPIO, BARO_CS_PIN);
}

/**********************************************************************************************************
*函 数 名: Spi_BaroDisable
*功能说明: 气压计CS脚失能
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Spi_BaroDisable(void)
{
    GPIO_SetBits(BARO_CS_GPIO, BARO_CS_PIN);
}

/**********************************************************************************************************
*函 数 名: Spi_BaroSingleWrite
*功能说明: 气压计单个寄存器写入
*形    参: 寄存器地址 写入值
*返 回 值: 无
**********************************************************************************************************/
void Spi_BaroSingleWrite(uint8_t reg, uint8_t value)
{
    Spi_BaroEnable();
    Spi_SingleWirteAndRead(BARO_SPI, reg);
    Spi_SingleWirteAndRead(BARO_SPI, value);
    Spi_BaroDisable();
}

/**********************************************************************************************************
*函 数 名: Spi_BaroMultiRead
*功能说明: 气压计多个寄存器读出
*形    参: 寄存器地址 读出缓冲区 读出长度
*返 回 值: 无
**********************************************************************************************************/
void Spi_BaroMultiRead(uint8_t reg,uint8_t *data, uint8_t length)
{
    Spi_BaroEnable();
    Spi_SingleWirteAndRead(BARO_SPI, reg);
    SPI_MultiWriteAndRead(BARO_SPI, data, 0, length);
    Spi_BaroDisable();
}


