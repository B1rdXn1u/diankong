#include "MPU6050.h"
#include "MPU6050_Reg.h"

/* ============================ I2C 句柄配置 ============================
 * MPU6050 与 OLED 共用同一条硬件 I2C 总线（I2C1：PB6=SCL，PB7=SDA），
 * 两者从机地址不同（OLED：0x78，MPU6050：0xD0），可以挂在同一总线上。
 * 串口/OLED 已使用 hi2c1，此处直接复用。
 * 若以后改用 I2C2（PB10/PB11），把下面的端口号改成 2 即可
 * ==================================================================== */
#define MPU6050_I2C_PORT  1

#define MPU6050_I2C_HANDLE_JOIN_(port) hi2c##port
#define MPU6050_I2C_HANDLE_JOIN(port)  MPU6050_I2C_HANDLE_JOIN_(port)
#define MPU6050_I2C_HANDLE             MPU6050_I2C_HANDLE_JOIN(MPU6050_I2C_PORT)

extern I2C_HandleTypeDef MPU6050_I2C_HANDLE;

#define MPU6050_ADDRESS			0xD0		//MPU6050的I2C从机地址（8位形式，HAL要求左对齐）
#define MPU6050_I2C_TIMEOUT		0x100U		//HAL库I2C读写超时时间，单位：毫秒



/**
  * @brief  MPU6050写寄存器
  * @param  RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
  * @param  Data 要写入寄存器的数据，范围：0x00~0xFF
  * @retval 无
  */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	/*HAL库函数一次性完成：起始条件 -> 发送从机地址 -> 发送寄存器地址 -> 发送数据 -> 终止条件*/
	HAL_I2C_Mem_Write(&MPU6050_I2C_HANDLE, MPU6050_ADDRESS, RegAddress,
		I2C_MEMADD_SIZE_8BIT, &Data, 1, MPU6050_I2C_TIMEOUT);
}

/**
  * @brief  MPU6050读寄存器
  * @param  RegAddress 寄存器地址，范围：参考MPU6050手册的寄存器描述
  * @retval 读取寄存器的数据，范围：0x00~0xFF
  */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data = 0;
	
	/*HAL库函数一次性完成：起始条件 -> 写寄存器地址 -> 重复起始条件 -> 读数据 -> 终止条件*/
	HAL_I2C_Mem_Read(&MPU6050_I2C_HANDLE, MPU6050_ADDRESS, RegAddress,
		I2C_MEMADD_SIZE_8BIT, &Data, 1, MPU6050_I2C_TIMEOUT);
	
	return Data;
}

/**
  * @brief  MPU6050检测器件是否应答
  * @param  无
  * @retval 1：器件在线；0：器件无应答
  */
uint8_t MPU6050_IsReady(void)
{
	return (HAL_I2C_IsDeviceReady(&MPU6050_I2C_HANDLE, MPU6050_ADDRESS, 3,
		MPU6050_I2C_TIMEOUT) == HAL_OK) ? 1 : 0;
}

/**
  * @brief  MPU6050初始化
  * @param  无
  * @retval 无
  */
void MPU6050_Init(void)
{
	/*I2C的时钟、GPIO（PB6/PB7 复用开漏）以及I2C外设的使能，
	  已经在CubeMX生成的 MX_I2C1_Init() 中完成，此处无需再配置*/
	
	/*MPU6050寄存器初始化，需要对照MPU6050手册的寄存器描述配置，此处仅配置了部分重要的寄存器*/
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);				//电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);				//电源管理寄存器2，保持默认值0，所有轴均不待机
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);				//采样率分频寄存器，配置采样率
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);					//配置寄存器，配置DLPF
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);			//陀螺仪配置寄存器，选择满量程为±2000°/s
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);			//加速度计配置寄存器，选择满量程为±16g
}

/**
  * @brief  MPU6050获取ID号
  * @param  无
  * @retval MPU6050的ID号
  */
uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);		//返回WHO_AM_I寄存器的值
}

/**
  * @brief  MPU6050获取数据
  * @param  AccX AccY AccZ 加速度计X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
  * @param  GyroX GyroY GyroZ 陀螺仪X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
  * @retval 无
  */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t DataH, DataL;								//定义数据高8位和低8位的变量
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);		//读取加速度计X轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);		//读取加速度计X轴的低8位数据
	*AccX = (int16_t)((DataH << 8) | DataL);			//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);		//读取加速度计Y轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);		//读取加速度计Y轴的低8位数据
	*AccY = (int16_t)((DataH << 8) | DataL);			//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);		//读取加速度计Z轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);		//读取加速度计Z轴的低8位数据
	*AccZ = (int16_t)((DataH << 8) | DataL);			//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);		//读取陀螺仪X轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);		//读取陀螺仪X轴的低8位数据
	*GyroX = (int16_t)((DataH << 8) | DataL);			//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);		//读取陀螺仪Y轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);		//读取陀螺仪Y轴的低8位数据
	*GyroY = (int16_t)((DataH << 8) | DataL);			//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);		//读取陀螺仪Z轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);		//读取陀螺仪Z轴的低8位数据
	*GyroZ = (int16_t)((DataH << 8) | DataL);			//数据拼接，通过输出参数返回
}

/**
  * @brief  MPU6050读取连续多个寄存器（用于一次性读取6轴数据，效率更高）
  * @param  RegAddress 起始寄存器地址
  * @param  pData 读出的数据缓冲区
  * @param  Len 要读取的字节数
  * @retval 1：读取成功；0：读取失败（器件无应答或总线错误）
  */
uint8_t MPU6050_ReadRegs(uint8_t RegAddress, uint8_t *pData, uint16_t Len)
{
	if (HAL_I2C_Mem_Read(&MPU6050_I2C_HANDLE, MPU6050_ADDRESS, RegAddress,
		I2C_MEMADD_SIZE_8BIT, pData, Len, MPU6050_I2C_TIMEOUT) == HAL_OK)
	{
		return 1;
	}
	return 0;
}
