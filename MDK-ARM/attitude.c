#include "attitude.h"
#include "MPU6050.h"
#include <math.h>
#include "MPU6050_Reg.h"

#define ATT_PI   3.14159265f
#define ATT_D2R  (ATT_PI / 180.0f)		/* 角度 → 弧度 */
#define ATT_R2D  (180.0f / ATT_PI)		/* 弧度 → 角度 */

/* 陀螺仪零偏，由 Attitude_CalibrateGyro() 测出，单位：°/s */
static float s_GyroOffset[3] = {0.0f, 0.0f, 0.0f};

/* 最近一次的角速度（已扣零偏），供静止检测使用，单位：°/s */
static float s_GyroRate[3] = {0.0f, 0.0f, 0.0f};

/* 连续静止累计时间，单位：ms。超过 ATT_STILL_DELAY_MS 后冻结加速度计修正 */
static uint32_t s_StillTime = 0;

/* 加速度计零偏参考值（单位：g），由 Attitude_LevelZero() 测得。
   之后所有读数都减去它，消除加速度计零偏/温漂/安装倾斜造成的静置漂移。
   只在 LevelZero 时更新，运行中不再变化。 */
static float s_AccRefX = 0.0f;
static float s_AccRefY = 0.0f;

/* MPU6050 连续读出的一帧数据：AX AY AZ <温度2字节> GX GY GZ */
#define ATT_BUF_SIZE  14

/**
  * @brief  角度归一化到 -180 ~ +180
  */
static float Attitude_Wrap180(float angle)
{
	while (angle > 180.0f)
	{
		angle -= 360.0f;
	}
	while (angle < -180.0f)
	{
		angle += 360.0f;
	}
	return angle;
}

/**
  * @brief  限幅
  */
static float Attitude_Limit(float value, float limit)
{
	if (value > limit)
	{
		return limit;
	}
	if (value < -limit)
	{
		return -limit;
	}
	return value;
}

/**
  * @brief  cos() 的小角度近似（角度制输入）
  * @note   cos(x) ≈ 1 - x²/2 + x⁴/24，x 超出 ±60° 时直接截断为 0.5，避免近似失效
  */
static float Attitude_CosSmall(float degree)
{
	float x = degree * ATT_D2R;
	float x2 = x * x;
	float c = 1.0f - x2 / 2.0f + x2 * x2 / 24.0f;
	/*上下都要截断：roll 超过 120° 时泰勒展开会变负，负的余弦会让修正量反向，
	  变成正反馈，所以下限钳在 0.5；上限 1.0 防止 x 极小时高次项微超 1*/
	if (c < 0.5f)
	{
		c = 0.5f;
	}
	if (c > 1.0f)
	{
		c = 1.0f;
	}
	return c;
}

void Attitude_Init(Attitude_t *att)
{
	att->roll = 0.0f;
	att->pitch = 0.0f;
	att->yaw = 0.0f;

	/*参考值一并清零，等 Attitude_LevelZero() 再填入实测值*/
	s_AccRefX = 0.0f;
	s_AccRefY = 0.0f;
	s_StillTime = 0;
}

void Attitude_CalibrateGyro(uint16_t samples)
{
	uint8_t buf[ATT_BUF_SIZE];
	int32_t sum[3] = {0, 0, 0};
	int16_t gx, gy, gz;
	uint16_t i;
	uint16_t valid = 0;		/* 有效采样次数 */

	if (samples == 0)
	{
		return;
	}

	for (i = 0; i < samples; i++)
	{
		/*起始地址 0x3B，连续读 14 字节 = 加速度3轴 + 温度 + 陀螺仪3轴*/
		if (MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, buf, ATT_BUF_SIZE) == 0)
		{
			continue;		/*读失败的数据不能计入零偏，否则会把零偏算反*/
		}

		gx = (int16_t)((buf[8]  << 8) | buf[9]);
		gy = (int16_t)((buf[10] << 8) | buf[11]);
		gz = (int16_t)((buf[12] << 8) | buf[13]);

		sum[0] += gx;
		sum[1] += gy;
		sum[2] += gz;
		valid++;

		HAL_Delay(1);		/*采样间隔，保证取到的是不同时刻的数据*/
	}

	if (valid == 0)
	{
		return;				/*一次都没读到，保持上次的零偏*/
	}

	/*除以有效采样次数，而不是请求次数，否则有读失败时会拉低零偏*/
	s_GyroOffset[0] = (float)sum[0] / (float)valid / ATT_GYRO_LSB_PER_DPS;
	s_GyroOffset[1] = (float)sum[1] / (float)valid / ATT_GYRO_LSB_PER_DPS;
	s_GyroOffset[2] = (float)sum[2] / (float)valid / ATT_GYRO_LSB_PER_DPS;
}

void Attitude_GetGyroOffset(float *pOffsetX, float *pOffsetY, float *pOffsetZ)
{
	if (pOffsetX != 0)
	{
		*pOffsetX = s_GyroOffset[0];
	}
	if (pOffsetY != 0)
	{
		*pOffsetY = s_GyroOffset[1];
	}
	if (pOffsetZ != 0)
	{
		*pOffsetZ = s_GyroOffset[2];
	}
}

void Attitude_LevelZero(Attitude_t *att, uint16_t samples)
{
	uint8_t buf[ATT_BUF_SIZE];
	int32_t sumAx = 0, sumAy = 0;
	uint16_t i, valid = 0;
	float axf, ayf;

	if (samples == 0)
	{
		return;
	}

	for (i = 0; i < samples; i++)
	{
		if (MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, buf, ATT_BUF_SIZE) == 0)
		{
			continue;
		}

		sumAx += (int16_t)((buf[0] << 8) | buf[1]);
		sumAy += (int16_t)((buf[2] << 8) | buf[3]);
		valid++;

		HAL_Delay(1);
	}

	if (valid == 0)
	{
		return;
	}

	/*注意：这里存的是"未乘 ATT_SIGN_xxx"的原始值，
	   与 Attitude_Update() 里"原始值 - 参考值，再乘符号"的顺序保持一致，
	   否则符号会重复作用一次，导致 2 倍偏差。*/
	axf = (float)sumAx / (float)valid / ATT_ACC_LSB_PER_G;
	ayf = (float)sumAy / (float)valid / ATT_ACC_LSB_PER_G;

	/*===== 关键：记录加速度计的零偏参考 =====
	 * 水平放置时理论上 ax=ay=0，实际因为零偏/温漂/安装倾斜会读到几百 mg。
	 * 如果直接用原始读数，滤波器会一直朝这个"错误的水平"收敛，
	 * 就表现为静置时 roll/pitch 缓慢递减。
	 * 把此刻的读数存成参考值，之后所有读数都减掉它，误差自然归零。*/
	s_AccRefX = axf;
	s_AccRefY = ayf;

	/*上电这一刻就是 0° 基准，所以角度直接归零，不需要再反解*/
	att->roll  = 0.0f;
	att->pitch = 0.0f;
	att->yaw   = 0.0f;

	s_StillTime = 0;
}

void Attitude_GetGyroRate(float *pGx, float *pGy, float *pGz)
{
	if (pGx != 0) *pGx = s_GyroRate[0];
	if (pGy != 0) *pGy = s_GyroRate[1];
	if (pGz != 0) *pGz = s_GyroRate[2];
}

uint8_t Attitude_Update(Attitude_t *att, float dt)
{
	uint8_t buf[ATT_BUF_SIZE];
	int16_t ax, ay, az, gx, gy, gz;
	float axf, ayf, azf;			/* 加速度，单位：g */
	float gxf, gyf, gzf;			/* 陀螺仪，单位：°/s */
	float cosR, cosP;				/* 当前姿态角的余弦近似值 */
	float corrRoll, corrPitch;		/* 加速度计给出的修正量，单位：°/s */
	float errRoll, errPitch;		/* 加速度计算出的倾角误差，单位：度 */
	float accGate;					/* 加速度计修正开关：1=开启，0=冻结 */

	if (dt <= 0.0f)
	{
		return 0;
	}
	/*一次读出 6 轴数据，比逐字节读快很多*/
	if (MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, buf, ATT_BUF_SIZE) == 0)
	{
		return 1;		/*I2C 读取失败*/
	}

	ax = (int16_t)((buf[0] << 8) | buf[1]);
	ay = (int16_t)((buf[2] << 8) | buf[3]);
	az = (int16_t)((buf[4] << 8) | buf[5]);
	gx = (int16_t)((buf[8]  << 8) | buf[9]);
	gy = (int16_t)((buf[10] << 8) | buf[11]);
	gz = (int16_t)((buf[12] << 8) | buf[13]);

	/*先做零偏扣除（都在"原始坐标系"里做），再乘轴符号，
	   保证与 Attitude_LevelZero() 存参考值时的顺序一致*/
	axf = ((float)ax / ATT_ACC_LSB_PER_G - s_AccRefX) * ATT_SIGN_PITCH_ACC;
	ayf = ((float)ay / ATT_ACC_LSB_PER_G - s_AccRefY) * ATT_SIGN_ROLL_ACC;
	azf = (float)az / ATT_ACC_LSB_PER_G;

	gxf = (float)gx / ATT_GYRO_LSB_PER_DPS - s_GyroOffset[0];
	gyf = (float)gy / ATT_GYRO_LSB_PER_DPS - s_GyroOffset[1];
	gzf = (float)gz / ATT_GYRO_LSB_PER_DPS - s_GyroOffset[2];

	gxf *= ATT_SIGN_ROLL_GYRO;
	gyf *= ATT_SIGN_PITCH_GYRO;

	/*静止去零死区：陀螺仪残差很小时直接清零，避免积分缓慢累积*/
	if (gxf > -ATT_GYRO_STILL_DZ && gxf < ATT_GYRO_STILL_DZ) gxf = 0.0f;
	if (gyf > -ATT_GYRO_STILL_DZ && gyf < ATT_GYRO_STILL_DZ) gyf = 0.0f;
	if (gzf > -ATT_GYRO_STILL_DZ && gzf < ATT_GYRO_STILL_DZ) gzf = 0.0f;

	/*记录本帧角速度，供静止检测使用*/
	s_GyroRate[0] = gxf;
	s_GyroRate[1] = gyf;
	s_GyroRate[2] = gzf;

	/*===== 静止检测 =====
	 * 三轴角速度都很小 → 视为静止。持续静止超过 ATT_STILL_DELAY_MS 后，
	 * 冻结加速度计修正（gate=0）。
	 * 目的：静止时加速度计的任何直流偏差（零偏、温漂、安装倾斜）
	 *       都会把角度慢慢"拉"向它，表现为静置时角度缓慢递减。
	 *       冻结之后角度只由陀螺仪保持，而陀螺仪零偏已校准、又有死区，
	 *       所以静置时角度就稳住了。一有动作 gate 立刻恢复为 1。*/
	if (gxf > -ATT_STILL_THRESHOLD && gxf < ATT_STILL_THRESHOLD &&
		gyf > -ATT_STILL_THRESHOLD && gyf < ATT_STILL_THRESHOLD &&
		gzf > -ATT_STILL_THRESHOLD && gzf < ATT_STILL_THRESHOLD)
	{
		if (s_StillTime < 0xFFFFFFFFUL)
		{
			s_StillTime += (uint32_t)(dt * 1000.0f);
		}
	}
	else
	{
		s_StillTime = 0;
	}

	accGate = (s_StillTime >= ATT_STILL_DELAY_MS) ? 0.0f : 1.0f;

	/*===== 加速度计修正量（注意单位必须是 °/s）=====
	 * 加速度计给出的倾角误差（小角度近似）：
	 *   errRoll  ≈ -ayf / cosR        （°，因为 ayf 以 g 为单位，直接就是弧度→角度）
	 *   errPitch ≈ -axf · cosR / cosP
	 *
	 * 把误差乘上增益 K 作为"角速度修正量"（°/s），再和陀螺仪角速度相加后积分，
	 * 得到一阶闭环：  d(θ)/dt = ω + K·(θ_acc - θ)
	 * 时间常数 τ ≈ 1/K。K=2.0 → τ≈0.5s。
	 *
	 * 【历史坑】早先的写法是 corr ∝ (1-ATT_ALPHA)·θ_acc·57.3，
	 * 量纲上其实是"角度"却被当成"角速度"用，等效增益比设计值小了约 57 倍，
	 * 表现为加速度计几乎不起作用、角度漂移很慢却停不下来（本次问题的成因之一）。
	 *
	 * 符号：由旋转矩阵 R_x(φ)·g=(0,-sinφ,cosφ)、R_y(θ)·g=(-sinθ,0,cosθ) 推出
	 *   roll 修正 ∝ -ay，pitch 修正 ∝ -ax
	 * 不同模块的轴丝印方向可能相反，所以再用 ATT_SIGN_xxx 宏兜底（见 attitude.h）。*/
	cosR = Attitude_CosSmall(att->roll);
	cosP = Attitude_CosSmall(att->pitch);

	/*重力模长归一化：|a| 静止时≈1g，但手持加速时会偏离 1g。
	   不归一化的话，晃动时的线性加速度会被当成倾角误差放大。
	   sqrtf 每帧只调用一次，100Hz 下开销可以接受。*/
	{
		float mag = sqrtf(axf * axf + ayf * ayf + azf * azf);
		if (mag < 0.5f)
		{
			mag = 0.5f;		/*防止失重或除零*/
		}

		errRoll  = -ayf / mag / cosR;
		errPitch = -axf / mag * cosR / cosP;
	}

	/*死区：误差小于 ATT_ACC_DEADBAND 度时不修正。
	   加速度计几个度的直流偏差（零偏/温漂/安装倾斜）就不会把角度慢慢拉走，
	   而倾斜几十度时修正照常工作。单位和 errX 一致，都是度。*/
	if (errRoll  > -ATT_ACC_DEADBAND && errRoll  < ATT_ACC_DEADBAND) errRoll  = 0.0f;
	if (errPitch > -ATT_ACC_DEADBAND && errPitch < ATT_ACC_DEADBAND) errPitch = 0.0f;

	/*误差 → 角速度修正量（°/s）*/
	corrRoll  = errRoll  * ATT_CORR_GAIN;
	corrPitch = errPitch * ATT_CORR_GAIN;

	/*限幅，防止磕碰、晃动等加速度冲击把姿态带偏*/
	corrRoll  = Attitude_Limit(corrRoll,  ATT_ACC_CORR_LIMIT);
	corrPitch = Attitude_Limit(corrPitch, ATT_ACC_CORR_LIMIT);

	/*===== 互补滤波 =====
	 * 角度 = 上一次角度 + (陀螺仪角速度 + 加速度计修正 × gate) × dt
	 * 两者都是 °/s，量纲一致；gate=0 时等于纯陀螺仪保持*/
	att->roll  += (gxf + corrRoll  * accGate) * dt;
	att->pitch += (gyf + corrPitch * accGate) * dt;
	att->yaw   += gzf * dt;

	/*角度归一化：Pitch 限制在 ±90°，Roll/Yaw 限制在 ±180°*/
	if (att->pitch > 90.0f)
	{
		att->pitch = 90.0f;
	}
	if (att->pitch < -90.0f)
	{
		att->pitch = -90.0f;
	}

	att->roll = Attitude_Wrap180(att->roll);
	att->yaw  = Attitude_Wrap180(att->yaw);

	return 0;
}
