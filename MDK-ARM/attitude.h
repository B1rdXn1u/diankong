#ifndef __ATTITUDE_H
#define __ATTITUDE_H

/* ========================= 姿态解算模块（MPU6050） =========================
 * 算法：互补滤波（加速度计 + 陀螺仪融合）
 *   - Roll（横滚角）、Pitch（俯仰角）：由加速度计的重力分量修正陀螺仪积分，长期不漂移
 *   - Yaw（偏航角）  ：只能靠陀螺仪 Z 轴积分，开机时自动置零
 *                       （MPU6050 没有磁力计，Yaw 无绝对参考，长时间会缓慢漂移，属正常现象）
 *
 * 姿态角基于 MPU6050 模块自身的坐标系（右手系，Z 轴垂直模块板面）：
 *   - Roll / Pitch 以"调用 Attitude_LevelZero() 时的姿态"为 0°，
 *     所以 CalibrateGyro + LevelZero 之后，当前摆法就是 0° 基准
 *   - Yaw 以"上电瞬间的朝向"为零点，因为加速度计无法感知绕重力轴的旋转
 *
 * 静止时角度不漂移的两道保险：
 *   1) 加速度计死区：倾角偏差小于 ATT_ACC_DEADBAND 时不修正，
 *      抵消加速度计零偏/温漂/安装倾斜带来的持续拉扯
 *   2) 静止锁零   ：三轴角速度持续很小超过 ATT_STILL_DELAY_MS 后，
 *      完全冻结加速度计修正，只靠陀螺仪保持（零偏已校准，故不漂）
 *
 * 性能：运行时只用乘除法和小角度近似，不调用 sinf/cosf/atan2f/asinf，
 *       避免 Cortex-M3 软件浮点库的额外开销（atan2f 只在初始化时用两次）。
 * ======================================================================== */

#include <stdint.h>

/* MPU6050 传感器的原始量程配置，改量程时这里必须同步修改 */
#define ATT_ACC_LSB_PER_G        2048.0f   /* ACCEL_CONFIG = 0x18，±16g  → 2048 LSB/g   */
#define ATT_GYRO_LSB_PER_DPS     16.4f     /* GYRO_CONFIG  = 0x18，±2000°/s → 16.4 LSB/(°/s) */

/* ---- 轴向符号（如果角度方向反了，只需改这里的 -1 / +1） ----
 * 不同 MPU6050 模块的坐标轴丝印方向可能相反，表现为：
 *   "把板子往某个方向倾，显示的数却是反的" 或 "静止时角度往一边跑"。
 * 改这里比改公式安全得多。 */
#define ATT_SIGN_ROLL_ACC        (1.0f)
#define ATT_SIGN_PITCH_ACC       (1.0f)
#define ATT_SIGN_ROLL_GYRO       (1.0f)
#define ATT_SIGN_PITCH_GYRO      (1.0f)

/* 互补滤波参数 */
#define ATT_CORR_GAIN            2.0f      /* 加速度计修正增益，单位 1/s；时间常数 ≈ 1/该值，2.0 → 约 0.5 秒 */
#define ATT_ACC_CORR_LIMIT       10.0f     /* 单次加速度计修正量上限（°/s），防止加速度冲击带偏姿态 */
#define ATT_GYRO_STILL_DZ        0.3f      /* 陀螺仪静止去零死区（°/s），抑制静止时的角度抖动 */

/* ---- 静止锁零 ---- */
#define ATT_STILL_THRESHOLD      1.0f      /* 三轴角速度都小于此值（°/s）判为静止 */
#define ATT_STILL_DELAY_MS       400U      /* 持续静止这么久后，冻结加速度计修正 */
#define ATT_ACC_DEADBAND         2.0f      /* 加速度计修正死区（°），小于此偏差不修正 */

/* 姿态角结果，单位：度 */
typedef struct
{
	float roll;    /* 横滚角，范围 -180 ~ +180 */
	float pitch;   /* 俯仰角，范围 -90  ~ +90  */
	float yaw;     /* 偏航角，范围 -180 ~ +180 */
} Attitude_t;

/**
  * @brief  姿态解算初始化，把所有角度清零
  * @param  att 姿态结构体
  * @retval 无
  */
void Attitude_Init(Attitude_t *att);

/**
  * @brief  陀螺仪零偏校准（必须在模块静止时调用）
  * @param  samples 采样次数，例如 1000
  * @retval 无
  * @note   静止采集 samples 次取平均值作为零偏，之后在解算中扣除。
  *         不做校准的话，陀螺仪零偏会让角度持续缓慢漂移。
  */
void Attitude_CalibrateGyro(uint16_t samples);

/**
  * @brief  读取陀螺仪零偏校准结果，用于诊断 Yaw 漂移
  * @param  pOffsetX / pOffsetY / pOffsetZ 输出三个轴的零偏，单位：°/s
  * @retval 无
  * @note   静止时若某个轴的值明显大于 ATT_GYRO_STILL_DZ(0.3°/s)，
  *         说明零偏较大，Yaw 会可见地匀速漂移；
  *         若三个值全部为 0，说明校准没执行或 I2C 读取全部失败。
  */
void Attitude_GetGyroOffset(float *pOffsetX, float *pOffsetY, float *pOffsetZ);

/**
  * @brief  用加速度计把当前姿态设为"水平零点"（必须在模块静止、水平放置时调用）
  * @param  att 姿态结构体
  * @param  samples 采样次数
  * @retval 无
  * @note   这一步消除的是两件事：
  *         1) 模块安装时的轻微倾斜 —— 把"现在的姿态"定义成 0°
  *         2) 加速度计的零偏 —— 它会让滤波器的平衡点偏离真实水平
  *         如果不做，角度会从上电的 0° 慢慢"爬"到加速度计认为的水平位置，
  *         表现就是静置时角度持续递减。这是本次漂移问题的主要成因。
  */
void Attitude_LevelZero(Attitude_t *att, uint16_t samples);

/**
  * @brief  读取最近一次的角速度，用于判断是否静止
  * @param  pGx / pGy / pGz 输出三轴角速度，单位：°/s（已扣零偏）
  * @retval 无
  */
void Attitude_GetGyroRate(float *pGx, float *pGy, float *pGz);

/**
  * @brief  姿态解算更新，需要周期性调用
  * @param  att 姿态结构体
  * @param  dt  距离上一次调用经过的时间，单位：秒
  * @retval 0：成功；1：读取 MPU6050 失败（此时 att 保持上一次的值）
  */
uint8_t Attitude_Update(Attitude_t *att, float dt);

#endif
