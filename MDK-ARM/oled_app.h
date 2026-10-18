#ifndef __OLED_APP_H
#define __OLED_APP_H

/* ================== OLED 姿态角显示层 ==================
 * 把"角度排版"从 main.c 里拆出来，供 FreeRTOS 的显示任务调用。
 * 只依赖 oled.c 的字符显示函数和 attitude.h 的角度结构体。
 *
 * 内部维护一份"屏上现在是什么"的影子缓存，内容没变的字符不会重复写 I2C
 * （oled.c 是 1 个字符 16 次 I2C 传输的写法，重复写会把总线占满）。
 * 因此：整屏被清过之后必须调用 OLED_AppClear()，否则缓存会和屏幕不一致。
 * ====================================================== */

#include <stdint.h>
#include "attitude.h"

/* 用 'o' 代替"度"符号：字库只有 ASCII 0x20~0x7F，
   直接传 '°'(0xB0) 会越界读到数组外，显示乱码甚至 HardFault */
#define OLED_APP_CHAR_DEG   'o'

/**
  * @brief  清屏，并让显示缓存失效
  * @note   oled.c 的 OLED_Clear() 只管屏幕，不管本模块的缓存，
  *         所以工程里统一用这个函数代替 OLED_Clear()。
  */
void OLED_AppClear(void);

/**
  * @brief  OLED_ShowString 的 const 版本包装
  * @note   oled.c 里形参是 char*，传字符串字面量在严格告警下会报错
  */
void OLED_AppShowText(uint8_t Line, uint8_t Column, const char *Text);

/**
  * @brief  角度转成 0.01 度为单位的整数（四舍五入）
  * @param  Angle 角度，单位：度
  * @retval 角度 × 100
  * @note   先转整数再显示，否则 -0.004° 会显示成 "-0.00"
  */
int32_t OLED_AppAngleToCenti(float Angle);

/**
  * @brief  在 OLED 一行上显示"标签 + 带符号角度 + 度数符号"
  * @param  Line   行号 1~4
  * @param  Label  标签，如 "Roll"
  * @param  NumCol 第一位数字所在列，1~16（符号位在 NumCol-1）
  * @param  Value  角度值，单位：0.01度
  * @retval 无
  * @note   布局（NumCol=7）：Roll   12.34o 共 16 字符
  *         正数符号位用空格占位，小数点列位置恒定，刷新时数字不跳动
  */
void OLED_AppShowAngle(uint8_t Line, const char *Label, uint8_t NumCol, int32_t Value);

/**
  * @brief  画出整屏姿态界面框架（只在初始化后调用一次）
  */
void OLED_AppDrawFrame(void);

/**
  * @brief  刷新三个姿态角
  * @param  att 姿态结构体
  * @param  Err 0=正常，1=I2C 读取异常（第一行末尾显示 ERR）
  */
void OLED_AppUpdateAngles(const Attitude_t *att, uint8_t Err);

#endif
