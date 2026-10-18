#include "oled_app.h"
#include "oled.h"
#include <string.h>
#include <math.h>

/* ------------------------------------------------------------------------
 * 姿态角显示排版
 *   1) OLED 是 8x16 字库，每行 16 个 ASCII 字符（128/8 = 16）
 *   2) 字库只有 ASCII 0x20~0x7F，没有 '°'，用 'o' 代替
 *   3) 定宽字段：整数固定 2 列，小数点列位置恒定，数字不会左右跳动
 *
 * 【为什么要做"显示缓存"】
 *   oled.c 的字符函数是"一个字节一次 I2C 传输"的写法：显示 1 个字符要做
 *   16 次 HAL_I2C_Mem_Write。刷一行 16 个字符就是 256 次传输，
 *   在 400kHz 下要几十毫秒 —— 而 IMU 任务每 10ms 就要用同一条 I2C 总线
 *   读一次 MPU6050。
 *
 *   所以这里给每一行留一份"当前屏上是什么"的影子缓存：只有内容真的变了
 *   才去写 I2C。稳定时角度只变最后几位，一帧通常只写几个字符，
 *   I2C 占用从几十毫秒降到几毫秒，采样任务几乎不受影响。
 *
 *   屏幕被整屏清过（OLED_AppClear）之后，缓存必须一起作废，
 *   否则会以为屏上还有内容而漏写。
 * ---------------------------------------------------------------------- */

/* 每行 16 列的影子缓存；s_lineValid 表示该行缓存是否可信 */
static char    s_lineBuf[5][16];
static uint8_t s_lineValid[5];

/* ------------------------------------------------------------------------
 * 往指定位置写一个字符（内容没变就跳过 I2C）
 * ---------------------------------------------------------------------- */
static void OLED_AppPutChar(uint8_t Line, uint8_t Column, char Char)
{
	if (Line < 1U || Line > 4U || Column < 1U || Column > 16U)
	{
		return;
	}

	if ((s_lineValid[Line] != 0U) && (s_lineBuf[Line][Column - 1U] == Char))
	{
		return;		/*屏上已经是这个字符了，省掉 16 次 I2C 传输*/
	}

	OLED_ShowChar(Line, Column, Char);

	s_lineBuf[Line][Column - 1U] = Char;
	s_lineValid[Line] = 1U;
}

/* ------------------------------------------------------------------------
 * 清屏（同时让显示缓存失效）
 * ---------------------------------------------------------------------- */
void OLED_AppClear(void)
{
	OLED_Clear();

	memset(s_lineValid, 0, sizeof(s_lineValid));
	memset(s_lineBuf, 0, sizeof(s_lineBuf));
}

void OLED_AppShowText(uint8_t Line, uint8_t Column, const char *Text)
{
	uint8_t i;

	if (Text == 0)
	{
		return;
	}

	for (i = 0; Text[i] != '\0'; i++)
	{
		OLED_AppPutChar(Line, (uint8_t)(Column + i), Text[i]);
	}
}

int32_t OLED_AppAngleToCenti(float Angle)
{
	if (Angle > 179.99f)  Angle = 179.99f;
	if (Angle < -179.99f) Angle = -179.99f;

	/*lroundf 对 ±0.5 对称取整，-0.004° 得到 0 而不是 -0，
	  因此不会出现 "-0.00" 这种显示*/
	return (int32_t)lroundf(Angle * 100.0f);
}

void OLED_AppShowAngle(uint8_t Line, const char *Label, uint8_t NumCol, int32_t Value)
{
	char    buf[17];
	uint8_t i;
	int32_t AbsValue;
	int32_t IntPart;
	uint8_t IntDigits;

	if (Label == 0)
	{
		return;
	}

	memset(buf, ' ', 16);
	buf[16] = '\0';

	/*写标签*/
	for (i = 0; (i < 16U) && (Label[i] != '\0'); i++)
	{
		buf[i] = Label[i];
	}

	AbsValue = (Value < 0) ? -Value : Value;
	IntPart  = AbsValue / 100;

	/*整数位数 2~3 位（0 也显示成 "00"，保证宽度一致）。
	  |角度| 到 100° 以上（roll/yaw 最多 180°）时整数变成 3 位。*/
	IntDigits = (IntPart >= 100) ? 3 : 2;

	/*整个字段（符号位 NumCol-1 ~ 度数符号 NumCol+IntDigits+3）必须落在第 1~16 列，
	  否则整行不画，避免越界写*/
	if ((NumCol < 1U) || ((uint32_t)NumCol + (uint32_t)IntDigits + 3U > 15U))
	{
		return;
	}

	if (IntDigits == 3)
	{
		buf[NumCol]     = (char)('0' + (IntPart / 100) % 10);	/*百位*/
		buf[NumCol + 1] = (char)('0' + (IntPart / 10) % 10);	/*十位*/
		buf[NumCol + 2] = (char)('0' + IntPart % 10);			/*个位*/
		buf[NumCol + 3] = '.';									/*小数点*/
		buf[NumCol + 4] = (char)('0' + (AbsValue / 10) % 10);	/*十分位*/
		buf[NumCol + 5] = (char)('0' + AbsValue % 10);			/*百分位*/
		buf[NumCol + 6] = OLED_APP_CHAR_DEG;					/*度数符号*/
	}
	else
	{
		buf[NumCol]     = (char)('0' + (IntPart / 10) % 10);	/*十位*/
		buf[NumCol + 1] = (char)('0' + IntPart % 10);			/*个位*/
		buf[NumCol + 2] = '.';									/*小数点*/
		buf[NumCol + 3] = (char)('0' + (AbsValue / 10) % 10);	/*十分位*/
		buf[NumCol + 4] = (char)('0' + AbsValue % 10);			/*百分位*/
		buf[NumCol + 5] = OLED_APP_CHAR_DEG;					/*度数符号*/
	}

	/*符号位在字段最左边；正数用空格占位，保持字段宽度一致*/
	buf[NumCol - 1] = (Value < 0) ? '-' : ' ';

	/*整行 16 个字符逐个比较，只写变化的那些*/
	for (i = 0; i < 16U; i++)
	{
		OLED_AppPutChar(Line, (uint8_t)(i + 1U), buf[i]);
	}
}

void OLED_AppDrawFrame(void)
{
	OLED_AppClear();
	OLED_AppShowText(1, 1, "IMU Attitude");
	OLED_AppShowText(2, 1, "Roll ");
	OLED_AppShowText(3, 1, "Pitch");
	OLED_AppShowText(4, 1, "Yaw  ");
}

void OLED_AppUpdateAngles(const Attitude_t *att, uint8_t Err)
{
	float pitch;

	if (att == 0)
	{
		return;
	}

	/*通信异常时在第一行末尾提示，正常时清除*/
	OLED_AppShowText(1, 14, Err ? "ERR" : "   ");

	/*Pitch 物理范围 ±90，先限幅，免得显示越界*/
	pitch = att->pitch;
	if (pitch > 90.0f)  pitch = 90.0f;
	if (pitch < -90.0f) pitch = -90.0f;

	OLED_AppShowAngle(2, "Roll ", 7, OLED_AppAngleToCenti(att->roll));
	OLED_AppShowAngle(3, "Pitch", 7, OLED_AppAngleToCenti(pitch));
	OLED_AppShowAngle(4, "Yaw  ", 6, OLED_AppAngleToCenti(att->yaw));
}
