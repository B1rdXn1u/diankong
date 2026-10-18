#ifndef __TIM_H
#define __TIM_H

/* ======================================================================
 * 只为 hal_timebase.c 里的 htim4 提供声明。
 *
 * 正常情况下这个文件应该由 CubeMX 生成（SYS 时基选 TIM4 时会生成 tim.c/tim.h），
 * 但实践中有版本不会生成"虚拟时基"的源文件，所以这里手工补一个最小版本。
 * 如果以后 CubeMX 生成了真实的 tim.h，把本文件删掉即可（内容兼容）。
 * ====================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern TIM_HandleTypeDef htim4;

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H */
