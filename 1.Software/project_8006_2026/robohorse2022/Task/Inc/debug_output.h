/**
 * @file    debug_output.h
 * @brief   调试输出 — 编译时可开关的运行时诊断
 * @note    编译时加 -DDEBUG_ENABLE 开启, 默认关闭
 *          使用 UART7 输出 (与 PID 调参共用)
 */
#ifndef DEBUG_OUTPUT_H
#define DEBUG_OUTPUT_H

#ifdef DEBUG_ENABLE

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 调试 printf (UART7 输出)
 */
void debug_printf(const char *fmt, ...);

/**
 * @brief 打印变量名和值
 */
void debug_print_val(const char *name, float val);

/**
 * @brief 打印整型变量
 */
void debug_print_int(const char *name, int32_t val);

/* 便捷宏 */
#define DEBUG_LOG(fmt, ...)       debug_printf(fmt, ##__VA_ARGS__)
#define DEBUG_VAL(name, val)      debug_print_val(name, (float)(val))
#define DEBUG_INT(name, val)      debug_print_int(name, (int32_t)(val))

#ifdef __cplusplus
}
#endif

#else  /* !DEBUG_ENABLE — 编译为无操作 */

#define DEBUG_LOG(fmt, ...)       ((void)0)
#define DEBUG_VAL(name, val)      ((void)0)
#define DEBUG_INT(name, val)      ((void)0)

#endif /* DEBUG_ENABLE */

#endif /* DEBUG_OUTPUT_H */
