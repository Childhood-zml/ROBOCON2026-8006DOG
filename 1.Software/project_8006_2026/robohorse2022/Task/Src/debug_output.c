/**
 * @file    debug_output.c
 * @brief   调试输出实现 (仅 DEBUG_ENABLE 时编译)
 */
#ifdef DEBUG_ENABLE

#include "debug_output.h"

/* UART7 已移除，调试输出当前无底层实现 */
void debug_printf(const char *fmt, ...) { (void)fmt; }
void debug_print_val(const char *name, float val) { (void)name; (void)val; }
void debug_print_int(const char *name, int32_t val) { (void)name; (void)val; }

#endif
