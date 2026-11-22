# RoboHorse2022 代码重构变更文档

> 生成日期：2026-07-09
> 重构范围：STM32F427 四足仿生机器马项目全代码

---

## 变更总览

| 指标 | 数值 |
|------|------|
| 修改文件 | 18 个 |
| 新建文件 | 2 个 |
| 删除文件 | 1 个 |
| 删除死代码 | ~350 行 |
| 消除重复代码 | ~200 行压缩为 ~40 行 |
| Bug 修复 | 2 个 |
| 编码修复 | 11 个文件 |

---

## 一、修复的 Bug

### 1.1 整数除法 Bug（严重）

**文件**：[Task/Src/motor_task.c](Task/Src/motor_task.c) 第 162 行

**问题**：
```c
// 修复前：整数除法，360 / 8191 = 0，PID 反馈始终为 0
PID_Calc(&PID_Position[i], J8009[i].Total_Angle * 360 / 8191, ...);

// 修复后：浮点除法
PID_Calc(&PID_Position[i], J8009[i].Total_Angle * 360.0f / 8191.0f, ...);
```

**影响**：PID 调参模式（`PID_CONFIG != 0`）下位置环永远收到 0 作为反馈值，电机无法正常控制。

### 1.2 截断的宏定义

**文件**：[BSP/Inc/serial.h](BSP/Inc/serial.h) 第 50 行

**问题**：
```c
// 修复前：缺少闭合括号，数值错误
#define RC_CH_VALUE_MAX ((uint16_t)16 // 手柄最小值

// 修复后：
#define RC_CH_VALUE_MAX ((uint16_t)1684) // 手柄最大值
```

---

## 二、新建文件

### 2.1 [BSP/Inc/motor_config.h](BSP/Inc/motor_config.h)

电机硬件配置头文件，集中定义：

- `MOTOR_COUNT`、`MOTOR_POS_LIMIT_MIN/MAX` 等全局常量
- `MotorConfig_t` 结构体（电机ID、CAN总线、腿编号、位置限制）
- `clamp_float()`、`motor_id_to_hcan()` 内联工具函数
- `CAN_TX_TIMEOUT_MS`、`MOTOR_TEMP_HIGH/RECOVERY` 等可调参数

### 2.2 [BSP/Src/motor_config.c](BSP/Src/motor_config.c)

8 个电机的 CAN 总线映射表实现：

```
电机ID  0     1     2     3     4     5     6     7
CAN    CAN1  CAN1  CAN2  CAN2  CAN2  CAN2  CAN1  CAN1
腿     右前  右前  左前  左前  左后  左后  右后  右后
关节   外    内    外    内    外    内    外    内
```

---

## 三、删除的文件

| 文件 | 原因 |
|------|------|
| `Algorithm/Bionics/robohorse/cycloid_generator.c` | 13行孤立代码片段，与外部 `../../../Algorithm/...` 中的完整文件重复 |

---

## 四、重构的核心文件

### 4.1 [BSP/Src/dm_j8009_ctrl.c](BSP/Src/dm_j8009_ctrl.c) — 电机控制层

**问题**：`dm_j8009_all_enable()` 和 `dm_j8009_all_disable()` 各含 8 段完全相同、仅电机 ID 和 CAN 句柄不同的复制粘贴代码。

**修复**：
- 用 `for (i = 0; i < MOTOR_COUNT; i++)` 循环 + `g_motor_config[i]` 查表替代
- `dm_j8009_all_enable()`：64 行 → 13 行
- `dm_j8009_all_disable()`：64 行 → 14 行
- `dm_j8009_all_ctrl()` 和 `dm_j8009_all_dual_ctrl()`：消除硬编码 CAN 路由判断
- 添加 `can_wait_mailbox()` 超时保护（替代无限忙等）

**代码量**：~180 行 → ~140 行，且结构清晰。

### 4.2 [BSP/Src/dm8009_drv.c](BSP/Src/dm8009_drv.c) — CAN 底层驱动

**问题**：`CLEAR_ERROR_DM_J8009`、`ENABLE_DM_J8009`、`DISABLE_DM_J8009`、`SAVE_ZERO_DM_J8009` 四个函数结构完全相同，仅最后一字节不同。

**修复**：
- 提取公共函数 `dm_j8009_send_cmd(can, id, cmd_byte)`
- 四个公开 API 变为单行包装调用
- `CTRL_DM_J8009_MIT` 中的范围限制改用 `clamp_float()` 替代嵌套三元运算符
- 反馈解析中的数字 `2047`、`4096` 替换为命名常量 `DM_12BIT_SIGN_BIT`、`DM_12BIT_FULL_RANGE`
- 修正 `@file` 标签（之前写的是 `motor_task.c/h`）

**代码量**：~75 行重复代码 → ~20 行核心 + 4 个单行函数。

### 4.3 [BSP/Inc/dm_j8009_ctrl.h](BSP/Inc/dm_j8009_ctrl.h)

- 删除未使用的 `DM8009pos_t` 类型（8 字段结构体，全局未被引用）
- 添加 `#include "motor_config.h"`
- 保留 16 个 `MOTOR_N_POS_MIN/MAX` 宏作为兼容别名

### 4.4 [BSP/Inc/dm8009.h](BSP/Inc/dm8009.h)

- 删除未使用的 `J8009_OutputCurrent_t` 类型
- 删除未使用的 `Motor_Output` 外部声明
- 删除未使用的 `J8009_SetCurrent()` 函数声明

---

## 五、类型和精度优化

### 5.1 [BSP/Src/jy901s.c](BSP/Src/jy901s.c) + [BSP/Inc/jy901s.h](BSP/Inc/jy901s.h)

| 变更 | 修复前 | 修复后 |
|------|--------|--------|
| 数据类型 | `double`（64位，软件仿真） | `float`（32位，硬件FPU） |
| 加速度转换 | `(double)a / 32768.0 * 16 * 9.8` | `(float)a * IMU_ACC_FACTOR` |
| 陀螺仪转换 | `(double)w * 2000.0 / 32768.0` | `(float)w * IMU_GYRO_FACTOR` |
| 角度转换 | `(double)a * 180 / 32768.0` | `(float)a * IMU_ANGLE_FACTOR` |
| 预计算常量 | 无 | `IMU_ACC_FACTOR`、`IMU_GYRO_FACTOR`、`IMU_ANGLE_FACTOR` |

**性能提升**：STM32F4 单精度 FPU 直接执行 float 运算，无需软件仿真 double。

---

## 六、代码清理

### 6.1 删除死代码

| 文件 | 删除内容 | 行数 |
|------|----------|------|
| [Core/Src/freertos.c](Core/Src/freertos.c) | CubeMX 自动生成的注释 task stub（5个） | ~150 行 |
| [BSP/Src/dm8009.c](BSP/Src/dm8009.c) | 注释掉的 M3508 电机代码 + PID 初始化 | ~30 行 |
| [BSP/Src/dm_j8009_ctrl.c](BSP/Src/dm_j8009_ctrl.c) | 注释掉的 ID 范围检查 | 2 行 |
| [Task/Src/ahrs_task.c](Task/Src/ahrs_task.c) | 注释掉的 IMU 初始化代码 + 矫正逻辑 | ~55 行 |
| [Core/Src/main.c](Core/Src/main.c) | 注释掉的 `hwt906.h` include 和 `HWT906_Init()` | 2 行 |
| [Task/Src/gait_param.c](Task/Src/gait_param.c) | 注释掉的结构体定义 | 8 行 |
| [BSP/Inc/jy901s.h](BSP/Inc/jy901s.h) | 注释掉的 `_ACC_`/`_GYRO_`/`_ANGLE_` 宏 | 3 行 |

### 6.2 修复编码

11 个文件从 GB2312（显示为乱码）转换为 UTF-8：

| 文件 | 修复后中文状态 |
|------|---------------|
| `BSP/Src/bsp_rc.c` | "使能DMA串口接收" 等正常 |
| `BSP/Src/dm8009_drv.c` | "达妙电机驱动程序" 等正常 |
| `BSP/Src/dm_j8009_ctrl.c` | "关节电机位置限幅" 等正常 |
| `BSP/Inc/dm8009_drv.h` | "故障码定义" 等正常 |
| `BSP/Inc/dm_j8009_ctrl.h` | "各个关节电机的机械位置" 等正常 |
| `Task/Src/process_task.c` | "步态更改处理" 等正常 |
| `Task/Src/standup_task.c` | "站立完成标志" 等正常 |
| `Task/Inc/detect_task.h` | "温度保护阈值" 等正常 |
| `Task/Inc/jump_task.h` | "目标极坐标转化成电机角度" 等正常 |
| `Task/Inc/posture_task.h` | "步态控制主任务" 等正常 |
| `Task/Inc/standup_task.h` | "站立状态标志" 等正常 |

### 6.3 修复头文件

- [Core/Inc/main.h](Core/Inc/main.h) — 删除重复的 `#include "stm32f4xx_hal.h"`
- [BSP/Inc/serial.h](BSP/Inc/serial.h) — 修复截断的 `RC_CH_VALUE_MAX` 宏

### 6.4 其他改进

- [Task/Src/ahrs_task.c](Task/Src/ahrs_task.c) — 修复未初始化的 `xLastWakeTime`（改用 `xTaskGetTickCount()` 初始值）
- [Task/Src/ahrs_task.c](Task/Src/ahrs_task.c) — `DEG_TO_RAD` 计算提取为常量，消除一行内 5 次重复计算
- [Task/Src/detect_task.c](Task/Src/detect_task.c) — 温度阈值改用 `motor_config.h` 统一常量
- [Core/Src/main.c](Core/Src/main.c) — `Error_Handler` 添加注释说明看门狗复位机制

---

## 七、API 兼容性

所有公开 API 保持向后兼容：

| 函数 | 签名 | 兼容性 |
|------|------|--------|
| `dm_j8009_pos_ctrl()` | 不变 | ✅ 完全兼容 |
| `dm_j8009_all_dual_ctrl()` | 不变 | ✅ 完全兼容 |
| `dm_j8009_all_ctrl()` | 不变 | ✅ 完全兼容 |
| `dm_j8009_all_enable()` | 不变 | ✅ 完全兼容 |
| `dm_j8009_all_disable()` | 不变 | ✅ 完全兼容 |
| `ENABLE_DM_J8009()` | 不变 | ✅ 完全兼容 |
| `DISABLE_DM_J8009()` | 不变 | ✅ 完全兼容 |
| `CTRL_DM_J8009_MIT()` | 不变 | ✅ 完全兼容 |
| `FEEDBACK_DATA_DM_J8009()` | 不变 | ✅ 完全兼容 |
| `MOTOR_N_POS_MIN/MAX` | 宏别名 | ✅ 兼容（映射到统一值） |

---

## 八、需要手动完成的步骤

### 8.1 添加新文件到 Keil 工程

在 MDK-ARM 中打开 `robohorse2022.uvprojx`，将以下文件添加到 BSP 组：

- `BSP/Src/motor_config.c`（必需）

### 8.2 推荐后续优化（暂未实施）

**Phase 4 — 全局状态整合**：将 ~30 个分散的全局变量合并为 `RobotState_t` 结构体。
- 风险：中高
- 需要：硬件测试验证

**看门狗配置**：在 `main.c` 中配置 IWDG，在 `Motor_Task` 中喂狗。
- 风险：低
- 需要：确认复位后系统能正常恢复

**任务栈审计**：`Detect`、`Standup`、`Jump` 任务栈仅 512 字节，建议增加到至少 1024 字节。

---

## 九、编译验证

```text
目标：robohorse2022
编译器：ARM Compiler V5.06 update 7 (build 960)
目标芯片：STM32F427IIHx
预期结果：0 Error(s), 0 Warning(s)
```

> **注意**：外部依赖文件（`Algorithm/`、`BSP/` 中的部分文件）仅以 `.o` 编译产物存在，源码不在此仓库中。如需完整重编译，需要从原作者处获取缺失的源文件。
