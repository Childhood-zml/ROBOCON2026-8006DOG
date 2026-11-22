# RoboHorse2022 项目架构分析

## 1. 硬件平台

| 组件 | 型号 | 说明 |
|------|------|------|
| MCU | STM32F427xx | Cortex-M4, 180MHz, FPU |
| 电机 | DM-J8009 ×8 | 达妙无刷电机, CAN总线, MIT模式控制 |
| IMU | JY901S | 9轴姿态传感器 (UART8 DMA) |
| 遥控器 | DJI 航模遥控器 | SBUS协议 (UART1 DMA) |
| 蓝牙调试 | — | UART7 中断接收 (PID调参) |
| CAN | CAN1 + CAN2 | CAN1: 电机0,1,6,7 (右前/右后); CAN2: 电机2,3,4,5 (左前/左后) |

## 2. 系统架构 (数据流)

```
遥控器SBUS帧 (UART1 DMA, 18字节)
    │
    ▼
RemoCtrl_Task (100Hz) ─── RC_Transform() 解析 ──→ my_rc_ctrl (全局)
    │                                                 │
    │  Ctrl_byRemoctrl() 更新                           │ 开关状态
    │  Cycloid_Gen[4] 参数                             ▼
    │                                        Change_NowState()
    │                                              │
    ▼                                              ▼
Posture_Task (100Hz) ──── Calculate_Gait() ──→ aim_x[4], aim_y[4]
    │                        (摆线轨迹)              │
    │                                         Target_toMotor()
    │                                         IK解算 → Aim_Angle[8]
    │                                              │
    ▼                                              ▼
Motor_Task (500Hz) ─── PID_Position[8] + PID_Speed[8] ──→ CAN发送
    │                   (双环PID: 位置环→速度环→力矩)
    │
    ▼
CAN中断回调 ─── FEEDBACK_DATA_DM_J8009() ──→ J8009[8].Total_Angle (反馈)

AHRS_Task (100Hz) ─── JY901_Read() ──→ caculate_LegRevise()
    │                                   │
    │                              imu_Leg_offset (roll补偿)
    │                              step_len_dev (yaw偏差 → 步长差)
    │
    ▼
Detect_Task (2Hz) ─── 电机温度检测, LED状态灯
```

## 3. 任务清单 (共9个FreeRTOS任务)

| 任务 | 优先级 | 周期 | 栈大小 |
|------|--------|------|--------|
| Detect_Task | Normal5 (高) | 500ms | 512B |
| AHRS_Task | Normal4 | 10ms | 2KB |
| RemoCtrl_Task | Normal3 | 10ms | 2KB |
| Motor_Task | Normal2 | 2ms (500Hz) | 2KB |
| Standup_Task | Normal1 | 单次 | 512B |
| Jump_Task | Normal1 | 事件驱动 | 512B |
| Posture_Task | Normal (基准) | 10ms (100Hz) | 2KB |
| Process_Task | BelowNormal7 (低) | 事件驱动 | 2KB |

**关键时序**: Motor_Task (500Hz) > Posture_Task (100Hz) = RemoCtrl_Task (100Hz) = AHRS_Task (100Hz)

## 4. 核心算法

### 4.1 摆线轨迹生成 (cycloid_generator.c)
- 输入: Cycloid_Generator_Param_t (步长/高度/幅度/频率/飞行占比) + phase (0~1周期)
- 输出: Cycloid_Generator_Solution_t (x, y足端坐标)
- 原理: 摆动相 x线性+y正弦弧线, 支撑相 x线性+y轻微下压
- 相位: WALK/TROT用对角步态 (腿0+2同相, 腿1+3反相)

### 4.2 逆运动学 IK (stanford_type.c)
- 使用精简版 (Lite): 2连杆串联腿模型
- `Stanford_Type_Lite_Inverse_Kinematics()`: (x,y) → (α1, α2) 关节角度
- L1=12cm (大腿), L2=20cm (小腿)

### 4.3 双环PID控制 (motor_task.c)
- 位置环: angle_err → vel_target (P+D, 无I)
- 速度环: vel_err → torque (P+D, 无I)
- 输出限幅: vel±720 RPM, torque±18 N·m

### 4.4 IMU姿态校正 (ahrs_task.c)
- roll角 → imu_Leg_offset (左右腿高度补偿, ±22°限制)
- yaw角 → step_len_dev (前后腿步长差, PID_IMU_Line)

## 5. 运动状态机

```
STOP → WALK/TROT/CLIMBING/BRIDGE/TURN_*/JUMP_* → STOP
         ↑ 遥控器开关切换触发                    ↑
         Change_NowState() 设置 Mode_Change_Flag=1
```

- 状态切换时: 重置摆线相位 + 启动身体高度平滑过渡 (800ms)
- 摇杆回中时: 停止运动但保持姿态 (400ms平滑回站立)

## 6. 遥控器映射

| 开关组合 | 状态 | 摇杆功能 |
|----------|------|----------|
| S1中+S2中 | WALK | CH3=步长/速度, CH2=转向, CH1=频率 |
| S1中+S2下 | TROT | 同上 (更快的对角步态) |
| S1中+S2上 | CLIMBING | 同上 + 爬坡高度 |
| S1上+S2中 | BRIDGE | 低趴直线 (CH2锁死) |
| S1下+S2下 | STOP (急停) | 电机全部失能 |
| S1上+S2上 | JUMP | CH1高+CH3高=上坡跳, 否则=平地跳 |

## 7. 已知问题 / FIXME / TODO 标注

### 7.1 代码中已有的 FIXME
1. **cycloid_generator.c:13** — `Cycloid_Generator()第三个参数不能为0 否则切换步态会导致一条腿残废`
   - phase=0 会导致某条腿卡死不动
   - 当前方案: 所有步态相位都加了偏移 (0.01, 0.26, 0.51, 0.76)

2. **stanford_type.c:30** — `FIXME:还没完工，网上提供的公式好像有问题`
   - 完整版IK未完成, 只用了精简版
   - 精简版假设2连杆, 实际应该是6连杆

3. **cycloid_generator.h:3** — `抄完发现是正弦轨迹生成器`
   - 实际是正弦轨迹, 不是真正的摆线

4. **Error_Handler:212** — `TODO: 添加HAL_IWDG_Refresh()支持看门狗复位`

### 7.2 架构上的潜在问题

5. **Mode_Change_Flag 多值状态机** (cycloid_generator.c:22-31)
   ```c
   if (Mode_Change_Flag == 1 || Mode_Change_Flag == 2 || Mode_Change_Flag == 3 || Mode_Change_Flag == 4)
       Mode_Change_Flag += 1;  // 累加到5后归零
   else
       Mode_Change_Flag = 0;
   ```
   这个4步递增的机制很脆弱: 用于为4条腿依次设置phase, 但如果Posture_Task调用周期与Cycloid_Generator_Calc不同步, 可能导致某些腿phase未初始化。

6. **Target_toMotor() 和 Polar_setCoord() 的电机映射不一致**
   - posture_task.c Target_toMotor(): 
     - 腿0: Aim_Angle[2]=inner, [3]=outer
     - 腿3: Aim_Angle[0]=outer, [1]=inner
   - jump_task.c Polar_setCoord():
     - 腿0: Aim_Angle[2]=inner, [3]=outer  ✓一致
     - 腿3: Aim_Angle[0]=outer, [1]=inner  ✓一致
     两者一致, 但公式略有不同 (jump_task不除以 MOTOR_ANGLE_SCALE)

7. **CAN发送时vTaskDelay(1ms)的影响**
   - dm_j8009_all_dual_ctrl() 中每个电机发送后delay 1ms
   - 8个电机 = 8ms, 但 Motor_Task 周期只有2ms
   - 这意味着: Motor_Task的500Hz目标实际上只能达到约125Hz

8. **IMU的roll方向补偿逻辑** (ahrs_task.c + motor_task.c)
   - roll → imu_Leg_offset (角度弧度)
   - 在motor_task中加到左右腿的PID_Aim_Angle上
   - 但右腿的补偿符号已经被反转 (#63-71), 这里可能再次反转导致补偿方向错误

9. **JY901_Init() 的递归初始化风险** (jy901s.c:45-52)
   - 被注释掉了, 但如果启用会导致栈溢出 (在FreeRTOS任务中调用JY901_Init不会返回)

10. **CAN过滤器配置** (dm8009.c:24-39)
    - FilterBank 14 同时用于CAN1和CAN2
    - 两个CAN共用同一个FilterBank编号可能导致冲突

11. **Standup_Task 不执行实际起立动作** (standup_task.c)
    - 整个起立序列被注释掉
    - 直接设置 flag_Standup=1 然后挂起
    - 这意味着上电后电机直接进入闭环控制, 没有从地面趴伏到站立的过渡

12. **Jump_Task 使用忙等待 (busy-wait)**
    - `while (t <= total_time)` 循环内没有任何osDelay
    - 在最高优先级下会完全阻塞其他任务约2.93秒
    - 期间CAN反馈也无法处理

13. **Posture_Task 中 CLIMBING 状态 handle**
    - 走和WALK一样的对角步态, 但没处理前后腿不同高度的协调
    - UpdateClimbingHeight() 设置了不同高度, 但摆线生成器统一使用gait_params

## 8. 调试建议

### 如果需要调试, 按优先级排列:

1. **检查CAN总线通信** — 用逻辑分析仪确认电机反馈帧是否正常
2. **验证IK解算** — 用固定(x,y)输入检查输出的Aim_Angle是否在合理范围
3. **检查电机映射** — 每条腿的两个电机(内/外)是否转动方向正确
4. **PID参数调优** — 当前PID可能过硬或过软导致振荡/无力
5. **IMU数据验证** — 检查JY901S是否正常输出roll/pitch/yaw
6. **模式切换稳定性** — Mode_Change_Flag递增机制可能导致某条腿掉相
7. **检查堆栈使用** — 各任务栈大小是否足够 (尤其是Motor/Posture 2KB)
