# ROBOCON2025\-8006DOG

[![English README](https://img.shields.io/badge/English-d9d9d9)](./README_EN.md)
[![简体中文 README](https://img.shields.io/badge/简体中文-d9d9d9)](./README.md)

> Open-source project for the 2026 ROBOCON quadruped robot (Team 2) of Suzhou University of Technology

## Introduction

This repository archives the software, control schemes and supporting documentation for **Team 2’s quadruped robot in ROBOCON 2025, Suzhou University of Technology**.

Competition results for the 2026 season: **National Third Prize for both events**

The main code framework inherits from the previous season’s Team 2 repository: [https://github.com/Childhood-zml/ROBOCON2025-8009DOG](https://github.com/Childhood-zml/ROBOCON2025-8009DOG)

Key updates and iterations:

- **Hardware recalibration**: The encoder conversion coefficient for last season was 1100.26 (reduction ratio 3591/187). After switching motors with a new reduction ratio in the 2026 season, the coefficient was updated to 515.66, with corresponding joint zero positions and mounting directions adjusted.
- **New MIT dual-loop control interface**: Only the single-loop position control function `dm_j8009_pos_ctrl()` was available last season. The 2026 season adds `dm_j8009_all_dual_ctrl()` — the position loop output serves as the velocity target, and the velocity loop output serves as torque. Independent `PID_Speed[8]` controllers are configured for the 8 joints in `dm_j8009_ctrl.c`.
- **Thermal protection restoration, bug fixes & architecture simplification**: The motor temperature polling section in `detect_task.c` was fully commented out last season (only empty LED logic remained). It has been restored in 2026 with hysteresis logic: motors disable when `T_Rotor ≥100°C` and re-enable when `<80°C`. Fixed the bug where `PID_Reset()` failed to clear the integral term. The old state machine was refactored, redundant abstractions such as `LegGain`, `DetachedParam`, and `Cycloid_Generator_t` were removed, and task-layer logic is written directly for easier debugging.

## Demo

Live photo album:

- [https://alltuu.cc/album/73a32108739142c368e1fa39230fa9c4/?menu=live](https://m/)

## Repository Directory Structure


| Directory | Description |
| --- | --- |
| `1.Software/` | Full lower-computer firmware (STM32F427, Keil uVision project: robohorse2022.uvprojx) |
| `2.Document/` | Reference documents and manuals (PDF files) |
| `3.Picture/` | Competition photos |
| `assets/` | Image assets for README |

## Quick Start

### Lower-computer Keil Project

Open [`1.Software/project_8006_2026/robohorse2022/MDK-ARM/robohorse2022.uvprojx`](https://github.com/Childhood-zml/ROBOCON2026-8006DOG/blob/main/1.Software/project_8006_2026/robohorse2022/MDK-ARM/robohorse2022.uvprojx)

> 
> Recommended environment: Keil uVision5, target chip STM32F427, compatible with DM 8006 motor drivers

## Important Notice

As an electronic control member of the team for the 2026 season, some original design documents, debugging logs and internal notes are incomplete. The materials may not be neatly organized, and we appreciate your understanding. Please feel free to raise discussions in Issues if you have any questions.

## Acknowledgements & Open-Source References

Special thanks to Ningbo University of Technology for their open-source project as our reference:

- [https://gitee.com/Ha-WthorN/robocon-nbut](https://gitee.com/Ha-WthorN/robocon-nbut)
- [https://gitlab.com/nbutroboticslab/Competition_Projects/robocon/robocon2022](https://gitlab.com/nbutroboticslab/Competition_Projects/robocon/robocon2022)

README documentation structure references:

- [https://blog.csdn.net/weixin_52612260/article/details/134124028](https://blog.csdn.net/weixin_52612260/article/details/134124028)
- [https://blog.csdn.net/Hbutneymar/article/details/147161479](https://blog.csdn.net/Hbutneymar/article/details/147161479)

Reference materials:

- [Must-read: LocoWiki Introduction](https://lain-database.feishu.cn/wiki/HwyJwB70niKbW7khmwlcbCH2nXb)

## License Statement

No `LICENSE` file is attached to this repository for now. If you intend to use this work in academic papers or commercial projects, please contact the author in advance to confirm the scope of authorization.

