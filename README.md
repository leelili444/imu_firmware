# IMU_Firmware

STM32F4 + ICM-42688P IMU firmware.

## Overview
This project implements STM32F4 support for the ICM-42688P sensor, runs AHRS/INS algorithms, and sends telemetry at 1 kHz.

## Key Features
- ICM-42688P initialization and read: `ICM42688P_Init`, `ICM42688P_read_dma`, `ICM42688P_decode`, `ICM42688P_GetData`
- Sensor register definitions: `Icm426xxDefs.h`
- AHRS / fusion library: `Fusion` (`Fusion.h`)
- Pose estimation task: `InitializePose`, `GetPose` (ins_task.c / ins_task.h)
- Telemetry packing and 1 kHz send: `Telemetry_Send_1kHz` (mav_task.c / mav_task.h)
- Project entry and task creation: `main.c`

## Code Locations
- Core: `Core/Src/main.c`  
- IMU driver: `Core/Src/ICM42688P.c`, `Core/Inc/ICM42688P.h`  
- ICM register defs: `Core/Inc/Icm426xxDefs.h`  
- AHRS / tools: `Core/Inc/Fusion/Fusion.h`  
- INS task: `Core/Src/ins_task.c`, `Core/Inc/ins_task.h`  
- MAV task: `Core/Src/mav_task.c`, `Core/Inc/mav_task.h`  
- STM32Cube project files: `.mxproject`, `.project`

## Quick Build & Flash
- Recommended: open the project in STM32CubeIDE (`.mxproject` / `.project`).  
- Flash the built firmware to the STM32F405 via ST-LINK / CubeIDE.

## Run & Debug
- On startup, imuTask reads sensor data (see `StartImuTask` in `Core/Src/main.c`).  
- insTask runs AHRS updates and calls `GetPose`.  
- mavTask packs telemetry and sends via DMA at 1 kHz (`Telemetry_Send_1kHz`).

## Common Types & Entry Points
- Sensor data struct: `ICM42688P_Data_t` (`Core/Inc/ICM42688P.h`)  
- Telemetry frame struct: `IMU_Frame_t` (`Core/Inc/mav_task.h`)
