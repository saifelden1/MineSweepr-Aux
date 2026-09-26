# MCU #1 Comprehensive Implementation Guide & System Plan
**Project Directory:** `STM32_Auxiliary`  
**MCU Board:** STM32 BlackPill (STM32F411CEU6)  
**Operating System:** FreeRTOS (CMSIS_V2 API)

---

## 1. System Architecture & Role

MCU #1 is the **Drivetrain, Heading & CAN Communication ECU** for the MineSweeper Rover. It is dedicated to real-time closed-loop motor control (4 independent motor channels across 2x Dual-Channel Cytron MDD10A drivers), IMU orientation tracking, wheel encoder decoding, and high-speed CAN messaging.

```
                               CAN BUS (500 kbps)
                                       │
       ┌───────────────────────────────┼───────────────────────────────┐
       │                               │                               │
┌──────┴──────────────────────┐ ┌──────┴──────────────────────┐ ┌──────┴──────────────────────┐
│  MCU #1 (Mobility & IMU)    │ │  MCU #2 (Mine Detector)     │ │   Raspberry Pi 4 (Master)   │
│  - 2x Cytron MDD10A (4 Motor│ │  - Pulse Induction Detector │ │  - Xbox Camera Surface AI   │
│    Channels: 4 PWM + 4 DIR) │ │  - ADC1 + DMA Flyback       │ │  - Navigation & Mapping     │
│  - TIM2 & TIM3 Enc Hardware │ │  - Emergency CAN Broadcast  │ │  - Web Dashboard            │
│  - MPU6500 IMU (I2C1)       │ │                             │ │                             │
│  - MCP2515 CAN (SPI1)       │ │                             │ │                             │
└─────────────────────────────┘ └─────────────────────────────┘ └─────────────────────────────┘
```

---

## 2. Requirements from Other System Nodes

### A. What MCU #1 expects from MCU #2 (Mine Detector ECU)
* **`CAN ID 0x010` (Emergency Stop / Mine Detected):**
  - Priority: Highest (0x010).
  - Rate: Event-driven (Sent within < 1ms of metal tail detection).
  - Action: MCU #1 `Task_SaftyWDT` catches this frame and forces all 4 motor PWM channels to 0% in **< 2ms**.

### B. What MCU #1 expects from Raspberry Pi (Master ECU)
* **`CAN ID 0x301` (Target Velocity Command):**
  - Rate: 20 Hz.
  - Payload: `[Linear_X (mm/s), Angular_Z (mrad/s)]`.
  - Action: MCU #1 converts linear/angular speeds to individual target speeds for Left and Right motor pairs for the PID loop.

### C. What MCU #1 sends to Raspberry Pi
* **`CAN ID 0x202` (IMU Telemetry):**
  - Rate: 50 Hz.
  - Payload: `[Yaw (int16), Pitch (int16), Roll (int16), Status]`.
* **`CAN ID 0x302` (Odometry Feedback):**
  - Rate: 20 Hz.
  - Payload: `[Left_Encoder_Ticks (int32), Right_Encoder_Ticks (int32)]`.

---

## 3. Hardware Pin Assignment Table (Updated for 4 PWM Channels)

| Function | Peripheral | STM32 Pin | Connected Component & Pin | Description |
| :--- | :--- | :--- | :--- | :--- |
| **CAN SCK** | SPI1 | `PA5` | MCP2515 SCK | 10 MHz SPI Clock |
| **CAN MISO** | SPI1 | `PA6` | MCP2515 SO | SPI Master In |
| **CAN MOSI** | SPI1 | `PA7` | MCP2515 SI | SPI Master Out |
| **CAN CS** | GPIO Output | `PB0` | MCP2515 CS | Active LOW Chip Select |
| **CAN INT** | GPIO EXTI1 | `PB1` | MCP2515 INT | Active LOW Interrupt |
| **IMU SCL** | I2C1 | `PB6` | MPU6500 SCL | 400 kHz Fast Mode |
| **IMU SDA** | I2C1 | `PB7` | MPU6500 SDA | 400 kHz Fast Mode |
| **Enc L_A** | TIM2_CH1 | `PA0` | Left Encoder Channel A | Hardware Encoder Mode (TIM2) |
| **Enc L_B** | TIM2_CH2 | `PA1` | Left Encoder Channel B | Hardware Encoder Mode (TIM2) |
| **Enc R_A** | TIM3_CH1 | `PA4` | Right Encoder Channel A | Hardware Encoder Mode (TIM3) |
| **Enc R_B** | TIM3_CH2 | `PB5` (or `PA5`)| Right Encoder Channel B | Hardware Encoder Mode (TIM3) |
| **Motor FL PWM**| TIM4_CH1 | `PB6` | Driver 1 (Left) PWM1 | Front Left Motor Speed |
| **Motor FL DIR**| GPIO Output | `PB4` | Driver 1 (Left) DIR1 | Front Left Motor Direction |
| **Motor RL PWM**| TIM4_CH2 | `PB7` | Driver 1 (Left) PWM2 | Rear Left Motor Speed |
| **Motor RL DIR**| GPIO Output | `PB5` | Driver 1 (Left) DIR2 | Rear Left Motor Direction |
| **Motor FR PWM**| TIM4_CH3 | `PB8` | Driver 2 (Right) PWM1 | Front Right Motor Speed |
| **Motor FR DIR**| GPIO Output | `PC13` | Driver 2 (Right) DIR1 | Front Right Motor Direction |
| **Motor RR PWM**| TIM4_CH4 | `PB9` | Driver 2 (Right) PWM2 | Rear Right Motor Speed |
| **Motor RR DIR**| GPIO Output | `PC14` | Driver 2 (Right) DIR2 | Rear Right Motor Direction |

---

## 4. Software & Driver Directory Structure

Reflecting your exact directory layout in `STM32_Auxiliary`:

```text
STM32_Auxiliary/
└── Core/
    ├── HAL_Drivers/
    │   ├── Encoders/
    │   │   ├── Encoder.h          <-- Hardware Timer Encoder Reader (TIM2 & TIM3)
    │   │   └── Encoder.c
    │   ├── IMU_6500/
    │   │   ├── IMU_6500.h         <-- Direct ST HAL I2C MPU6500 Driver
    │   │   └── IMU_6500.c
    │   ├── MCP2515/
    │   │   ├── MCP2515.h          <-- Direct ST HAL SPI CAN Controller Driver
    │   │   └── MCP2515.c
    │   ├── MDD10A/
    │   │   ├── MDD10A.h           <-- 2x Dual-Channel Cytron Driver (4 PWM + 4 DIR)
    │   │   └── MDD10A.c
    │   └── PID_Control/
    │       ├── PID_Control.h      <-- Closed-Loop Velocity PID Math
    │       └── PID_Control.c
    └── Tasks/
        ├── Task_SaftyWDT/
        │   ├── Task_SaftyWDT.h
        │   └── Task_SaftyWDT.c    <-- Priority 5 (Emergency Stop < 2ms)
        ├── Task_Motor_control/
        │   ├── Task_Motor_control.h
        │   └── Task_Motor_control.c <-- Priority 4 (50Hz Speed PID)
        ├── Task_IMU/
        │   ├── Task_IMU.h
        │   └── Task_IMU.c         <-- Priority 3 (50Hz MPU6500 & CAN 0x202)
        └── Task_CAN_Manger/
            ├── Task_CAN_Manger.h
            └── Task_CAN_Manger.c  <-- Priority 3 (Event-Driven CAN Dispatch)
```

---

## 5. STM32CubeMX Configuration Guide

1. **System Core -> SYS:**
   * **Timebase Source:** Change from `SysTick` to **`TIM5`** *(Crucial to prevent HAL_Delay deadlocks with FreeRTOS)*.
2. **System Core -> RCC:**
   * **HSE:** Crystal/Ceramic Resonator ($25\,\text{MHz}$).
   * **System Clock:** Configure PLL to **$96\,\text{MHz}$**.
3. **Connectivity -> SPI1:**
   * **Mode:** Full-Duplex Master (`PA5` SCK, `PA6` MISO, `PA7` MOSI).
4. **Connectivity -> I2C1:**
   * **Mode:** I2C Fast Mode ($400\,\text{kHz}$) on `PB6` SCL and `PB7` SDA.
5. **Timers -> TIM2 & TIM3 (Hardware Encoder Mode):**
   * **Combined Channels:** Encoder Mode (TI1 and TI2).
   * **Function:** Counts Quadrature encoder signals automatically in hardware without CPU interrupts.
6. **Timers -> TIM4 (4-Channel PWM for Motor Drivers):**
   * **Clock Source:** Internal Clock.
   * **Channel 1, 2, 3, 4:** PWM Generation on `PB6`, `PB7`, `PB8`, `PB9` (Frequency = $20\,\text{kHz}$).
7. **Middleware -> FREERTOS:**
   * **Interface:** CMSIS_V2.

---

## 6. Implementation Task Checklist

### Task 1: STM32CubeMX & FreeRTOS Initialization
- [ ] Configure clock to 96MHz and change SYS Timebase to `TIM5`.
- [ ] Enable TIM4 Channel 1, 2, 3, 4 for 4-channel motor PWM generation.
- [ ] Enable FreeRTOS CMSIS_V2 and generate project files.

### Task 2: HAL Drivers Implementation
- [ ] Implement `MCP2515.c/h` using `HAL_SPI_TransmitReceive`.
- [ ] Implement `IMU_6500.c/h` using `HAL_I2C_Mem_Read`.
- [ ] Implement `MDD10A.c/h` for 4 PWM channels + 4 DIR pins across 2 Cytron drivers.
- [ ] Implement `Encoder.c/h` for TIM2/TIM3 hardware counter reading.
- [ ] Implement `PID_Control.c/h` velocity calculation.

### Task 3: FreeRTOS Tasks Integration
- [ ] Create `Task_SaftyWDT.c/h` (Emergency stop on CAN 0x010).
- [ ] Create `Task_Motor_control.c/h` (50Hz PID loop).
- [ ] Create `Task_IMU.c/h` (50Hz angle calculations & CAN frame 0x202).
- [ ] Create `Task_CAN_Manger.c/h` (CAN frame dispatch).

### Task 4: Verification & Bench Testing
- [ ] Verify IMU angles over UART/CAN (`candump can0` on RPi).
- [ ] Verify motor rotation across all 4 wheels independently.
- [ ] Verify instant brake when simulated mine frame `0x010` is injected on CAN bus.
