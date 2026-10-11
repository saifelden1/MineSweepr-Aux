# STM32CubeMX Visual GUI Configuration Manual & Hardware Specification
## STM32_Auxiliary ECU: Landmine Detection, Robotic Gripper & Field Diagnostics

**Target Microcontroller:** STM32F411CEU6 (WeAct Studio BlackPill V3.0 / V3.1)  
**Core Architecture:** ARM 32-bit Cortex-M4 with single-precision FPU (FPv4-SP-D16) @ 96.0 MHz  
**Primary Clocks:** 25.0 MHz Onboard HSE Crystal Oscillator $\to$ Main PLL $\to$ 96.0 MHz SYSCLK  
**Operating System:** FreeRTOS (CMSIS_V2 API) with 32 KB Heap (`heap_4`)  
**SYS Timebase Source:** TIM11 (Hardware Timer, isolated from FreeRTOS SysTick)  
**High-Speed Interconnect:** micro-ROS Serial Client over USART1 DMA @ 921,600 Baud (8N1)  
**Project Path:** `e:\MineSweeper\STM32_Auxiliary\STM32_Auxiliary.ioc`  

---

## Table of Contents

1. [Executive Subsystem Overview & Architectural Topology](#1-executive-subsystem-overview--architectural-topology)
2. [Complete Physical Pinout & BlackPill Hardware Pin Mapping](#2-complete-physical-pinout--blackpill-hardware-pin-mapping)
   - [2.1. 40-Pin BlackPill Visual Pinout Reference Diagram](#21-40-pin-blackpill-visual-pinout-reference-diagram)
   - [2.2. Master Pinout Allocation Table](#22-master-pinout-allocation-table)
3. [System Core, RCC & 96 MHz Clock Tree Setup](#3-system-core-rcc--96-mhz-clock-tree-setup)
   - [3.1. Clock Frequency Derivations & PLL Equations](#31-clock-frequency-derivations--pll-equations)
   - [3.2. Clock Tree Block Diagram](#32-clock-tree-block-diagram)
   - [3.3. Click-by-Click Clock Configuration GUI Steps](#33-click-by-click-clock-configuration-gui-steps)
   - [3.4. System Core: SYS & RCC Configuration](#34-system-core-sys--rcc-configuration)
4. [Pulse Induction Metal Detector Configuration](#4-pulse-induction-metal-detector-configuration)
   - [4.1. Operating Principles & Timing Constraints](#41-operating-principles--timing-constraints)
   - [4.2. PB0 Excitation Pulse (AUX_PULSE_OUT) Configuration](#42-pb0-excitation-pulse-aux_pulse_out-configuration)
   - [4.3. PA1 Decay Sampler (ADC1_IN1 + DMA2_Stream0) Configuration](#43-pa1-decay-sampler-adc1_in1--dma2_stream0-configuration)
5. [Robotic Gripping Mechanism Configuration](#5-robotic-gripping-mechanism-configuration)
   - [5.1. 50 Hz PWM Timing Equations & Pulse Width Setpoints](#51-50-hz-pwm-timing-equations--pulse-width-setpoints)
   - [5.2. TIM4 Channels 1, 2, 3 Parameter Setup (PB6, PB7, PB8)](#52-tim4-channels-1-2-3-parameter-setup-pb6-pb7-pb8)
6. [Debug & Field Diagnostics Hardware Configuration](#6-debug--field-diagnostics-hardware-configuration)
   - [6.1. Hardware Allocation & Electrical Topologies](#61-hardware-allocation--electrical-topologies)
   - [6.2. GPIO Parameter Setup: PB12, PB13, PB14, PB15, PC13](#62-gpio-parameter-setup-pb12-pb13-pb14-pb15-pc13)
7. [micro-ROS Serial Link Configuration (USART1 DMA)](#7-micro-ros-serial-link-configuration-usart1-dma)
   - [7.1. Baud Rate Derivation (921,600 Baud @ 96 MHz APB2)](#71-baud-rate-derivation-921600-baud--96-mhz-apb2)
   - [7.2. USART1 Parameter Settings & Mode Selection](#72-usart1-parameter-settings--mode-selection)
   - [7.3. DMA2 Stream 2 (RX) & DMA2 Stream 7 (TX) Configuration](#73-dma2-stream-2-rx--dma2-stream-7-tx-configuration)
   - [7.4. NVIC Interrupt Controller Configuration](#74-nvic-interrupt-controller-configuration)
8. [FreeRTOS (CMSIS_V2) Real-Time Operating System Setup](#8-freertos-cmsis_v2-real-time-operating-system-setup)
   - [8.1. Kernel Parameters & 32 KB Memory Heap Allocation](#81-kernel-parameters--32-kb-memory-heap-allocation)
   - [8.2. Task Allocation: Task_Detector, Task_Gripper, Task_MicroROS](#82-task-allocation-task_detector-task_gripper-task_microros)
9. [Project Manager Settings & Zero-Code-Deletion Strategy](#9-project-manager-settings--zero-code-deletion-strategy)
   - [9.1. Project Properties & Toolchain Selection](#91-project-properties--toolchain-selection)
   - [9.2. Code Generator Settings for Firmware Preservation](#92-code-generator-settings-for-firmware-preservation)
10. [Step-by-Step Validation & Hardware Verification Checklist](#10-step-by-step-validation--hardware-verification-checklist)

---

## 1. Executive Subsystem Overview & Architectural Topology

The `STM32_Auxiliary` Electronic Control Unit (ECU) functions as the specialized landmine detection and manipulator payload controller on the Autonomous Minesweeper Rover. It is electrically isolated from the high-current mobility drivetrain and executes deterministic real-time sensing, coil excitation, and servo articulation tasks while bridging telemetry with ROS 2 Humble running on a Raspberry Pi 5.

```
                                ┌────────────────────────────────────────────────────────┐
                                │           Host SBC: Raspberry Pi 5 (ROS 2 Humble)      │
                                │  • micro_ros_agent Serial Node @ 921,600 Baud          │
                                │  • Topics: /metal_detector/reading, /gripper/status    │
                                │  • Subscriptions: /gripper/command, /beacon/alert      │
                                └───────────────────────────┬────────────────────────────┘
                                                            │ USART1 Full-Duplex DMA
                                                            │ PA9 (TX) / PA10 (RX)
                                                            ▼
                                ┌────────────────────────────────────────────────────────┐
                                │                  STM32_Auxiliary ECU                   │
                                │               (STM32F411CEU6 BlackPill)                │
                                ├────────────────────────────────────────────────────────┤
                                │ 1. Pulse Induction Metal Detector:                     │
                                │    - PB0: IRF740 N-MOSFET Gate (70 µs pulse @ 1 kHz)   │
                                │    - PA1: TL072 Decay Sampler (ADC1_IN1 via DMA2_S0)   │
                                │ 2. Robotic Gripping Mechanism:                         │
                                │    - PB6: TIM4_CH1 (PWM 50 Hz - Base / Jaw 1)          │
                                │    - PB7: TIM4_CH2 (PWM 50 Hz - Wrist / Jaw 2)         │
                                │    - PB8: TIM4_CH3 (PWM 50 Hz - Lift / Payload Axis)   │
                                │ 3. Field Diagnostics & Warning Stage:                  │
                                │    - PB12: DEBUG_BUZZER (Active Siren Alarm Drive)     │
                                │    - PB13: DEBUG_LED1_RED (Mine Strobe / Alert)        │
                                │    - PB14: DEBUG_LED2_YELLOW (Gripper Busy Indicator)  │
                                │    - PB15: DEBUG_LED3_GREEN (System Ready / ROS Link)  │
                                │    - PC13: HEARTBEAT_LED (Active-LOW User Blink)       │
                                │ 4. Core Infrastructure:                                │
                                │    - RCC: 25.0 MHz HSE Crystal -> 96.0 MHz SYSCLK      │
                                │    - SYS: TIM11 Timebase Source (1 ms Tick for HAL)    │
                                │    - FreeRTOS CMSIS_V2: 32 KB Heap, 3 Dedicated Tasks  │
                                └────────────────────────────────────────────────────────┘
```

---

## 2. Complete Physical Pinout & BlackPill Hardware Pin Mapping

### 2.1. 40-Pin BlackPill Visual Pinout Reference Diagram

```
                               ┌────────────────┐
                               │  USB-C CONNECTOR
                               └────────────────┘
      [DEBUG_BUZZER]    PB12   1 ──              ── 40  VBAT
     [DEBUG_LED1_RED]   PB13   2 ──              ── 39  PC13   [HEARTBEAT_LED] (Active LOW)
   [DEBUG_LED2_YELLOW]  PB14   3 ──              ── 38  PC14   [OSC32_IN]
    [DEBUG_LED3_GREEN]  PB15   4 ──              ── 37  PC15   [OSC32_OUT]
                        PA8    5 ──  STM32F411   ── 36  NRST   [System Reset]
         [MICROROS_TX]  PA9    6 ──  BlackPill   ── 35  PA0    [WKUP]
         [MICROROS_RX]  PA10   7 ──  (Top View)  ── 34  PA1    [AUX_DECAY_IN] (ADC1_IN1)
                        PA11   8 ──              ── 33  PA2
                        PA12   9 ──  ARM 96 MHz  ── 32  PA3
                        PA15  10 ──              ── 31  PA4
                        PB3   11 ──              ── 30  PA5
                        PB4   12 ──              ── 29  PA6
                        PB5   13 ──              ── 28  PA7
     [AUX_SERVO1_PWM]   PB6   14 ──              ── 27  PB0    [AUX_PULSE_OUT] (MOSFET Gate)
     [AUX_SERVO2_PWM]   PB7   15 ──              ── 26  PB1
     [AUX_SERVO3_PWM]   PB8   16 ──              ── 25  PB2    [BOOT1]
                        PB9   17 ──              ── 24  PB10
                        5V    18 ──              ── 23  3V3
                        GND   19 ──              ── 22  GND
                        3V3   20 ──              ── 21  5V
                               └──┬────┬────┬───┘
                                  │    │    │   
                                 GND  CLK  DIO  3V3
                                      [SWD HEADER]
                                  PA14=SWCLK, PA13=SWDIO
```

### 2.2. Master Pinout Allocation Table

| Pin | Selected Mode | Alternate Function | User Label | Subsystem / Connected Hardware | Electrical Characteristics |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **PB0** | `GPIO_Output` | — | `AUX_PULSE_OUT` | IRF740 N-MOSFET Gate Driver | Push-Pull, Very High Speed, Initial Low |
| **PA1** | `ADC1_IN1` | Analog | `AUX_DECAY_IN` | TL072 Decay Tail Sampler | Analog Input, 12-bit, Sample Time 15 Cyc |
| **PB6** | `TIM4_CH1` | AF2 (`TIM4`) | `AUX_SERVO1_PWM` | Gripper Servo 1 (Jaw 1 / Base) | Alternate Function Push-Pull, 50 Hz PWM |
| **PB7** | `TIM4_CH2` | AF2 (`TIM4`) | `AUX_SERVO2_PWM` | Gripper Servo 2 (Jaw 2 / Wrist) | Alternate Function Push-Pull, 50 Hz PWM |
| **PB8** | `TIM4_CH3` | AF2 (`TIM4`) | `AUX_SERVO3_PWM` | Gripper Servo 3 (Aux / Lift Axis) | Alternate Function Push-Pull, 50 Hz PWM |
| **PB12** | `GPIO_Output` | — | `DEBUG_BUZZER` | NPN Transistor Driving 5V Siren | Push-Pull, Low Speed, Initial Low |
| **PB13** | `GPIO_Output` | — | `DEBUG_LED1_RED` | Mine Alert Warning LED (Red) | Push-Pull, Low Speed, Initial Low |
| **PB14** | `GPIO_Output` | — | `DEBUG_LED2_YELLOW` | Gripper Busy Warning LED (Yellow) | Push-Pull, Low Speed, Initial Low |
| **PB15** | `GPIO_Output` | — | `DEBUG_LED3_GREEN` | System Ready / ROS Active (Green) | Push-Pull, Low Speed, Initial Low |
| **PC13** | `GPIO_Output` | — | `HEARTBEAT_LED` | Onboard BlackPill User LED | Open Drain, Low Speed, Initial High (OFF) |
| **PA9** | `USART1_TX` | AF7 (`USART1`) | `MICROROS_TX` | Raspberry Pi 5 Serial RX Line | AF Push-Pull, High Speed, Pull-up |
| **PA10** | `USART1_RX` | AF7 (`USART1`) | `MICROROS_RX` | Raspberry Pi 5 Serial TX Line | AF Input / Floating, High Speed |
| **PA13** | `SYS_JTMS-SWDIO`| AF0 (`SYS`) | `SWDIO` | ST-Link v2 / v3 Debug Data Line | Internal Pull-Up |
| **PA14** | `SYS_JTCK-SWCLK`| AF0 (`SYS`) | `SWCLK` | ST-Link v2 / v3 Debug Clock Line | Internal Pull-Down |
| **PH0** | `RCC_OSC_IN` | Analog | `OSC_IN` | 25.0 MHz Quartz Crystal Input | External HSE Oscillator |
| **PH1** | `RCC_OSC_OUT` | Analog | `OSC_OUT` | 25.0 MHz Quartz Crystal Output | External HSE Oscillator |

---

## 3. System Core, RCC & 96 MHz Clock Tree Setup

### 3.1. Clock Frequency Derivations & PLL Equations

The WeAct Studio STM32F411CEU6 board carries an onboard **25.0 MHz** High-Speed External (HSE) quartz crystal. The internal Phase-Locked Loop (PLL) multiplies and divides this frequency to reach the targeted maximum system clock frequency of **96.0 MHz**.

$$\begin{aligned}
f_{\text{VCO\_IN}} &= \frac{f_{\text{HSE}}}{M} = \frac{25\,\text{MHz}}{25} = \mathbf{1.0\,\text{MHz}} \quad (\text{Constraint: } 1.0\,\text{MHz} \le f_{\text{VCO\_IN}} \le 2.0\,\text{MHz}) \\[8pt]
f_{\text{VCO\_OUT}} &= f_{\text{VCO\_IN}} \times N = 1.0\,\text{MHz} \times 192 = \mathbf{192.0\,\text{MHz}} \quad (\text{Constraint: } 100\,\text{MHz} \le f_{\text{VCO\_OUT}} \le 432\,\text{MHz}) \\[8pt]
f_{\text{SYSCLK}} &= \frac{f_{\text{VCO\_OUT}}}{P} = \frac{192.0\,\text{MHz}}{2} = \mathbf{96.0\,\text{MHz}} \quad (\text{Constraint: } \le 100\,\text{MHz for STM32F411}) \\[8pt]
f_{\text{HCLK}} &= \frac{f_{\text{SYSCLK}}}{\text{AHB\_PRESC}} = \frac{96.0\,\text{MHz}}{1} = \mathbf{96.0\,\text{MHz}} \quad (\text{Core, Memory, DMA Clock}) \\[8pt]
f_{\text{APB1}} &= \frac{f_{\text{HCLK}}}{\text{APB1\_PRESC}} = \frac{96.0\,\text{MHz}}{2} = \mathbf{48.0\,\text{MHz}} \quad (\text{Max allowed for APB1: } 50\,\text{MHz}) \\[8pt]
f_{\text{TIM\_APB1}} &= 2 \times f_{\text{APB1}} = 2 \times 48.0\,\text{MHz} = \mathbf{96.0\,\text{MHz}} \quad (\text{Because APB1 Prescaler } \ne 1 \implies \times 2) \\[8pt]
f_{\text{APB2}} &= \frac{f_{\text{HCLK}}}{\text{APB2\_PRESC}} = \frac{96.0\,\text{MHz}}{1} = \mathbf{96.0\,\text{MHz}} \quad (\text{Max allowed for APB2: } 100\,\text{MHz}) \\[8pt]
f_{\text{TIM\_APB2}} &= 1 \times f_{\text{APB2}} = \mathbf{96.0\,\text{MHz}} \quad (\text{Because APB2 Prescaler } = 1 \implies \times 1) \\[8pt]
f_{\text{USB/SDIO}} &= \frac{f_{\text{VCO\_OUT}}}{Q} = \frac{192.0\,\text{MHz}}{4} = \mathbf{48.0\,\text{MHz}} \quad (\text{Precise } 48.0\,\text{MHz USB OTG FS Clock})
\end{aligned}$$

### 3.2. Clock Tree Block Diagram

```
 [25 MHz HSE]
      │
      ▼
 [ /M = 25 ] ────► 1.0 MHz VCO Input
      │
      ▼
 [ *N = 192 ] ───► 192.0 MHz VCO Output
      │
      ├───────────► [ /P = 2 ] ──► 96.0 MHz (PLLCLK)
      │                                 │
      │                                 ▼ [SYSCLK Mux: PLLCLK]
      │                           96.0 MHz SYSCLK
      │                                 │
      │                                 ▼ [AHB Prescaler: /1]
      │                           96.0 MHz HCLK (Cortex-M4, DMA2, Flash)
      │                            ├──► FCLK Cortex: 96.0 MHz
      │                            │
      │                            ├──► [APB1 Prescaler: /2] ──► 48.0 MHz PCLK1
      │                            │                                   │
      │                            │                                   └─► APB1 Timers (x2): 96.0 MHz [TIM4]
      │                            │
      │                            └──► [APB2 Prescaler: /1] ──► 96.0 MHz PCLK2 [USART1, ADC1]
      │                                                                │
      │                                                                └─► APB2 Timers (x1): 96.0 MHz [TIM11]
      │
      └───────────► [ /Q = 4 ] ──► 48.0 MHz USB / SDIO / RNG Clock
```

### 3.3. Click-by-Click Clock Configuration GUI Steps

1. In STM32CubeMX, click the **Clock Configuration** tab in the top navigation ribbon.
2. In the input box labeled **Input frequency (HSE)** on the left, type `25`.
3. Locate the **PLL Source Mux** radio button group: select **HSE** (crystal path).
4. Locate the PLL parameter fields:
   - Set **PLLM** dropdown to `/25`.
   - Set ***N** multiplier field to `192`.
   - Set **PLLP** dropdown to `/2`.
   - Set **PLLQ** dropdown to `/4`.
5. Locate the **System Clock Mux**: select the radio button for **PLLCLK**.
6. Set the **AHB Prescaler** dropdown to `/1` $\implies$ verify **HCLK (MHz)** reads `96`.
7. Set the **APB1 Prescaler** dropdown to `/2` $\implies$ verify **APB1 peripheral clocks (PCLK1)** reads `48` and **APB1 Timer clocks** reads `96`.
8. Set the **APB2 Prescaler** dropdown to `/1` $\implies$ verify **APB2 peripheral clocks (PCLK2)** reads `96` and **APB2 Timer clocks** reads `96`.
9. Verify no field displays in red highlighting.

### 3.4. System Core: SYS & RCC Configuration

#### A. RCC Configuration
- **Navigation Pathway:** Left panel $\to$ **System Core** $\to$ **RCC**.
- **Mode Panel:**
  - **High Speed Clock (HSE):** Select `Crystal/Ceramic Resonator` from the dropdown list.
  - **Low Speed Clock (LSE):** Leave as `Disable`.

#### B. SYS Configuration (HAL Timebase & Debugger)
- **Navigation Pathway:** Left panel $\to$ **System Core** $\to$ **SYS**.
- **Mode Panel:**
  - **Debug:** Select `Serial Wire` (allocates PA13 to SWDIO and PA14 to SWCLK).
  - **Timebase Source:** Click the dropdown and select `TIM11`.

> [!IMPORTANT]
> **Why TIM11 is Mandatory as Timebase Source:**  
> FreeRTOS CMSIS_V2 takes exclusive control of the Cortex-M SysTick timer for RTOS kernel context switching at the lowest interrupt priority (Preemption Priority 15). If the HAL library also relied on SysTick, calling `HAL_Delay()` or waiting on HAL peripheral flags inside FreeRTOS tasks or ISRs would induce deadlock conditions. Designating `TIM11` provides an independent, unmasked 1 ms timebase for HAL operations.

---

## 4. Pulse Induction Metal Detector Configuration

### 4.1. Operating Principles & Timing Constraints

The Pulse Induction (PI) metal detector excites a search coil inductor by pulsing an IRF740 N-channel power MOSFET gate for $70\,\mu\text{s}$. When turned off, the coil experiences a back-EMF flyback spike clamped by zener/damping networks, followed by an exponential decay curve. Conductive/ferrous targets produce opposing eddy currents that alter the decay time constant.

```
       PB0 (Pulse)  ┌──────────────────┐
       Gate Drive   │  70 µs On Time   │
     ───────────────┘                  └──────────────────────────────────────────────
                                       ◄──20 µs──►◄─────── 50 µs Sampling ────────►
                                        Blanking  
     Coil Voltage  ▲ High-Voltage Flyback         ┌───────────────────────────────┐
       & Op-Amp    │ Spike (Clamped)              │ Eddy-Current Target Tail      │
       Decay Tail  │                              │ Sampled by ADC1 via DMA2_S0   │
     ──────────────┴──────────────────────────────┴───────────────────────────────┴──
```

- **PB0 Pulse Generation:** Executed via GPIO high-speed toggling with cycle-accurate timing or microsecond delays.
- **PA1 Decay Acquisition:** 12-bit ADC conversion triggered following the $20\,\mu\text{s}$ blanking delay, transferred via DMA2 Stream 0 in HalfWord format.

### 4.2. PB0 Excitation Pulse (AUX_PULSE_OUT) Configuration

1. In the **Pinout & Configuration** tab, locate pin **PB0** on the interactive BlackPill chip diagram.
2. Left-click pin **PB0** and select **GPIO_Output**.
3. In the left panel, navigate to **System Core** $\to$ **GPIO**.
4. In the GPIO table, click the row for **PB0** and configure the properties in the lower pane:

| Property | Value | Rationale |
| :--- | :--- | :--- |
| **GPIO Output Level** | `Low` | Keeps IRF740 MOSFET off at reset to prevent coil burnout |
| **GPIO Mode** | `Output Push Pull` | Provides active sourcing and sinking for gate capacitance |
| **GPIO Pull-up/Pull-down** | `No pull-up and no pull-down` | Gate driver includes external $10\,\text{k}\Omega$ pulldown |
| **Maximum output speed** | `Very High` | Minimizes rise/fall transition times to under $5\,\text{ns}$ |
| **User Label** | `AUX_PULSE_OUT` | Standardized firmware HAL identifier |

### 4.3. PA1 Decay Sampler (ADC1_IN1 + DMA2_Stream0) Configuration

#### Step 1: Pin Assignment & Mode Selection
1. Click pin **PA1** on the BlackPill chip view and select **ADC1_IN1**.
2. In the left navigation panel, expand **Analog** $\to$ **ADC1**.
3. In the Mode window, check the box for **IN1** (Channel 1).

#### Step 2: ADC1 Parameter Settings
Click the **Parameter Settings** tab in the lower pane and configure the exact register fields:

| Parameter Category | Setting Field | Configured Value | Explanation |
| :--- | :--- | :--- | :--- |
| **ADC_Settings** | Clock Prescaler | `PCLK2 divided by 4` | $f_{\text{ADC}} = 96\,\text{MHz} / 4 = 24\,\text{MHz}$ (within $\le 36\,\text{MHz}$ limit) |
| | Resolution | `12 bits (15 ADC clock cycles)` | Highest granularity ($0 \dots 4095$ raw counts) |
| | Data Alignment | `Right alignment` | Standard 12-bit format in 16-bit word |
| | Scan Conversion Mode | `Disabled` | Single regular channel acquisition |
| | Continuous Conversion Mode | `Disabled` | Software/burst triggered per pulse cycle |
| | Discontinuous Mode | `Disabled` | N/A |
| | DMA Continuous Requests | `Disabled` | Burst initiated per measurement window |
| | End Of Conversion Selection | `End of single conversion` | Single channel conversion complete |
| **ADC_Regular_ConversionMode** | Number Of Conversion | `1` | One conversion per trigger event |
| **Rank 1** | Channel | `Channel 1` | Pin PA1 |
| | Sampling Time | `15 Cycles` | Total cycles: $15 + 15 = 30$ ADC cycles ($1.25\,\mu\text{s}$) |

#### Step 3: DMA Settings for ADC1
1. Switch to the **DMA Settings** tab inside the ADC1 configuration view.
2. Click the **Add** button.
3. In the **DMA Request** column, select **ADC1**.
4. Confirm the assigned stream is **DMA2 Stream 0**.
5. Set the parameters in the DMA configuration table:

| DMA Setting | Value | Rationale |
| :--- | :--- | :--- |
| **Stream** | `DMA2 Stream 0` | Hardware mapped stream for ADC1 |
| **Direction** | `Peripheral To Memory` | Transfer ADC conversion data into RAM buffer |
| **Priority** | `Medium` | Balanced priority below micro-ROS serial RX |
| **Mode** | `Normal` | Transfers fixed batch of oversamples, then halts |
| **Increment Address: Peripheral**| `Disable` | Fixed at ADC1 Data Register (`ADC1->DR`) |
| **Increment Address: Memory** | `Enable` | Steps through RAM array elements |
| **Data Width: Peripheral** | `Half Word` | 16-bit width matches 12-bit ADC data |
| **Data Width: Memory** | `Half Word` | 16-bit elements in `uint16_t` decay buffer |
| **Use FIFO** | `Disable` | Direct peripheral-to-memory transfer |

#### Step 4: NVIC Settings for ADC1 DMA
1. In the left panel, navigate to **System Core** $\to$ **NVIC**.
2. Ensure the checkbox for **DMA2 stream 0 global interrupt** is **CHECKED**.
3. Set **Preemption Priority** to `5` and **Sub Priority** to `0`.

---

## 5. Robotic Gripping Mechanism Configuration

### 5.1. 50 Hz PWM Timing Equations & Pulse Width Setpoints

The robot manipulator features 3 heavy-duty RC servos (Base/Jaw 1, Wrist/Jaw 2, Lift/Payload Axis). Standard RC servo protocol mandates a **50 Hz frame frequency** (20 ms period) with pulse widths between $1.0\,\text{ms}$ and $2.0\,\text{ms}$.

$$\begin{aligned}
f_{\text{TIM4\_CLK}} &= 96.0\,\text{MHz} \quad (\text{Derived from APB1 timer clock doubler}) \\[6pt]
\text{Target Counter Tick} &= 1.0\,\mu\text{s} \implies f_{\text{CNT}} = 1.0\,\text{MHz} \\[6pt]
\text{Prescaler (PSC)} &= \frac{f_{\text{TIM4\_CLK}}}{f_{\text{CNT}}} - 1 = \frac{96\,000\,000}{1\,000\,000} - 1 = 96 - 1 = \mathbf{95} \\[6pt]
\text{Target PWM Period} &= 20\,\text{ms} = 20\,000\,\mu\text{s} \\[6pt]
\text{Counter Period (ARR)} &= (\text{Period} \times f_{\text{CNT}}) - 1 = (20\,000 \times 1) - 1 = \mathbf{19999} \\[6pt]
f_{\text{PWM}} &= \frac{f_{\text{TIM4\_CLK}}}{(\text{PSC} + 1) \times (\text{ARR} + 1)} = \frac{96\,000\,000}{96 \times 20\,000} = \mathbf{50.0\,\text{Hz}} \quad (\text{Exact } 20.0\,\text{ms})
\end{aligned}$$

#### Servo Pulse Width Mapping Table

| Mechanical Position | Pulse Width ($t_{\text{high}}$) | CCR Register Value | Duty Cycle | Function |
| :--- | :--- | :--- | :--- | :--- |
| **Retracted / Open** | $1.0\,\text{ms}$ ($1000\,\mu\text{s}$) | `1000` | $5.0\%$ | Fully open jaws / envelope entry |
| **Neutral / Travel** | $1.5\,\text{ms}$ ($1500\,\mu\text{s}$) | `1500` | $7.5\%$ | Safe driving / baseline orientation |
| **Clamped / Gripped**| $2.0\,\text{ms}$ ($2000\,\mu\text{s}$) | `2000` | $10.0\%$ | Clamped on surface landmine sample |

### 5.2. TIM4 Channels 1, 2, 3 Parameter Setup (PB6, PB7, PB8)

#### Step 1: Mode & Channel Selection
1. In the left navigation panel, expand **Timers** $\to$ **TIM4**.
2. In the TIM4 Mode panel:
   - **Slave Mode:** Select `Disable`.
   - **Clock Source:** Select `Internal Clock`.
   - **Channel 1:** Select `PWM Generation CH1` (automatically maps to **PB6**).
   - **Channel 2:** Select `PWM Generation CH2` (automatically maps to **PB7**).
   - **Channel 3:** Select `PWM Generation CH3` (automatically maps to **PB8**).
   - **Channel 4:** Leave as `Disable`.

#### Step 2: TIM4 Parameter Configuration
Switch to the **Parameter Settings** tab in the lower pane:

| Parameter Category | Configuration Field | Setting Value | Detailed Notes |
| :--- | :--- | :--- | :--- |
| **Counter Settings** | Prescaler (PSC - 16 bits value) | `95` | Yields $1\,\mu\text{s}$ counter resolution ($96\,\text{MHz} / 96$) |
| | Counter Mode | `Up` | Standard upcounting timer mode |
| | Counter Period (ARR - 16 bits) | `19999` | 20,000 counts @ $1\,\mu\text{s} = 20\,\text{ms}$ ($50\,\text{Hz}$) |
| | Internal Clock Division (CKD) | `No Division` | Standard clock input |
| | auto-reload preload | `Enable` | Updates ARR synchronously on update event |
| **PWM Generation CH1** | Mode | `PWM mode 1` | Output active while counter $<$ pulse value |
| | Pulse (16 bits value) | `1500` | Neutral position ($1.5\,\text{ms}$ pulse width) |
| | Output compare fast mode | `Disable` | Standard compare operation |
| | CH Polarity | `High` | Active-HIGH pulse drive |
| **PWM Generation CH2** | Mode | `PWM mode 1` | Output active while counter $<$ pulse value |
| | Pulse (16 bits value) | `1500` | Neutral position ($1.5\,\text{ms}$ pulse width) |
| | Output compare fast mode | `Disable` | Standard compare operation |
| | CH Polarity | `High` | Active-HIGH pulse drive |
| **PWM Generation CH3** | Mode | `PWM mode 1` | Output active while counter $<$ pulse value |
| | Pulse (16 bits value) | `1500` | Neutral position ($1.5\,\text{ms}$ pulse width) |
| | Output compare fast mode | `Disable` | Standard compare operation |
| | CH Polarity | `High` | Active-HIGH pulse drive |

#### Step 3: Pin Labels Assignment
1. Click pin **PB6** on the chip view $\to$ click **Enter User Label** $\to$ type `AUX_SERVO1_PWM`.
2. Click pin **PB7** on the chip view $\to$ click **Enter User Label** $\to$ type `AUX_SERVO2_PWM`.
3. Click pin **PB8** on the chip view $\to$ click **Enter User Label** $\to$ type `AUX_SERVO3_PWM`.

---

## 6. Debug & Field Diagnostics Hardware Configuration

### 6.1. Hardware Allocation & Electrical Topologies

The Auxiliary ECU features a comprehensive diagnostic interface conforming to competition judging standards:

1. **Audible Siren Stage (PB12):** Drives an external NPN transistor / piezo buzzer for acoustic mine alerts.
2. **Mine Strobe Red LED (PB13):** High-intensity visual flasher activated upon target confirmation.
3. **Gripper Busy Yellow LED (PB14):** Visual indicator active during manipulator articulation.
4. **System Ready Green LED (PB15):** Illuminates solid when micro-ROS session is active and healthy.
5. **Heartbeat LED (PC13):** Onboard BlackPill user LED toggled at 2 Hz by the safety background task.

```
                  ┌────────────────────────────────────────────────────────┐
                  │                 STM32F411CEU6 BlackPill                │
                  │                                                        │
                  │   PB12 (Push-Pull) ──► 1kΩ ──► [NPN Base] ──► Buzzer   │
                  │   PB13 (Push-Pull) ──► 330Ω ──► [Red LED]  ──► GND      │
                  │   PB14 (Push-Pull) ──► 330Ω ──► [Yel LED]  ──► GND      │
                  │   PB15 (Push-Pull) ──► 330Ω ──► [Grn LED]  ──► GND      │
                  │   PC13 (Open-Drain) ──► [Onboard Blue LED] ──► 3.3V     │
                  └────────────────────────────────────────────────────────┘
```

### 6.2. GPIO Parameter Setup: PB12, PB13, PB14, PB15, PC13

In the interactive chip view or via **System Core** $\to$ **GPIO**, configure the pins as follows:

| Pin Name | Pin Mode | Output Level | Output Mode | Pull-Up / Pull-Down | Maximum Speed | User Label |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **PB12** | `GPIO_Output` | `Low` | `Output Push Pull` | `No pull-up and no pull-down` | `Low` | `DEBUG_BUZZER` |
| **PB13** | `GPIO_Output` | `Low` | `Output Push Pull` | `No pull-up and no pull-down` | `Low` | `DEBUG_LED1_RED` |
| **PB14** | `GPIO_Output` | `Low` | `Output Push Pull` | `No pull-up and no pull-down` | `Low` | `DEBUG_LED2_YELLOW` |
| **PB15** | `GPIO_Output` | `Low` | `Output Push Pull` | `No pull-up and no pull-down` | `Low` | `DEBUG_LED3_GREEN` |
| **PC13** | `GPIO_Output` | `High` | `Output Open Drain`| `No pull-up and no pull-down` | `Low` | `HEARTBEAT_LED` |

> [!NOTE]
> **PC13 Active-LOW Operation:**  
> On the WeAct Studio BlackPill, the onboard user LED cathode is connected to PC13 while its anode connects to 3.3V through a current-limiting resistor. Writing `GPIO_PIN_RESET` turns the LED ON, while writing `GPIO_PIN_SET` turns the LED OFF. An initial level of `High` ensures the LED is OFF at system startup.

---

## 7. micro-ROS Serial Link Configuration (USART1 DMA)

### 7.1. Baud Rate Derivation (921,600 Baud @ 96 MHz APB2)

The serial link between the STM32_Auxiliary ECU and Raspberry Pi 5 runs at **921,600 Baud**. USART1 resides on the high-speed APB2 peripheral bus ($f_{\text{PCLK2}} = 96.0\,\text{MHz}$).

$$\begin{aligned}
\text{USARTDIV} &= \frac{f_{\text{PCLK2}}}{16 \times \text{Baud Rate}} = \frac{96\,000\,000}{16 \times 921\,600} = \frac{96\,000\,000}{14\,745\,600} \approx \mathbf{6.510416} \\[8pt]
\text{DIV\_Mantissa} &= \lfloor 6.510416 \rfloor = \mathbf{6} \quad (\text{Hex: } 0x06) \\[8pt]
\text{DIV\_Fraction} &= \text{round}(0.510416 \times 16) = \text{round}(8.1666) = \mathbf{8} \quad (\text{Hex: } 0x08) \\[8pt]
\text{BRR Register Value} &= (\text{DIV\_Mantissa} \ll 4) \mid \text{DIV\_Fraction} = (6 \ll 4) \mid 8 = \mathbf{0x0068} \\[8pt]
\text{Actual Baud Rate} &= \frac{96\,000\,000}{16 \times \left(6 + \frac{8}{16}\right)} = \frac{96\,000\,000}{16 \times 6.5} = \mathbf{923\,076.9\,\text{Baud}} \\[8pt]
\text{Baud Error Rate} &= \frac{923\,076.9 - 921\,600}{921\,600} \times 100\% = \mathbf{+0.16\%} \quad (\text{Well below } \pm 2.0\% \text{ tolerance})
\end{aligned}$$

### 7.2. USART1 Parameter Settings & Mode Selection

1. In the left navigation panel, expand **Connectivity** $\to$ **USART1**.
2. In the USART1 Mode panel:
   - **Mode:** Select `Asynchronous`.
   - **Hardware Flow Control (RS232):** Select `Disable`.
3. In the chip diagram, confirm **PA9** is set to `USART1_TX` and **PA10** is set to `USART1_RX`.
4. Switch to the **Parameter Settings** tab in the lower pane:

| Configuration Property | Target Setting | Explanation |
| :--- | :--- | :--- |
| **Baud Rate** | `921600 Bits/s` | High-speed ROS 2 serial telemetry transport |
| **Word Length** | `8 Bits (including Parity)` | Standard 8-bit byte frames |
| **Parity** | `None` | No parity bit |
| **Stop Bits** | `1` | Single stop bit (8N1) |
| **Data Direction** | `Receive and Transmit` | Full duplex bidirectional communication |
| **Over Sampling** | `16 Samples` | Maximum noise rejection and sample stability |

### 7.3. DMA2 Stream 2 (RX) & DMA2 Stream 7 (TX) Configuration

Using DMA offloads all byte shifting from the CPU, enabling deterministic execution of metal detector pulse loops without micro-ROS dropped frames.

1. In the USART1 view, click the **DMA Settings** tab.
2. Configure **USART1_RX** Stream:
   - Click **Add** $\to$ select `USART1_RX`.
   - Ensure the assigned stream is **DMA2 Stream 2**.
   - Set **Direction:** `Peripheral To Memory`.
   - Set **Priority:** `Very High`.
   - Set **Mode:** `Circular` (essential for continuous ring buffer reception).
   - Set **Increment Address:** Peripheral `Disable`, Memory `Enable`.
   - Set **Data Width:** Peripheral `Byte`, Memory `Byte`.
   - Set **Use FIFO:** `Disable`.
3. Configure **USART1_TX** Stream:
   - Click **Add** $\to$ select `USART1_TX`.
   - Ensure the assigned stream is **DMA2 Stream 7**.
   - Set **Direction:** `Memory To Peripheral`.
   - Set **Priority:** `High`.
   - Set **Mode:** `Normal` (triggered per outbound message packet).
   - Set **Increment Address:** Peripheral `Disable`, Memory `Enable`.
   - Set **Data Width:** Peripheral `Byte`, Memory `Byte`.
   - Set **Use FIFO:** `Disable`.

#### Master DMA Table for STM32_Auxiliary

| DMA Request | Stream | Channel | Direction | Priority | Mode | Increment | Data Width |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **ADC1** | `DMA2 Stream 0` | Ch 0 | Periph to Mem | Medium | Normal | Mem Only | HalfWord (16b) |
| **USART1_RX** | `DMA2 Stream 2` | Ch 4 | Periph to Mem | Very High | Circular | Mem Only | Byte (8b) |
| **USART1_TX** | `DMA2 Stream 7` | Ch 4 | Mem to Periph | High | Normal | Mem Only | Byte (8b) |

### 7.4. NVIC Interrupt Controller Configuration

1. In the USART1 view, click the **NVIC Settings** tab:
   - Check the box for **USART1 global interrupt** $\to$ set to **Enabled**.
2. Navigate to **System Core** $\to$ **NVIC**:
   - Verify **Priority Group** is set to `4 bits for pre-emption priority, 0 bits for sub-priority` (`NVIC_PRIORITYGROUP_4`).
   - Check and configure the preemption priorities:

| Interrupt Table Entry | Interrupt Enabled | Preemption Priority | Sub Priority |
| :--- | :---: | :---: | :---: |
| **TIM1 trigger and commutation and TIM11** | Enabled | `15` | `0` |
| **DMA2 stream 0 global interrupt (ADC1)** | Enabled | `5` | `0` |
| **DMA2 stream 2 global interrupt (USART1_RX)** | Enabled | `5` | `0` |
| **DMA2 stream 7 global interrupt (USART1_TX)** | Enabled | `5` | `0` |
| **USART1 global interrupt** | Enabled | `5` | `0` |

> [!IMPORTANT]
> **FreeRTOS ISR Priority Rule:**  
> In FreeRTOS CMSIS_V2 on Cortex-M4, any peripheral ISR that invokes FreeRTOS API functions (`FromISR`) MUST have a preemption priority between `5` and `15` (inclusive). Priority values below 5 (0 to 4) are reserved for unmasked system exceptions and must never invoke FreeRTOS primitives.

---

## 8. FreeRTOS (CMSIS_V2) Real-Time Operating System Setup

### 8.1. Kernel Parameters & 32 KB Memory Heap Allocation

1. In the left navigation panel, expand **Middleware and Software Packs** $\to$ **FREERTOS**.
2. In the Mode pane, click the **Interface** dropdown and select **CMSIS_V2**.
3. In the lower pane, click the **Config parameters** tab:

| Parameter Category | Configuration Parameter | Setting Value | Architectural Rationale |
| :--- | :--- | :--- | :--- |
| **Memory management** | `TOTAL_HEAP_SIZE` | `32768` Bytes | Exactly 32 KB allocated for RTOS heap |
| | `Memory Allocation` | `Dynamic` | Allocates tasks, queues, semaphores dynamically |
| **Kernel Settings** | `TICK_RATE_HZ` | `1000` | 1 ms FreeRTOS scheduling timebase |
| | `MAX_PRIORITIES` | `56` | CMSIS_V2 priority resolution |
| | `MINIMAL_STACK_SIZE` | `128` Words | Minimal stack allocated for idle tasks (512 B) |
| | `USE_PREEMPTION` | `Enabled` | Preemptive task scheduling |
| | `USE_TIME_SLICING` | `Enabled` | Round-robin time slicing among equal priorities |
| | `IDLE_SHOULD_YIELD` | `Enabled` | Idle task yields immediately on ready task |
| | `CHECK_FOR_STACK_OVERFLOW` | `Option 2` | Pattern check on stack boundary |

### 8.2. Task Allocation: Task_Detector, Task_Gripper, Task_MicroROS

Switch to the **Tasks and Queues** tab. Delete the default task or configure the three dedicated production tasks:

```
┌────────────────────────────────────────────────────────────────────────────────────────┐
│                               FreeRTOS Tasks Hierarchy                                 │
├───────────────────┬──────────────┬─────────────┬─────────────────┬─────────────────────┤
│ Task Name         │ Priority     │ Stack Size  │ Entry Function  │ Responsibility      │
├───────────────────┼──────────────┼─────────────┼─────────────────┼─────────────────────┤
│ Task_Detector     │ High (40)    │ 512 Words   │ StartTaskDetect │ Pulse coil & ADC    │
│ Task_Gripper      │ AboveNorm(32)│ 256 Words   │ StartTaskGrip   │ 3-channel PWM servo │
│ Task_MicroROS     │ Normal (24)  │ 1024 Words  │ StartTaskMicro  │ ROS 2 spin & pub    │
└───────────────────┴──────────────┴─────────────┴─────────────────┴─────────────────────┘
```

#### Detailed Task Configuration Steps:

1. **Task 1: Task_Detector**
   - Click **Add** in the Tasks table.
   - **Task Name:** `Task_Detector`
   - **Priority:** `osPriorityHigh` (Numerical value: `40`)
   - **Stack Size (Words):** `512` (equals $2048$ Bytes)
   - **Entry Function:** `StartTaskDetector`
   - **Code Generation Option:** `Default`
   - **Allocation:** `Dynamic`

2. **Task 2: Task_Gripper**
   - Click **Add** in the Tasks table.
   - **Task Name:** `Task_Gripper`
   - **Priority:** `osPriorityAboveNormal` (Numerical value: `32`)
   - **Stack Size (Words):** `256` (equals $1024$ Bytes)
   - **Entry Function:** `StartTaskGripper`
   - **Code Generation Option:** `Default`
   - **Allocation:** `Dynamic`

3. **Task 3: Task_MicroROS**
   - Click **Add** in the Tasks table.
   - **Task Name:** `Task_MicroROS`
   - **Priority:** `osPriorityNormal` (Numerical value: `24`)
   - **Stack Size (Words):** `1024` (equals $4096$ Bytes)
   - **Entry Function:** `StartTaskMicroROS`
   - **Code Generation Option:** `Default`
   - **Allocation:** `Dynamic`

---

## 9. Project Manager Settings & Zero-Code-Deletion Strategy

### 9.1. Project Properties & Toolchain Selection

Click the **Project Manager** tab in the top navigation ribbon.

#### In the "Project" Sub-Tab:
- **Project Name:** `STM32_Auxiliary`
- **Project Location:** `e:/MineSweeper/STM32_Auxiliary`
- **Application Structure:** `Basic`
- **Toolchain / IDE:** Select `STM32CubeIDE`
- **Generate Under Root:** Checked
- **Linker Settings:**
  - **Minimum Heap Size:** `0x200` (512 Bytes)
  - **Minimum Stack Size:** `0x400` (1024 Bytes)

### 9.2. Code Generator Settings for Firmware Preservation

Click the **Code Generator** sub-tab on the left. This setup is crucial to avoid overwriting custom drivers and micro-ROS libraries during subsequent code regenerations.

| Setting Section | Checkbox Name | Required State | Critical Rationale |
| :--- | :--- | :---: | :--- |
| **STM32Cube MCU packages** | Copy only the necessary library files | **SELECTED** | Keeps workspace lean |
| **Generated files** | **Keep User Code when re-generating** | **CHECKED** | **MANDATORY:** Preserves all code within `USER CODE` tags |
| | **Delete previously generated files when not re-generated** | **UNCHECKED** | **MANDATORY:** Prevents deletion of unreferenced files |
| | **Generate peripheral initialization as a pair of '.c/.h' files per peripheral** | **CHECKED** | **MANDATORY:** Generates `adc.c/.h`, `tim.c/.h`, `usart.c/.h`, `gpio.c/.h` |
| | Backup previously generated files when re-generating | Unchecked | Optional |
| | Set all free pins as analog | Unchecked | Keeps spare test headers uncorrupted |

> [!CAUTION]
> If **"Keep User Code when re-generating"** is unchecked or **"Delete previously generated files"** is checked, regenerating code from CubeMX will destroy driver code in `Core/BSP` and wipe custom FreeRTOS task hooks. Double check these settings before clicking **Generate Code**!

---

## 10. Step-by-Step Validation & Hardware Verification Checklist

Follow this checklist to systematically verify the CubeMX configuration and physical hardware integration:

### 1. Clock & System Core Validation
- [ ] In STM32CubeIDE, compile the generated firmware (`Project -> Build Project` or `Ctrl+B`). Verify **0 Errors, 0 Warnings**.
- [ ] Connect ST-Link SWD to PA13 (SWDIO), PA14 (SWCLK), GND, and 3.3V.
- [ ] Launch debug session. Inspect the `SystemCoreClock` global variable in the Expressions window; verify it reads exactly `96000000`.
- [ ] Verify `HAL_GetTick()` increments accurately by 1000 counts per second using `TIM11`.

### 2. Metal Detector Hardware Stage (PB0 & PA1)
- [ ] Connect an oscilloscope channel 1 to **PB0** (`AUX_PULSE_OUT`).
- [ ] Verify the excitation gate pulse width measures $70.0\,\mu\text{s} \pm 0.5\,\mu\text{s}$ at a repetition frequency of $1.0\,\text{kHz}$.
- [ ] Verify the rise time on the IRF740 MOSFET gate is $\le 20\,\text{ns}$.
- [ ] Connect oscilloscope channel 2 to **PA1** (`AUX_DECAY_IN`).
- [ ] Verify that ADC sampling commences $20\,\mu\text{s}$ after the falling edge of PB0 (blanking window complete).
- [ ] Confirm DMA2 Stream 0 fills the halfword decay array without DMA FIFO errors or overrun flags.

### 3. Manipulator Servos Stage (PB6, PB7, PB8)
- [ ] Connect a logic analyzer to **PB6**, **PB7**, and **PB8**.
- [ ] Measure the PWM waveform period: verify it reads exactly $20.0\,\text{ms}$ ($50.0\,\text{Hz}$).
- [ ] Test the initial pulse duration: verify it reads $1500.0\,\mu\text{s}$ (Neutral position).
- [ ] Test command bounds:
  - Command Open ($0^\circ$): pulse duration equals $1000.0\,\mu\text{s}$ ($1.0\,\text{ms}$).
  - Command Clamped ($180^\circ$): pulse duration equals $2000.0\,\mu\text{s}$ ($2.0\,\text{ms}$).

### 4. Diagnostic Indicators (PB12, PB13, PB14, PB15, PC13)
- [ ] Toggle **PC13** (`HEARTBEAT_LED`): verify onboard blue LED blinks at 2 Hz (toggling between Open-Drain Low and High).
- [ ] Assert **PB12** (`DEBUG_BUZZER`): verify 5V siren drives cleanly without resetting the 3.3V microcontroller rail.
- [ ] Assert **PB13** (`DEBUG_LED1_RED`): verify Red LED illuminates.
- [ ] Assert **PB14** (`DEBUG_LED2_YELLOW`): verify Yellow LED illuminates.
- [ ] Assert **PB15** (`DEBUG_LED3_GREEN`): verify Green LED illuminates.

### 5. micro-ROS Serial Link (PA9, PA10 @ 921,600 Baud)
- [ ] Connect BlackPill PA9 (TX) to Raspberry Pi 5 UART RX and PA10 (RX) to Raspberry Pi 5 UART TX. Connect common GND.
- [ ] On the Raspberry Pi 5, launch the micro-ROS agent:
  ```bash
  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyAMA0 -b 921600
  ```
- [ ] Verify successful handshake: agent outputs `[micro_ros_agent] Client connected with session ID...`.
- [ ] Run ROS 2 topic echoes:
  ```bash
  ros2 topic list
  ros2 topic echo /metal_detector/reading
  ros2 topic echo /gripper/status
  ```
- [ ] Publish a test servo command:
  ```bash
  ros2 topic pub --once /gripper/command std_msgs/msg/Int32 "{data: 2}"
  ```
  Verify that PB6, PB7, and PB8 shift to $2000\,\mu\text{s}$ clamping pulses and PB14 (Yellow LED) strobes during movement.
