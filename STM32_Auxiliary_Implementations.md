# STM32_Auxiliary Implementation Guide & IOC Configuration Manual

**Project Directory:** `STM32_Auxiliary`  
**Target Hardware:** STM32 BlackPill (STM32F411CEU6 - ARM Cortex-M4 @ 96 MHz)  
**Operating System:** FreeRTOS (CMSIS_V2 API) + micro-ROS Client over UART DMA @ 921600 Baud  
**Payload Allocation:** 3x 50 Hz PWM Servos, 2-Pin Pulse Induction Detector, 3x Status LEDs, 1x Audible Siren  

---

## 1. Subsystem Architecture & Responsibilities

The `STM32_Auxiliary` microcontroller functions as the dedicated payload controller for the Autonomous Minesweeper Rover. It handles high-speed pulse excitation and decay tail acquisition for landmine detection, articulates a 3-servo robotic payload/gripper mechanism, manages field judge alarm beacons, and publishes telemetry to ROS 2 Humble on the Raspberry Pi 5.

```
                               ┌────────────────────────────────────────────────────────┐
                               │           Host SBC: Raspberry Pi 5 (ROS 2 Humble)      │
                               │  • micro_ros_agent serial node @ 921600 Baud           │
                               │  • Topics: /metal_detector/reading, /gripper/status    │
                               │  • Subscriptions: /gripper/command, /beacon/alert      │
                               └───────────────────────────┬────────────────────────────┘
                                                           │ USART1 DMA (PA9 / PA10)
                                                           ▼
                               ┌────────────────────────────────────────────────────────┐
                               │                  STM32_Auxiliary ECU                   │
                               │               (STM32F411CEU6 BlackPill)                │
                               ├────────────────────────────────────────────────────────┤
                               │ • 3x 50 Hz RC Servos (TIM4 CH1, CH2, CH3)              │
                               │   - PB6: Servo 1 (Gripper Jaw 1 / Base)                │
                               │   - PB7: Servo 2 (Gripper Jaw 2 / Wrist)               │
                               │   - PB8: Servo 3 (Payload / Aux Axis)                  │
                               │ • 2-Pin Pulse Induction Metal Detector:                │
                               │   - PB0: IRF740 MOSFET Gate (70 µs pulse @ 1 kHz)      │
                               │   - PA1: TL072 Decay Sampler (ADC1_IN1 after 20 µs)    │
                               │ • 3x Visual Status LEDs:                               │
                               │   - PB13: LED 1 Alert / Strobe (Red)                   │
                               │   - PB14: LED 2 Gripper Active (Yellow)                │
                               │   - PB15: LED 3 System Ready / ROS Linked (Green)      │
                               │ • 1x Audible Alarm Buzzer:                             │
                               │   - PB12: NPN Siren Drive (Active-HIGH)                │
                               └────────────────────────────────────────────────────────┘
```

---

## 2. Hardware Pinout Allocation Table

| Peripheral Group | STM32 Pin | Mode / Alternate Function | Signal / Net Name | Connected Hardware & Function |
| :--- | :--- | :--- | :--- | :--- |
| **3 Servos (50 Hz PWM)** | **PB6** | `TIM4_CH1` (AF2) | `AUX_SERVO1_PWM` | Servo 1: Gripper Base / Jaw 1 ($1.0..2.0\,\text{ms}$) |
| | **PB7** | `TIM4_CH2` (AF2) | `AUX_SERVO2_PWM` | Servo 2: Wrist / Jaw 2 ($1.0..2.0\,\text{ms}$) |
| | **PB8** | `TIM4_CH3` (AF2) | `AUX_SERVO3_PWM` | Servo 3: Payload / Aux Axis ($1.0..2.0\,\text{ms}$) |
| **2 Metal Detector Pins**| **PB0** | `GPIO_Output` (High Speed)| `AUX_PULSE_OUT` | IRF740 MOSFET Gate Driver ($70\,\mu\text{s}$ excitation pulse) |
| | **PA1** | `ADC1_IN1` (Analog) | `AUX_DECAY_IN` | TL072 Op-Amp Decay Tail (Sampled after $20\,\mu\text{s}$ blanking) |
| **3 Status LEDs** | **PB13**| `GPIO_Output` (Push-Pull) | `AUX_LED1_ALERT` | LED 1: Mine Alert / Strobe (Red) |
| | **PB14**| `GPIO_Output` (Push-Pull) | `AUX_LED2_PAYLOAD`| LED 2: Gripper Actuation Active (Yellow) |
| | **PB15**| `GPIO_Output` (Push-Pull) | `AUX_LED3_READY` | LED 3: System Ready & ROS Linked (Green) |
| **1 Audible Buzzer** | **PB12**| `GPIO_Output` (Push-Pull) | `AUX_BUZZER_OUT` | Transistor Base for Audible Siren Alarm |
| **micro-ROS Serial Link**| **PA9** | `USART1_TX` (AF7) | `MICROROS_TX` | Transmit to RPi5 (DMA2 Stream 7) @ 921600 Baud |
| | **PA10**| `USART1_RX` (AF7) | `MICROROS_RX` | Receive from RPi5 (DMA2 Stream 2) @ 921600 Baud |
| **Debug & Diagnostics** | **PC13**| `GPIO_Output` (Open-Drain)| `AUX_HEARTBEAT` | Onboard BlackPill User LED (Toggled at 2 Hz) |
| | **PA13**| `SYS_JTMS-SWDIO` (AF0) | `SWDIO` | ST-Link Debug Data |
| | **PA14**| `SYS_JTCK-SWCLK` (AF0) | `SWCLK` | ST-Link Debug Clock |

---

## 3. Zero-Code-Deletion Strategy for CubeMX Code Generation

When opening `STM32_Auxiliary.ioc` directly in STM32CubeMX or STM32CubeIDE to make changes or generate code, your custom firmware is 100% safeguarded by three policies:

### 1. Code Retention Setting in `.ioc`
In the **Project Manager** tab $\to$ **Code Generator**:
- Ensure **"Keep User Code when re-generating"** is **CHECKED** (`ProjectManager.KeepUserCode=true`).
- Ensure **"Delete previously generated files when not re-generated"** is **UNCHECKED** (`ProjectManager.DeletePrevious=false`).

### 2. User Code Sandboxing in `Core/Src/main.c`
All inclusions, peripheral initializations, driver setups, and task creations are placed exclusively within official ST comments:
```c
/* USER CODE BEGIN Includes */
#include "auxiliary_pin_config.h"
#include "status_beacon_driver.h"
#include "servo_gripper_driver.h"
#include "pi_detector_driver.h"
#include "task_detector.h"
#include "task_gripper.h"
#include "task_microros.h"
/* USER CODE END Includes */

/* USER CODE BEGIN 2 */
Status_Beacon_Init();
Servo_Gripper_Init();
PI_Detector_Init();
MicroROS_Transport_Init(&huart1, &hdma_usart1_rx, &hdma_usart1_tx);
MicroROS_Client_Init();
Task_Detector_Init();
Task_Gripper_Init();
Task_MicroROS_Init();
/* USER CODE END 2 */
```
STM32CubeMX guarantees that any code inside `USER CODE BEGIN ...` and `USER CODE END ...` is untouched during code regeneration.

### 3. Decoupled Modular Folders
All concrete driver implementations and FreeRTOS tasks live in modular directories:
- `Core/BSP/`: `servo_gripper_driver.c/.h`, `pi_detector_driver.c/.h`, `status_beacon_driver.c/.h`
- `Core/Tasks/`: `task_detector.c/.h`, `task_gripper.c/.h`, `task_microros.c/.h`
- `Core/Middleware/`: `microros_client.c/.h`, `microros_transport.c/.h`

CubeMX only overwrites code-generator files (`main.c`, `stm32f4xx_it.c`, `freertos.c`); it never deletes files in `Core/BSP/` or `Core/Tasks/`.

---

## 4. Step-by-Step STM32CubeMX GUI Recipe for `STM32_Auxiliary.ioc`

1. **System Clock & Timebase:**
   - **SYS:** Timebase Source = `TIM11` (to prevent FreeRTOS SysTick priority collisions). Debug = `Serial Wire`.
   - **RCC:** High Speed Clock (HSE) = `Crystal/Ceramic Resonator` ($25\,\text{MHz}$).
   - **Clock Configuration:** PLL Source = `HSE`, $/M = 25$, $*N = 192$, $/P = 2 \implies \mathbf{96\,\text{MHz}}$ SYSCLK.

2. **3x 50 Hz Servos (TIM4 CH1, CH2, CH3):**
   - Expand **Timers** $\to$ **TIM4**.
   - Clock Source: `Internal Clock`.
   - **Channel 1**: `PWM Generation CH1` (`PB6`).
   - **Channel 2**: `PWM Generation CH2` (`PB7`).
   - **Channel 3**: `PWM Generation CH3` (`PB8`).
   - Parameter Settings:
     * **Prescaler (PSC)**: `95` ($96\,\text{MHz} / 96 = 1\,\text{MHz}$ tick resolution).
     * **Counter Period (ARR)**: `19999` ($20\,\text{ms}$ period $\implies 50\,\text{Hz}$).
     * **Auto-reload preload**: `Enable`.
     * **CH1 / CH2 / CH3 Pulse**: `1500` (Neutral position).

3. **Metal Detector 2 Pins (PB0 Pulse Out & PA1 ADC1 In):**
   - **PA1 (Decay Sampler)**: Expand **Analog** $\to$ **ADC1** $\to$ check `IN1`.
     * Clock Prescaler: `PCLK2 divided by 4`.
     * Resolution: `12 bits`.
     * Sampling Time: `15 Cycles`.
   - **PB0 (Gate Excitation Pulse)**: Left-click `PB0` $\to$ `GPIO_Output`.
     * Mode: `Output Push Pull`.
     * Maximum output speed: `Very High`.
     * User Label: `AUX_PULSE_OUT`.

4. **1 Buzzer & 3 Status LEDs (PB12, PB13, PB14, PB15):**
   - **PB12**: `GPIO_Output`, Push Pull, Low Speed, User Label: `AUX_BUZZER_OUT`.
   - **PB13**: `GPIO_Output`, Push Pull, Low Speed, User Label: `AUX_LED1_ALERT` (Mine Alert / Strobe, Red).
   - **PB14**: `GPIO_Output`, Push Pull, Low Speed, User Label: `AUX_LED2_PAYLOAD` (Gripper Active, Yellow).
   - **PB15**: `GPIO_Output`, Push Pull, Low Speed, User Label: `AUX_LED3_READY` (System Ready / ROS Linked, Green).
   - **PC13**: `GPIO_Output`, Open Drain, Low Speed, User Label: `AUX_HEARTBEAT_LED`.

5. **micro-ROS Transport (USART1 DMA):**
   - Expand **Connectivity** $\to$ **USART1**.
   - Mode: `Asynchronous` on `PA9` (TX) and `PA10` (RX).
   - Baud Rate: `921600` (or `115200`), 8N1.
   - DMA Settings:
     * Add `USART1_RX`: Stream `DMA2_Stream2`, Mode `Circular`, Priority `Very High`.
     * Add `USART1_TX`: Stream `DMA2_Stream7`, Mode `Normal`, Priority `Medium`.

6. **FreeRTOS (CMSIS_V2):**
   - Total Heap Size: `65536` ($64\,\text{KB}$).
   - Tasks:
     * `Task_Detector`: Priority `osPriorityHigh` (40), Stack 512 words.
     * `Task_Gripper`: Priority `osPriorityAboveNormal` (40), Stack 256 words.
     * `Task_MicroROS`: Priority `osPriorityNormal` (24), Stack 1024 words.

---

## 5. Verification & Testing

Run the automated test suite from the workspace root:

```bash
pytest tests/
```
