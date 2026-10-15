> **Current target: STM32F103C8T6 Blue Pill, one servo.** See [BLUE_PILL.md](BLUE_PILL.md) for active configuration, builds, wiring and ROS interfaces. The Black Pill instructions below are historical and must not be used to flash the current firmware.

# STM32_Auxiliary ECU: Engineering Documentation & Subsystem Manual

**Microcontroller:** STM32 BlackPill ([STM32F411CEU6](file:///e:/MineSweeper/STM32_Auxiliary/Core/Inc/auxiliary_pin_config.h)) — ARM Cortex-M4F @ 96 MHz  
**Operating System:** FreeRTOS (CMSIS-RTOS v2) + micro-ROS Client  
**Host Interface:** Raspberry Pi 5 (ROS 2 Humble) via USART1 DMA @ 921,600 Baud  
**Hardware Configuration Header:** [`auxiliary_pin_config.h`](file:///e:/MineSweeper/STM32_Auxiliary/Core/Inc/auxiliary_pin_config.h)  
**Implementation Guide:** [`STM32_Auxiliary_Implementations.md`](file:///e:/MineSweeper/STM32_Auxiliary/STM32_Auxiliary_Implementations.md)

---

## 1. Executive Overview & System Role

The **`STM32_Auxiliary`** electronic control unit (ECU) operates as the dedicated real-time payload coprocessor for the Autonomous Minesweeper Rover. In conjunction with the primary mobility controller (`STM32_Mobility`), the Auxiliary ECU decouples timing-critical analog sensor acquisition, high-voltage coil pulse timing, and multi-axis mechanical articulation from the Linux-based master SBC (Raspberry Pi 5) and the differential-drive kinematics engine.

```
┌────────────────────────────────────────────────────────────────────────────────────────┐
│                        Host SBC: Raspberry Pi 5 (ROS 2 Humble)                         │
│  • micro_ros_agent serial gateway @ 921,600 Baud (/dev/ttyUSB_STM_AUX)                 │
│  • Autonomous Navigation & SLAM (Nav2, Costmap 2D, Boustrophedon Coverage)            │
│  • Surface Mine Vision Processing & Competition Grid Coordinate Exporter               │
└───────────────────────────────────────────┬────────────────────────────────────────────┘
                                            │ Full-Duplex UART DMA (PA9 / PA10)
                                            ▼
┌────────────────────────────────────────────────────────────────────────────────────────┐
│                              STM32_Auxiliary Payload ECU                               │
│                         (STM32F411CEU6 BlackPill @ 96 MHz)                             │
├────────────────────────────────────────────────────────────────────────────────────────┤
│  [Pulse Induction Metal Detector]                                                      │
│    • PB0 (Fast GPIO): 70 µs excitation pulse driving IRF740 power MOSFET gate          │
│    • PA1 (ADC1_IN1): 20 µs blanking window + 8x oversampled TL072 decay tail           │
│    • Exponential moving average (EMA) baseline subtraction & anomaly thresholding      │
│                                                                                        │
│  [3-Axis Payload & Gripper Articulation]                                               │
│    • PB6 (TIM4_CH1): Base / Jaw 1 PWM (50 Hz, 1.0 - 2.0 ms pulse)                      │
│    • PB7 (TIM4_CH2): Wrist / Jaw 2 PWM (50 Hz, 1.0 - 2.0 ms pulse)                     │
│    • PB8 (TIM4_CH3): Payload / Aux Axis PWM (50 Hz, 1.0 - 2.0 ms pulse)                │
│                                                                                        │
│  [Visual Status Beacons & Audible Siren]                                               │
│    • PB12: NPN Transistor stage driving 105 dB competition field alert buzzer/siren     │
│    • PB13 (Red): High-intensity Mine Alert / Strobe LED (local detect || ROS alert)    │
│    • PB14 (Yellow): Payload Actuation / Gripper Busy LED                               │
│    • PB15 (Green): System Ready & micro-ROS Link Established LED                       │
│    • PC13 (Green/Blue): Onboard BlackPill User LED (FreeRTOS scheduler heartbeat)      │
└────────────────────────────────────────────────────────────────────────────────────────┘
```

### Key Architectural Responsibilities
1. **Pulse Induction Landmine Sensing:** Generates sub-microsecond deterministic pulse sequences ($70\,\mu\text{s}$ excitation, $20\,\mu\text{s}$ flyback blanking) and executes high-speed oversampling of eddy-current decay responses without jitter.
2. **3-Axis Robotic Gripper Articulation:** Controls three independent 50 Hz PWM channels with $1\,\mu\text{s}$ edge resolution, providing reliable surface-mine pick-and-place capabilities.
3. **Audible & Visual Field Judge Indicators:** Drives dual-trigger alert beacons (audible siren and red strobe) in compliance with international minefield competition safety rules.
4. **Deterministic micro-ROS Serial Telemetry:** Integrates directly into the ROS 2 Humble node graph using USART1 DMA ring buffers at 921,600 baud, completely avoiding CPU context-switch bottlenecks.

---

## 2. Complete Pinout Allocation Table

The STM32F411CEU6 BlackPill pin allocation is centrally managed in [`auxiliary_pin_config.h`](file:///e:/MineSweeper/STM32_Auxiliary/Core/Inc/auxiliary_pin_config.h). All signals, alternate functions, electrical specifications, and connected hardware subsystems are detailed below:

| Peripheral Group | STM32 Pin | Mode / AF | Net / Signal Name | Electrical Specs | Connected Hardware & Subsystem Function |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Pulse Induction Metal Detector** | **`PB0`** | `GPIO_Output` | `AUX_PULSE_OUT` | 3.3V CMOS, Push-Pull, Very High Speed, $\le 25\,\text{mA}$ | Drives gate of IRF740 N-channel MOSFET via totem-pole BJT driver ($70\,\mu\text{s}$ gate pulse @ $1\,\text{kHz}$) |
| | **`PA1`** | `ADC1_IN1` | `AUX_DECAY_IN` | Analog Input ($0.0\,\text{V} - 3.3\,\text{V}$), $Z_{\text{in}} > 50\,\text{k}\Omega$ | Low-noise TL072 op-amp decay tail output; sampled after $20\,\mu\text{s}$ flyback blanking window |
| **3-Axis Payload Gripper** | **`PB6`** | `TIM4_CH1` (AF2) | `AUX_SERVO1_PWM` | 3.3V PWM, 50 Hz, $1.0 - 2.0\,\text{ms}$, High Speed | Servo 1: Gripper Base / Jaw 1 ($1000\,\mu\text{s}$ Open $\leftrightarrow 2000\,\mu\text{s}$ Grip) |
| | **`PB7`** | `TIM4_CH2` (AF2) | `AUX_SERVO2_PWM` | 3.3V PWM, 50 Hz, $1.0 - 2.0\,\text{ms}$, High Speed | Servo 2: Gripper Wrist / Jaw 2 ($1000\,\mu\text{s}$ Open $\leftrightarrow 2000\,\mu\text{s}$ Grip) |
| | **`PB8`** | `TIM4_CH3` (AF2) | `AUX_SERVO3_PWM` | 3.3V PWM, 50 Hz, $1.0 - 2.0\,\text{ms}$, High Speed | Servo 3: Payload / Elevation Axis ($1000\,\mu\text{s} - 2000\,\mu\text{s}$) |
| **Debug Indicators & Beacons** | **`PB12`** | `GPIO_Output` | `AUX_BUZZER_OUT` | 3.3V Push-Pull, Low Speed | Base of 2N2222/2N3904 NPN transistor switching $+5\text{V}/+12\text{V}$ Audible Siren |
| | **`PB13`** | `GPIO_Output` | `AUX_LED1_ALERT` | 3.3V Push-Pull, $330\,\Omega$ series resistor, $\sim 4\,\text{mA}$ | **LED 1 (Red)**: Mine Alert Strobe & Diagnostic Panic Beacon (10 Hz toggle during alert) |
| | **`PB14`** | `GPIO_Output` | `AUX_LED2_PAYLOAD`| 3.3V Push-Pull, $330\,\Omega$ series resistor, $\sim 4\,\text{mA}$ | **LED 2 (Yellow)**: Payload Actuation Active (Illuminated when servos are commanded/moving) |
| | **`PB15`** | `GPIO_Output` | `AUX_LED3_READY` | 3.3V Push-Pull, $330\,\Omega$ series resistor, $\sim 4\,\text{mA}$ | **LED 3 (Green)**: System Ready & micro-ROS Link Established (Solid ON when agent ping succeeds) |
| | **`PC13`** | `GPIO_Output` | `AUX_HEARTBEAT` | Active-LOW, Open-Drain / Push-Pull | Onboard BlackPill User LED (Toggled at 1 Hz by [`Task_Detector`](file:///e:/MineSweeper/STM32_Auxiliary/Core/Tasks/task_detector.c)) |
| **micro-ROS Serial Link** | **`PA9`** | `USART1_TX` (AF7) | `MICROROS_TX` | 3.3V LVTTL UART, High Speed | Transmit line to RPi5 CP2102/FTDI RX (DMA2 Stream 7, Normal Mode, 921,600 Baud) |
| | **`PA10`** | `USART1_RX` (AF7) | `MICROROS_RX` | 3.3V LVTTL UART, Internal Pull-Up | Receive line from RPi5 CP2102/FTDI TX (DMA2 Stream 2, Circular Mode, 921,600 Baud) |
| **SWD Debug & Flashing** | **`PA13`** | `SYS_JTMS-SWDIO` | `SWDIO` | 3.3V Bidirectional Data Line | ST-Link v2 Serial Wire Debug Data |
| | **`PA14`** | `SYS_JTCK-SWCLK` | `SWCLK` | 3.3V Clock Line, Internal Pull-Down | ST-Link v2 Serial Wire Debug Clock |
| | **`NRST`** | `System Reset` | `NRST` | Active-LOW Reset, $10\,\text{k}\Omega$ pull-up to 3.3V | External hardware reset button / ST-Link reset pin |
| **Power Supply Rails** | **`3V3`** | Power Input/Output | `+3V3` | 3.3V DC Regulated ($\pm 5\%$) | Regulated MCU logic rail (BlackPill onboard ME6211 LDO) |
| | **`5V`** | Power Input | `+5V_VBUS` | 5.0V DC Input from USB or SBC | BlackPill LDO input rail |
| | **`GND`** | Ground Reference | `GND` | Common 0V Reference | Common system ground shared across MCU, UBEC, and RPi5 |

---

## 3. Electrical Interface & Hardware Schematics

### 3.1. Pulse Induction Metal Detector Front-End

The pulse induction (PI) front-end energizes a search coil with high pulsed current. When the magnetic field abruptly collapses, eddy currents induced within buried metallic objects prolong the decay tail of the flyback voltage.

```
       +12V Power Rail
             │
             ├───[ Coil: ~400 µH, 2.5 Ω ]───┬───────────────────────────────────────────────┐
             │                              │ (Flyback Node)                                │
             │                             [ ] R_damp (470 Ω)                               │
             │                              │                                               │
             │                              ├───[ 1N4148 / UF4007 Clamping Diodes ]───┐     │
             │                              │                                         │     │
             │                              ▼ Drain                                   ▼     │
     PB0 ────┤ BJT Gate Driver ├──────► [ IRF740 MOSFET ]                             ▲     │
             │ (Totem-Pole NPN/PNP)         │ Source                                  │     │
             │                              └───┬───► GND                             └──┬──┘
             │                                  │                                        │
             │                                 GND                       High-Speed Clamp│(±0.7V)
             │                                                                           ▼
             │                                                          TL072 Non-Inverting Op-Amp
             │                                                          ┌───────────────────────────┐
             │                                                          │ V+ = +5V, V- = GND        │
             │                                                          │ Gain = 1 + (R_f / R_in)   │
             │                                                          └─────────────┬─────────────┘
             │                                                                        │
             └────────────────────────────────────────────────────────────────────────┼──► PA1 (ADC1_IN1)
                                                                                      │    (Sampled after
                                                                                     GND    20 µs blanking)
```

#### Theory of Operation & Decay Mathematics
1. **Coil Excitation Phase ($70\,\mu\text{s}$):**
   `PB0` drives the gate of an **IRF740** power MOSFET ($V_{DS} = 400\,\text{V}, I_D = 10\,\text{A}, R_{DS(\text{on})} = 0.55\,\Omega$). The search coil ($L \approx 400\,\mu\text{H}, R \approx 2.5\,\Omega$) is connected to $+12\,\text{V}$. During the $70\,\mu\text{s}$ on-time, the current ramps linearly:
   $$I_{\text{peak}} = \frac{V_{\text{batt}}}{R} \left(1 - e^{-\frac{R \cdot t_{\text{pulse}}}{L}}\right) \approx \frac{12\,\text{V}}{2.5\,\Omega} \left(1 - e^{-\frac{2.5 \cdot 70 \cdot 10^{-6}}{400 \cdot 10^{-6}}}\right) \approx 1.7\,\text{A}$$

2. **Flyback & Damping Phase ($20\,\mu\text{s}$ Blanking):**
   When `PB0` pulls LOW, the MOSFET shuts off abruptly. The magnetic field collapse produces a counter-EMF flyback spike:
   $$V_{\text{flyback}} = -L \frac{dI}{dt} \approx 200\,\text{V} - 350\,\text{V}$$
   A damping resistor $R_{\text{damp}} \approx 470\,\Omega$ in parallel with the coil prevents underdamped oscillation. The back-to-back diode clamp network clips the voltage presented to the operational amplifier to within safe operational limits ($\pm 0.7\,\text{V}$).

3. **Decay Tail Signal Conditioning:**
   A low-noise **TL072** JFET-input op-amp amplifies the residual decay tail. The decay voltage is modeled as:
   $$V(t) = V_{\text{soil}} e^{-\frac{t}{\tau_{\text{soil}}}} + V_{\text{target}} e^{-\frac{t}{\tau_{\text{metal}}}}$$
   Because conductive metals produce eddy currents with $\tau_{\text{metal}} \gg \tau_{\text{soil}}$, sampling the decay tail after a $20\,\mu\text{s}$ blanking window isolates the metallic signature.
   
4. **8x Rapid ADC1 Oversampling:**
   [`PI_Detector_TriggerPulseAndSample()`](file:///e:/MineSweeper/STM32_Auxiliary/Core/BSP/pi_detector_driver.c) acquires 8 consecutive ADC conversions across $25\,\mu\text{s} - 60\,\mu\text{s}$, computes the mean, and applies an Exponential Moving Average (EMA) filter ($\alpha = 0.02$) to adapt to ground mineralisation:
   $$V_{\text{baseline}}[k] = \alpha \cdot V_{\text{sample}}[k] + (1 - \alpha) \cdot V_{\text{baseline}}[k-1]$$
   $$\Delta V = V_{\text{sample}} - V_{\text{baseline}}$$
   When $\Delta V \ge \text{Threshold}$ ($300\,\text{LSB}$), a landmine is flagged and the baseline update freezes to avoid absorbing the target signal.

---

### 3.2. 3-Axis Gripper Servo Connections & Power Topology

RC servos draw severe transient current spikes during acceleration and stall conditions ($1.5\,\text{A} - 2.5\,\text{A}$ per servo). **Servos must NEVER be powered from the STM32 3.3V or 5V VBUS rail.**

```
                                  +12V Main Battery Rail
                                            │
                                            ▼
                           ┌─────────────────────────────────┐
                           │      External 5V / 6V UBEC      │
                           │   (High Efficiency Switching)   │
                           │     Continuous Current: ≥ 5A    │
                           └────────┬──────────────┬─────────┘
                                    │ +5V          │ GND (Power Ground)
                                    │              │
                   ┌────────────────┼──────────────┼────────────────┐
                   │ Servo 1 Power  │ Servo 2 Power│ Servo 3 Power  │
                   ▼                ▼              ▼                │
            ┌─────────────┐  ┌─────────────┐  ┌─────────────┐       │
            │ Servo 1: Jaw│  │ Servo 2:    │  │ Servo 3:    │       │
            │   (MG996R)  │  │   Wrist     │  │   Elevation │       │
            └──────┬──────┘  └──────┬──────┘  └──────┬──────┘       │
                   │ Signal         │ Signal         │ Signal       │
                   │ (PB6)          │ (PB7)          │ (PB8)        │
                   │                │                │              │
                   │ TIM4_CH1       │ TIM4_CH2       │ TIM4_CH3     │
       ┌───────────┴────────────────┴────────────────┴──────────────┴───────────┐
       │                       STM32_Auxiliary ECU                              │
       │                   Common Star Ground Reference                         │
       └────────────────────────────────────────────────────────────────────────┘
```

#### Ground Referencing Rule
* Connect the **UBEC Ground**, **Battery Ground**, and **STM32 Board Ground** together at a single **Star Ground Point**.
* Floating grounds will result in servo chatter, lost PWM frames, or erratic motion.
* The $3.3\,\text{V}$ PWM logic levels generated on `PB6`, `PB7`, and `PB8` directly satisfy the TTL high threshold ($V_{IH} \ge 2.0\,\text{V}$) of standard RC servos.

---

### 3.3. Debug Indicators & Buzzer Driver Circuits

#### Audible Siren / Buzzer Stage (`PB12`)
The BlackPill GPIO pins source up to $25\,\text{mA}$, which is insufficient to drive an audible $85 - 105\,\text{dB}$ competition siren. An external NPN transistor driver is required:

```
               +5V / +12V Rail
                     │
                     ├───[ Active Buzzer / Siren ]───┐
                     │                               │
                     ├───[ 1N4148 Flyback Diode ]────┤
                     │   (Cathode to +V, Anode to C) │
                     │                               ▼ Collector
      PB0 / PB12 ────┼───[ 1 kΩ Base Resistor ]───► [ 2N2222 / 2N3904 NPN ]
                     │                               │ Emitter
                    [ ] 100 kΩ Pull-Down             ▼
                     │ (Noise Immunity)             GND
                     ▼
                    GND
```

#### Diagnostic Status LEDs (`PB13`, `PB14`, `PB15`)
Each indicator LED is driven via a $330\,\Omega$ current-limiting resistor directly from a push-pull GPIO:
$$I_F = \frac{V_{OH} - V_F}{R_{\text{series}}} = \frac{3.3\,\text{V} - 2.0\,\text{V}}{330\,\Omega} \approx 3.94\,\text{mA}$$

```
    PB13 (Red - Mine Alert)       ────► [ 330 Ω ] ────► [ LED (Red) ]    ────► GND
    PB14 (Yellow - Gripper Move)  ────► [ 330 Ω ] ────► [ LED (Yellow) ] ────► GND
    PB15 (Green - System Ready)   ────► [ 330 Ω ] ────► [ LED (Green) ]  ────► GND
```

---

## 4. micro-ROS & ROS 2 Architecture

The STM32_Auxiliary firmware integrates into ROS 2 Humble via a dedicated micro-ROS client task ([`Core/Tasks/task_microros.c`](file:///e:/MineSweeper/STM32_Auxiliary/Core/Tasks/task_microros.c)) running over circular DMA USART1 at 921,600 baud.

### 4.1. ROS 2 Topic Specifications

```
                              ROS 2 Humble Node Graph
    ┌────────────────────────────────────────────────────────────────────────┐
    │  Host Master: /micro_ros_agent_auxiliary                               │
    └──────────────┬──────────────────────────────────────────▲──────────────┘
                   │                                          │
                   │ Subscribed by MCU                        │ Published by MCU
                   ▼                                          │
    ┌──────────────────────────────┐          ┌───────────────┴──────────────┐
    │ /gripper/command             │          │ /metal_detector/reading      │
    │   std_msgs/msg/Int8          │          │   std_msgs/msg/Float32 (20Hz)│
    │   (0=Neutral, 1=Open, 2=Grip)│          ├──────────────────────────────┤
    ├──────────────────────────────┤          │ /gripper/status              │
    │ /beacon/alert                │          │   std_msgs/msg/Int8 (10Hz)   │
    │   std_msgs/msg/Bool          │          ├──────────────────────────────┤
    │   (true=Force Alarm Sound)   │          │ /aux_heartbeat               │
    └──────────────────────────────┘          │   std_msgs/msg/Bool (1Hz)    │
                                              └──────────────────────────────┘
```

#### Published Topics
1. **`/metal_detector/reading`**
   - **Type:** `std_msgs/msg/Float32`
   - **Update Rate:** 20 Hz (every 50 ms)
   - **Field Specification:**
     * `data`: `float32` — Normalized detection intensity in range $[0.0f .. 1.0f]$.
       - `0.0f`: Baseline clean ground.
       - `0.0f < data < 0.2f`: Ambient soil mineralisation noise.
       - `≥ 0.2f`: Threshold trigger indicating metallic anomaly / mine detection.
       - `1.0f`: Full-scale analog saturation.
2. **`/gripper/status`**
   - **Type:** `std_msgs/msg/Int8`
   - **Update Rate:** 10 Hz (every 100 ms)
   - **Field Specification:**
     * `data`: `int8` — Discrete mechanical gripper position:
       - `0`: Neutral / Travel state ($1500\,\mu\text{s}$ pulse width).
       - `1`: Open envelope state ($1000\,\mu\text{s}$ pulse width).
       - `2`: Gripped / Clamped state ($2000\,\mu\text{s}$ pulse width).
       - `-1`: Error / Unknown state.
3. **`/aux_heartbeat`** (Alias: `/stm32_heartbeat`)
   - **Type:** `std_msgs/msg/Bool`
   - **Update Rate:** 1 Hz (every 1000 ms)
   - **Field Specification:**
     * `data`: `bool` — `true` confirms RTOS scheduler vitality and healthy DMA transport.

#### Subscribed Topics
1. **`/gripper/command`**
   - **Type:** `std_msgs/msg/Int8`
   - **Field Specification:**
     * `data`: `int8` — Target state setpoint:
       - `0`: Neutral ($1500\,\mu\text{s}$)
       - `1`: Open ($1000\,\mu\text{s}$)
       - `2`: Grip ($2000\,\mu\text{s}$)
2. **`/beacon/alert`**
   - **Type:** `std_msgs/msg/Bool`
   - **Field Specification:**
     * `data`: `bool` — Remote alarm override:
       - `true`: Forces buzzer siren and Red strobe ON regardless of local metal detector state.
       - `false`: Relinquishes override back to local detection logic.

---

### 4.2. Host SBC micro-ROS Agent Setup & Execution

On the Raspberry Pi 5 running Ubuntu 22.04 LTS and ROS 2 Humble:

#### 1. Udev Rule for Deterministic Serial Port
Create `/etc/udev/rules.d/99-stm32-auxiliary.rules`:
```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="AUX_BLACKPILL", SYMLINK+="ttyUSB_STM_AUX", MODE="0666"
```
Reload udev rules:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

#### 2. Launching micro-ROS Agent (Native Execution)
```bash
source /opt/ros/humble/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB_STM_AUX -b 921600
```

#### 3. Launching micro-ROS Agent (Docker Container)
```bash
docker run -it --rm \
    --net=host \
    --privileged \
    -v /dev:/dev \
    microros/micro-ros-agent:humble \
    serial --dev /dev/ttyUSB_STM_AUX -b 921600
```

#### 4. Host Verification & Diagnostics Commands
```bash
# Monitor live metal detector telemetry stream
ros2 topic echo /metal_detector/reading

# Monitor gripper state telemetry
ros2 topic echo /gripper/status

# Command gripper to Open
ros2 topic pub --once /gripper/command std_msgs/msg/Int8 "{data: 1}"

# Command gripper to Grip / Clamp
ros2 topic pub --once /gripper/command std_msgs/msg/Int8 "{data: 2}"

# Command gripper to Return to Neutral
ros2 topic pub --once /gripper/command std_msgs/msg/Int8 "{data: 0}"

# Trigger competition alarm test (Buzzer + Red Strobe)
ros2 topic pub --once /beacon/alert std_msgs/msg/Bool "{data: true}"
```

---

## 5. Debug Indicators API & Integration Guide

The board support driver [`status_beacon_driver.c`](file:///e:/MineSweeper/STM32_Auxiliary/Core/BSP/status_beacon_driver.c) / [`status_beacon_driver.h`](file:///e:/MineSweeper/STM32_Auxiliary/Core/BSP/status_beacon_driver.h) provides a clean C API to control the 3 status LEDs and audible buzzer.

### 5.1. Indicator Meaning & Visual Map

| Physical Indicator | Pin | Normal Operational State | Error / Diagnostic Trigger State |
| :--- | :--- | :--- | :--- |
| **LED 1 (Red)** | `PB13` | OFF (Clean ground) | **Flashing (10 Hz):** Landmine detected or remote alarm active<br>**Solid ON:** HardFault / System Panic |
| **LED 2 (Yellow)** | `PB14` | OFF (Gripper Neutral) | **Solid ON:** Gripper in non-neutral state (Open or Gripping)<br>**Pulsing:** Servo motion queue busy |
| **LED 3 (Green)** | `PB15` | **Solid ON:** micro-ROS agent synchronized | **OFF:** micro-ROS link down / serial timeout |
| **Buzzer** | `PB12` | Silent | **100 ms Chirp:** Successful boot<br>**Continuous / Pulsed Tone:** Active mine alert / HardFault panic |
| **Heartbeat LED** | `PC13` | **2 Hz Blinking (500 ms):** FreeRTOS running | **Frozen (Solid ON or OFF):** FreeRTOS kernel hung or starved |

---

### 5.2. Concrete C Code API Snippets

```c
#include "status_beacon_driver.h"
#include "auxiliary_pin_config.h"

/* ------------------------------------------------------------- */
/* 1. Startup Audible Chirp & Self-Test                          */
/* ------------------------------------------------------------- */
void System_Startup_SelfTest(void)
{
    Status_Beacon_Init();

    /* Sequential LED sweep: Red -> Yellow -> Green */
    Status_Beacon_SetLed(1, true);  HAL_Delay(100); Status_Beacon_SetLed(1, false);
    Status_Beacon_SetLed(2, true);  HAL_Delay(100); Status_Beacon_SetLed(2, false);
    Status_Beacon_SetLed(3, true);  HAL_Delay(100); Status_Beacon_SetLed(3, false);

    /* 100 ms audible chirp confirming bootloader handoff */
    Status_Beacon_SetBuzzer(true);
    HAL_Delay(100);
    Status_Beacon_SetBuzzer(false);
}

/* ------------------------------------------------------------- */
/* 2. Metal Detection Alert Trigger (in Task_Detector)           */
/* ------------------------------------------------------------- */
void Handle_Detector_Sample(detector_reading_t *sample)
{
    /* Automatically triggers Red LED strobe (PB13) and Siren (PB12) if target_detected is true */
    Status_Beacon_SetMineAlert(sample->target_detected);
}

/* ------------------------------------------------------------- */
/* 3. Gripper Movement Toggle (in Task_Gripper)                  */
/* ------------------------------------------------------------- */
void Handle_Gripper_State_Change(gripper_state_t new_state)
{
    /* Turn ON Yellow LED when gripper is in Open or Gripped position */
    bool is_busy_or_clamped = (new_state == GRIPPER_STATE_OPEN || new_state == GRIPPER_STATE_CLOSED);
    Status_Beacon_SetPayloadActive(is_busy_or_clamped);
}

/* ------------------------------------------------------------- */
/* 4. micro-ROS Connection Health (in Task_MicroROS)             */
/* ------------------------------------------------------------- */
void Update_Communication_Status(bool agent_connected)
{
    /* Turn ON Green LED when micro-ROS session is healthy */
    Status_Beacon_SetSystemReady(agent_connected);
}

/* ------------------------------------------------------------- */
/* 5. Hard Fault & Fatal Exception Trapping (in stm32f4xx_it.c)  */
/* ------------------------------------------------------------- */
void HardFault_Handler(void)
{
    __disable_irq();

    /* Sound audible alarm continuously */
    HAL_GPIO_WritePin(AUX_BEACON_BUZZER_PORT, AUX_BEACON_BUZZER_PIN, GPIO_PIN_SET);

    /* Turn OFF normal status indicators */
    HAL_GPIO_WritePin(AUX_LED2_PORT, AUX_LED2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AUX_LED3_PORT, AUX_LED3_PIN, GPIO_PIN_RESET);

    /* Trap processor in rapid Red LED distress flash loop */
    while (1)
    {
        HAL_GPIO_TogglePin(AUX_LED1_PORT, AUX_LED1_PIN);
        for (volatile uint32_t i = 0; i < 400000; i++)
        {
            __NOP();
        }
    }
}
```

---

## 6. Build, Flash & Verification Guide

### 6.1. STM32CubeIDE Build Procedure
1. Launch **STM32CubeIDE** (Version 1.14.0 or higher).
2. Select **File $\to$ Open Projects from File System...** and navigate to `e:\MineSweeper\STM32_Auxiliary`.
3. Verify that the project configuration is set to **`Release`** or **`Debug`**.
4. In **Project $\to$ Properties $\to$ C/C++ Build $\to$ Settings $\to$ Tool Settings**:
   - **MCU GCC Compiler $\to$ Include paths:** Confirm the following include directories are registered:
     * `../Core/Inc`
     * `../Core/Inc/interfaces`
     * `../Core/BSP`
     * `../Core/Tasks`
     * `../Core/Middleware/microros`
     * `../Drivers/STM32F4xx_HAL_Driver/Inc`
     * `../Middlewares/Third_Party/FreeRTOS/Source/include`
     * `../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2`
5. Press <kbd>Ctrl</kbd> + <kbd>B</kbd> or click **Project $\to$ Build Project**.
6. Ensure compilation finishes with `0 errors, 0 warnings`.

---

### 6.2. ST-Link v2 SWD Flashing

#### Physical Wiring to BlackPill:
* **ST-Link Pin 2 (SWCLK)** $\longrightarrow$ BlackPill **`PA14`** (`SWCLK`)
* **ST-Link Pin 4 (SWDIO)** $\longrightarrow$ BlackPill **`PA13`** (`SWDIO`)
* **ST-Link Pin 6 (GND)** $\longrightarrow$ BlackPill **`GND`**
* **ST-Link Pin 8 (3.3V)** $\longrightarrow$ BlackPill **`3V3`**

#### Flashing via STM32CubeProgrammer CLI:
```powershell
STM32_Programmer_CLI.exe -c port=SWD mode=UR -w "e:\MineSweeper\STM32_Auxiliary\Debug\STM32_Auxiliary.elf" -v -hardRst
```

#### Flashing via OpenOCD:
```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
        -c "program Debug/STM32_Auxiliary.elf verify reset exit"
```

---

### 6.3. Verification & Automated Test Suite

The repository includes a comprehensive pytest hardware-in-the-loop and software oracle validation suite located in `tests/`.

```powershell
# From workspace root (e:\MineSweeper):
pytest tests/tier1_features/test_r2_auxiliary_payload.py -v
```

#### Hardware Bench Test Checklist:
- [ ] **Oscilloscope on PB0:** Verify a clean $70.0\,\mu\text{s} \pm 1.0\,\mu\text{s}$ excitation pulse at $1000\,\text{Hz}$ repetition rate.
- [ ] **Oscilloscope on PA1:** Confirm that ADC sampling begins exactly $20\,\mu\text{s}$ after the falling edge of `PB0`, bypassing flyback ringing.
- [ ] **Logic Analyzer on PB6/PB7/PB8:** Verify $50\,\text{Hz}$ ($20\,\text{ms}$) frame rate with pulse widths of $1000\,\mu\text{s}$ (Open), $1500\,\mu\text{s}$ (Neutral), and $2000\,\mu\text{s}$ (Grip).
- [ ] **Audible Buzzer (PB12) & Red Strobe (PB13):** Confirm activation upon placing metallic target within coil field.
- [ ] **micro-ROS Link (PA9/PA10):** Verify continuous message reception on Raspberry Pi 5 terminal at 921,600 baud without frame dropouts.
