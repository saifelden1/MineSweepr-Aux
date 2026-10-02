# STM32_Auxiliary ECU: System Requirements & Implementation Specification

**Document Version:** 1.0.0  
**Target Hardware:** STM32F411CEU6 BlackPill (ARM Cortex-M4 @ 96 MHz)  
**RTOS:** FreeRTOS (CMSIS-RTOS v2 API) + micro-ROS Client  
**Project Directory:** `STM32_Auxiliary/`

---

## 1. Subsystem Scope & Objectives

STM32_Auxiliary is the dedicated payload microcontroller responsible for real-time pulse induction metal detection, surface mine gripper actuation, visual/audible detection alerts, and micro-ROS telemetry with the Raspberry Pi 5 master SBC.

---

## 2. Hardware Interfaces & Circuit Specification

### 2.1. Pulse Induction Metal Detector Circuit
The detection coil relies on a pulsed magnetic field; eddy currents induced in metallic objects prolong the inductive flyback decay tail.

```
       STM32_Auxiliary (BlackPill)
       ┌───────────────────────────────┐
       │                               │
       │ Pin 1: TIM / Digital Pulse Out│────► [Buffer Transistors] ────► IRF740 MOSFET Gate ────► Search Coil
       │        (PB0 / PA8)            │                                                           │
       │                               │                                                       (Flyback)
       │ Pin 2: Analog Input (ADC1)    │◄─── [TL072 / TL071 Op-Amp] ◄──────────────────────────────┘
       │        (PA1 / ADC1_IN1)       │     (Decay Tail Signal Conditioning)
       └───────────────────────────────┘
```

#### Pin 1: Pulse Generator / Excitation Output
- **Pin Assignment:** `PB0` or `PA8` (Hardware Timer PWM/One-Pulse or Fast GPIO).
- **Driver Stage:** Interfaced to the gate of an IRF740 N-channel power MOSFET via bipolar buffer transistors (e.g. totem-pole NPN/PNP) for fast charging/discharging of the gate capacitance.
- **Pulse Duration:** $50\,\mu\text{s} - 100\,\mu\text{s}$ excitation pulse width.
- **Pulse Frequency:** $500\,\text{Hz} - 1000\,\text{Hz}$ repetition rate (period $1\,\text{ms} - 2\,\text{ms}$).

#### Pin 2: Signal Sampler / ADC Input
- **Pin Assignment:** `PA1` (Configured as `ADC1_IN1`, 12-bit resolution).
- **Signal Conditioning:** Direct output from a low-noise TL072/TL071 JFET-input operational amplifier stage that clamps high-voltage flyback spikes and amplifies the microsecond decay tail.
- **Blanking Interval:** A critical delay of $15\,\mu\text{s} - 25\,\mu\text{s}$ immediately following MOSFET gate turn-off must elapse before ADC conversion to bypass lethal flyback ringing.
- **Sampling Window:** ADC1 samples the decay curve across $25\,\mu\text{s} - 150\,\mu\text{s}$.
- **Signal Processing:**
  - Running exponential average of baseline soil readings:
    $$V_{baseline} \leftarrow \alpha V_{sample} + (1 - \alpha) V_{baseline}$$
  - Anomaly detection metric:
    $$\Delta V = V_{sample} - V_{baseline}$$
  - Normalized signal $S \in [0.0, 1.0]$ published to ROS 2:
    $$S = \text{clamp}\left(\frac{\Delta V}{V_{max\_scale}}, 0.0, 1.0\right)$$

### 2.2. Gripper Mechanism Actuation
- **Actuator Type:** 1-2 standard RC servo motors (or DC gear motor with end-stop limit switches).
- **Control Interface:** 50 Hz PWM via TIM3/TIM2:
  - `1000 µs`: Gripper fully OPEN.
  - `1500 µs`: NEUTRAL / Travel position.
  - `2000 µs`: Gripper fully CLOSED / GRIP.

### 2.3. Detection Beacon & Siren
- **Buzzer Output:** GPIO driving NPN transistor for audible frequency alarm.
- **Strobe LED:** High-brightness LED beacon signaling landmine location to competition field judges.

### 2.4. SBC Communication (micro-ROS Client)
- **Interface:** USART2 or Native USB CDC to Raspberry Pi 5.
- **Baud Rate:** $921600\,\text{baud}$ (UART) or 12 Mbps (USB CDC).

---

## 3. micro-ROS Topic & Interface Specifications

1. **Publisher:** `/metal_detector/reading` (`std_msgs/msg/Float32` @ 20 Hz)
   - Transmits normalized metal detection intensity ($0.0 = \text{clean ground}, 1.0 = \text{heavy metallic signature}$).
2. **Publisher:** `/gripper/status` (`std_msgs/msg/Int8` @ 10 Hz)
   - Acknowledges current mechanical gripper state (`0`=Neutral, `1`=Open, `2`=Grip, `3`=Moving/Busy).
3. **Subscriber:** `/gripper/command` (`std_msgs/msg/Int8`)
   - Commands gripper state change (`1` = Open to acquire, `2` = Grip object, `0` = Neutral).
4. **Subscriber:** `/beacon/alert` (`std_msgs/msg/Bool`)
   - Enables/disables buzzer siren and strobe light.

---

## 4. FreeRTOS Task Architecture

```
Task Name           Priority          Period       Function
-----------------------------------------------------------------------------------------
Task_PulseInduction osPriorityHigh     1-2 ms      Fires Pin 1 pulse, ADC DMA sample Pin 2
Task_GripperControl osPriorityNormal   10 ms       Drives gripper servos / motion states
Task_MicroROS       osPriorityNormal   5 ms        Runs micro-ROS agent executor & spin
Task_Alert          osPriorityLow      50 ms       Controls buzzer and LED blinking patterns
```

---

## 5. Implementation Checklist for Dedicated Chat

- [ ] **Task 1:** Remove obsolete cloned mobility files from `STM32_Auxiliary/Core/HAL_Drivers` (clean up Encoders, MDD10A, PID, IMU6500).
- [ ] **Task 2:** Rewrite `Core/BSP/pi_detector_driver.c` and `pi_detector_driver.h`:
  - Configure Pin 1 (timer pulse output driving IRF740 gate via buffer).
  - Configure Pin 2 (ADC1 channel reading TL072/TL071 decay tail).
  - Implement blanking delay ($15-25\,\mu\text{s}$) and sample integration.
- [ ] **Task 3:** Implement baseline ground subtraction and calibration function.
- [ ] **Task 4:** Finalize `Core/BSP/servo_gripper_driver.c` for 50 Hz PWM servo actuation.
- [ ] **Task 5:** Set up micro-ROS publishers (`/metal_detector/reading`, `/gripper/status`) and subscribers (`/gripper/command`, `/beacon/alert`).
- [ ] **Task 6:** Implement buzzer and strobe alert drivers.
- [ ] **Task 7:** Perform hardware bench verification of pulse timing with oscilloscope / logic analyzer.
