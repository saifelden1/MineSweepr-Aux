# Auxiliary ECU requirements

Baseline: **17 October 2026**. Target **STM32F103C8T6 Blue Pill, two servos moving together, single PB12 active-HIGH detection relay**. See [architecture](SYSTEM_ARCHITECTURE.md) for wiring, timing, protocol, physical commissioning order and build commands. This replaces the old F411/micro-ROS/separate-indicator requirements. The combined runbook is linked there; physical checks are not implied by publication.

## Board, memory and timing

| ID | Requirement | Acceptance evidence |
|---|---|---|
| AUX-01 | Active CubeMX, CubeIDE and CMake targets use F103C8T6, Cortex-M3 soft-float, F1 HAL and the CM3 RTOS port. | Confirm chip marking and compile both toolchain paths without active F4 sources. |
| AUX-02 | Firmware fits 64 KB flash / 20 KB SRAM, including RTOS heap, static objects and stacks. | Link map, memory report and sustained-run stack high-water marks. |
| AUX-03 | 8 MHz HSE produces 72 MHz system clock; TIM3 supplies HAL time and SysTick supplies RTOS time. | Inspect configuration and measured task/peripheral timing. |
| AUX-04 | Two synchronized servo outputs exist: PB6 / TIM4 CH1 and PB7 / TIM4 CH2, 50 Hz, 1 µs resolution. PB8 does not emit servo PWM. | Scope both servo pins and PB8 and inspect configuration. |
| AUX-05 | Detector pulse, blanking and ADC acquisition use initialized timers and bounded conversion waits, including counter wrap. | Scope 70 µs pulse / 20 µs blanking, acquisition sequence and failure/rollover behavior. |

## Payload behavior

| ID | Requirement | Acceptance evidence |
|---|---|---|
| AUX-06 | Commands 0/1/2 select neutral/open/close on both servos together. Default pulses are 1500/1000/2000 µs and must match measured mechanical limits. | GUI command → valid serial frame → scoped PWM → physical servo response. |
| AUX-07 | `/gripper/status` reports commanded state; unknown/error is −1. Do not infer closed-loop position or grasp. | Check telemetry during preset and invalid/unknown states. |
| AUX-08 | Metal readings come from actual recent ADC samples, normalize to 0–1, and are not published as fresh when acquisition fails. | Target/clear-ground observations plus missing/failed acquisition scenarios. |
| AUX-09 | Local detection alone controls the PB12 active-HIGH relay for siren + red indicator. Relay starts OFF, turns ON while metal is detected and OFF on a valid clear reading. No status LED/buzzer/heartbeat outputs or remote relay override. Detection does not automatically open/close the servo. | Observe PB12 at startup, detection and clear; verify remote beacon frames cannot switch it and servo commands remain independent. |
| AUX-10 | Invalid command values, wrong lengths or corrupt CRC frames cannot change servo/alarm state. | Framing, split-input, corruption and value-boundary checks. |

## Pi / ROS integration

| ID | Requirement | Acceptance evidence |
|---|---|---|
| AUX-11 | Use the existing CRC-framed USART1 protocol through a Pi serial bridge, at 921600 baud with distinct stable adapter identity. | Verify actual wiring, device identity and bidirectional frames. |
| AUX-12 | Publish detector/status/heartbeat at nominal 20/10/1 Hz and map IDs/types exactly as specified in the architecture. | Measured ROS topic rates and decoded frames under load. |
| AUX-13 | GUI/bridge reject servo commands before valid Aux health and after its 2.5 s timeout; reconnect must not replay old commands. | Startup, unplug/replug, board reset and queued-command observations. |
| AUX-14 | Kernel/object/task initialization is valid; ADC timing cannot hang in a HAL timeout while interrupts are masked. | Compile/source checks plus stressed runtime/failure-injection evidence. |

## Scope and readiness

No independent servo commands, third servo, mobility motor control, IMU/GPS, XRCE client, object-grasp feedback or autonomous pickup is required in this ECU baseline. Its `microros_*` filenames are legacy names for custom serial code.

CMake compile/link passed on 16 October 2026: 25,064 bytes flash / 65,536 available, and 16,880 bytes linker-reported RAM / 20,480 available. Runtime stack high-water marks, CubeMX/CubeIDE compatibility, flashing and physical measurements remain acceptance work. Use the workspace testing guide at `Auto/TestingGuide.md`, and report PASS/FAIL/BLOCKED/NOT RUN rather than inferring hardware readiness from compilation.

## Deployment handoff — 16 October 2026

The Blue Pill is not yet available to the operator. Paired PWM outputs are implemented in source, but flashing, servo travel/current checks and detector calibration remain NOT RUN on hardware. Laptop GUI support and the optional Pi bridge are tracked in `Auto/Auto_ws/DEPLOYMENT.md` in the sibling workspace. Use a separate 3.3 V USB–UART adapter; Mobility already occupies the Pi GPIO UART. A common ground and external regulated servo supply are required. Do not run a micro-ROS agent on this custom-protocol Aux link.
