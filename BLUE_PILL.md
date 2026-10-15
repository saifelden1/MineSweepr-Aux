# Auxiliary ECU — Blue Pill, one servo

The active target is **STM32F103C8T6**, 8 MHz HSE → 72 MHz, 64 KB flash and 20 KB SRAM. `STM32_Auxiliary.ioc`, both CubeIDE configurations and CMake target the F1 HAL, CM3 FreeRTOS port and `STM32F103C8TX_FLASH.ld`. Old F4 vendor libraries/linker scripts remain as historical source and are excluded from active builds; they are not the active firmware target.

## Wiring and timing

| Function | Pin / peripheral | Configuration |
|---|---|---|
| Single servo | PB6, TIM4 CH1 | 50 Hz; 1 µs tick, PSC 71, ARR 19999 |
| Detector gate | PB0 GPIO | 70 µs excitation, then 20 µs blanking |
| Detector ADC | PA1, ADC1 IN1 | 12 MHz ADC; eight samples with bounded conversion waits |
| Detector time counter | TIM2 | 1 MHz, 16-bit; wrap-safe subtraction |
| Pi serial | PA9 TX / PA10 RX, USART1 | 3.3 V USB–UART adapter, 921600 baud; DMA1 CH4 TX / CH5 circular RX |
| HAL clock | TIM3 | 1 kHz; SysTick belongs to FreeRTOS |
| Indicators | PB12 buzzer; PB13/PB14/PB15 LEDs; PC13 heartbeat | Existing payload indicator wiring |
| Programming | PA13 SWDIO / PA14 SWCLK | ST-Link/SWD and common ground |

PB7/PB8 are no longer configured as servo outputs. Power the servo from a suitable external supply with common ground; do not supply its motor through an STM GPIO. Confirm the detector front end respects the ADC voltage range and that pin assignments match the assembled circuit.

Neutral/open/close commands are `0/1/2`, defaulting to 1500/1000/2000 µs respectively. These pulses are configurable in `Core/Inc/auxiliary_pin_config.h` and must be checked against the actual servo mechanism. `/gripper/status` reports the commanded preset; error/unknown is `-1`. There is no physical feedback or automatic detector-triggered servo movement.

## ROS alignment

Despite legacy `microros_*` filenames, Aux uses **custom framed UART**, not the XRCE protocol. Start the Pi's `payload_serial_bridge`, not a micro-ROS agent, for this board.

Frames: `AA 55`, one-byte topic ID, little-endian 16-bit payload length, payload, little-endian CRC16/MODBUS over topic+length+payload.

| ID | Direction | ROS topic / payload |
|---|---|---|
| `0x10` | Aux → Pi | `/metal_detector/reading`, little-endian float32, normalized 0–1, 20 Hz |
| `0x11` | Aux → Pi | `/gripper/status`, signed int8, 10 Hz |
| `0x12` | Aux → Pi | `/payload/heartbeat`, one byte 0/1, 1 Hz |
| `0x20` | Pi → Aux | `/gripper/command`, one byte 0/1/2 |
| `0x21` | Pi → Aux | `/beacon/alert`, one byte 0/1 |

Detector readings are published only after actual recent samples. Serial commands have strict length/value checks. The Pi/GUI do not queue or replay servo commands when the link is unavailable. The one-entry local servo queue processes live commands; no multi-servo channel API remains. UART-open status inside firmware is not proof of a live ROS session.

## Build and verification

Import the existing project into CubeIDE, or run:

```bash
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-gcc.cmake
cmake --build build
```

CMake writes `build/STM32_Auxiliary.elf`, `.bin` and `.map`. ARM GNU tools are required. Check flash/RAM fit and task stack high-water marks before flashing through ST-Link. FreeRTOS heap is 10 KB instead of the old 32 KB, with detector/gripper/link stacks of 1024/768/1536 bytes. CMSIS objects are created after kernel initialization, and duplicate generated task implementations were removed.

Vendored dependencies: ST F1 HAL v1.1.10 (`77fbb30b7a1d02533980400083e48c559aae5a4f`), CMSIS F1 v4.3.4 (`49daecceeb4b251999a784995e8b8d714b59dbc2`), FreeRTOS CM3 port V10.3.1 matching the bundled kernel. Component license files and source notices are retained.

The custom peripheral/application sources are authoritative. The `.ioc` records configuration but its regeneration/opening has not been checked here. Regenerate into a separate folder and compare before replacing custom code. `README.md` and other older Black Pill guides describe the previous target; this document is the current board reference. Builds, flashing, detector/servo timing and runtime acceptance are assigned to `Auto/Auto_ws/deploy/manual/ANTIGRAVITY_HANDOFF.md`.
