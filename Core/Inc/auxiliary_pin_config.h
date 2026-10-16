/**
 * @file auxiliary_pin_config.h
 * @brief Centralized Hardware Pin & Peripheral Configuration for STM32_Auxiliary ECU.
 *
 * Microcontroller: STM32F103C8T6 Blue Pill (ARM Cortex-M3 @ 72 MHz)
 * System Clock: 72 MHz via 8 MHz HSE PLL
 *
 * Centralizes all pin numbers, GPIO ports, timer channels, and ADC channels.
 * Modifying definitions in this header remaps hardware peripherals without
 * changing any driver or application logic.
 */

#ifndef AUXILIARY_PIN_CONFIG_H
#define AUXILIARY_PIN_CONFIG_H

#include "stm32f1xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/* 1. PULSE INDUCTION METAL DETECTOR CONFIGURATION                           */
/* ========================================================================= */
/** Excitation MOSFET Gate Output: Drives IRF740 N-channel power MOSFET */
#define AUX_DETECTOR_PULSE_PORT         GPIOB
#define AUX_DETECTOR_PULSE_PIN          GPIO_PIN_0
#define AUX_DETECTOR_PULSE_CLK_ENABLE() __HAL_RCC_GPIOB_CLK_ENABLE()

/** Analog Decay Tail Input: Reads TL072/TL071 operational amplifier decay tail */
#define AUX_DETECTOR_ADC_PORT           GPIOA
#define AUX_DETECTOR_ADC_PIN            GPIO_PIN_1
#define AUX_DETECTOR_ADC_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()
#define AUX_DETECTOR_ADC_INSTANCE       ADC1
#define AUX_DETECTOR_ADC_CHANNEL        ADC_CHANNEL_1
#define AUX_DETECTOR_ADC_CLK_ENABLE_ADC() __HAL_RCC_ADC1_CLK_ENABLE()

/** Pulse Induction Timing & Filtering Parameters */
#define AUX_DETECTOR_PULSE_WIDTH_US     (70U)   /**< Gate excitation pulse width (70 us) */
#define AUX_DETECTOR_BLANKING_US        (20U)   /**< Flyback blanking window delay (20 us) */
#define AUX_DETECTOR_REPETITION_HZ      (1000U) /**< Excitation coil repetition rate limit */
#define AUX_DETECTOR_OVERSAMPLES        (8U)    /**< Successive ADC1 oversamples per pulse */
#define AUX_DETECTOR_BASELINE_ALPHA     (0.02f) /**< Exponential Moving Average alpha */
#define AUX_DETECTOR_DEFAULT_SCALE_ADC  (1500U) /**< ADC delta span mapped to 1.0f intensity */
#define AUX_DETECTOR_DEFAULT_THRESH_ADC (300U)  /**< ADC delta threshold to flag mine detection */

/* ========================================================================= */
/* 2. DUAL-CHANNEL SYNCHRONIZED SERVO CONFIGURATION (TIM4 CH1 & CH2)        */
/* ========================================================================= */
/** Servo Timer Base */
#define AUX_GRIPPER_TIMER_INSTANCE      TIM4
#define AUX_GRIPPER_TIM_CLK_ENABLE()    __HAL_RCC_TIM4_CLK_ENABLE()

/** Servo 1 PWM (Channel 1: PB6, TIM4_CH1) */
#define AUX_GRIPPER_PWM1_PORT           GPIOB
#define AUX_GRIPPER_PWM1_PIN            GPIO_PIN_6
#define AUX_GRIPPER_PWM1_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define AUX_GRIPPER_PWM1_CHANNEL        TIM_CHANNEL_1

/** Servo 2 PWM (Channel 2: PB7, TIM4_CH2 - moves in unison with Servo 1) */
#define AUX_GRIPPER_PWM2_PORT           GPIOB
#define AUX_GRIPPER_PWM2_PIN            GPIO_PIN_7
#define AUX_GRIPPER_PWM2_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define AUX_GRIPPER_PWM2_CHANNEL        TIM_CHANNEL_2

#define AUX_SERVO1_PORT                 AUX_GRIPPER_PWM1_PORT
#define AUX_SERVO1_PIN                  AUX_GRIPPER_PWM1_PIN
#define AUX_SERVO1_CHANNEL              AUX_GRIPPER_PWM1_CHANNEL

#define AUX_SERVO2_PORT                 AUX_GRIPPER_PWM2_PORT
#define AUX_SERVO2_PIN                  AUX_GRIPPER_PWM2_PIN
#define AUX_SERVO2_CHANNEL              AUX_GRIPPER_PWM2_CHANNEL

/** 50 Hz RC Servo Timing (1 MHz timer clock: 1 us / tick) */
#define AUX_GRIPPER_PWM_PERIOD_TICKS    (19999U) /**< 20 ms period (50 Hz) ARR = 19999 */
#define AUX_GRIPPER_PWM_PRESCALER       (71U)    /**< 72 MHz / 72 = 1 MHz tick resolution */

/** Discrete State Microsecond Setpoints */
#define AUX_GRIPPER_PULSE_NEUTRAL_US    (1500U)  /**< State 0: Neutral / Travel (1.5 ms) */
#define AUX_GRIPPER_PULSE_OPEN_US       (1000U)  /**< State 1: Open envelope (1.0 ms) */
#define AUX_GRIPPER_PULSE_GRIP_US       (2000U)  /**< State 2: Grip / Clamped (2.0 ms) */
#define AUX_GRIPPER_PULSE_MIN_US        (1000U)  /**< Lower mechanical limit */
#define AUX_GRIPPER_PULSE_MAX_US        (2000U)  /**< Upper mechanical limit */

/* ========================================================================= */
/* 3. SINGLE DETECTION RELAY OUTPUT                                          */
/* ========================================================================= */
/** PB12 controls a 3.3 V-compatible relay module, never a bare relay coil.
 * HIGH = siren + red indicator ON; LOW = OFF. Use a hardware pull-down
 * on the module input to keep an active-HIGH relay off during reset.
 */
#define AUX_RELAY_PORT                 GPIOB
#define AUX_RELAY_PIN                  GPIO_PIN_12
#define AUX_RELAY_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()
#define AUX_RELAY_ON_LEVEL             GPIO_PIN_SET
#define AUX_RELAY_OFF_LEVEL            GPIO_PIN_RESET

/* ========================================================================= */
/* 5. MICROROS SERIAL TRANSPORT CONFIGURATION                                */
/* ========================================================================= */
/** Prominent baud rate configuration macro */
#ifndef MICROROS_BAUDRATE
#define MICROROS_BAUDRATE               (921600U)
#endif

#define AUX_MICROROS_BAUDRATE           MICROROS_BAUDRATE
#define AUX_MICROROS_UART_INSTANCE      USART1
#define AUX_MICROROS_UART_TX_PORT       GPIOA
#define AUX_MICROROS_UART_TX_PIN        GPIO_PIN_9
#define AUX_MICROROS_UART_RX_PORT       GPIOA
#define AUX_MICROROS_UART_RX_PIN        GPIO_PIN_10
#define AUX_MICROROS_UART_CLK_ENABLE()  __HAL_RCC_USART1_CLK_ENABLE()
#define AUX_MICROROS_DMA_CLK_ENABLE()   __HAL_RCC_DMA1_CLK_ENABLE()
#define AUX_MICROROS_RX_BUF_SIZE        (2048U)

/* ========================================================================= */
/* 6. HARDWARE TIMEBASE TIMERS                                               */
/* ========================================================================= */
/** Free-running microsecond counter (TIM2, 32-bit @ 1 MHz) */
#define AUX_TIMEBASE_TIMER_INSTANCE     TIM2
#define AUX_TIMEBASE_TIM_CLK_ENABLE()   __HAL_RCC_TIM2_CLK_ENABLE()
#define AUX_TIMEBASE_PRESCALER          (71U)    /**< 72 MHz / 72 = 1 MHz */

#ifdef __cplusplus
}
#endif

#endif /* AUXILIARY_PIN_CONFIG_H */
