/**
 * @file auxiliary_pin_config.h
 * @brief Centralized Hardware Pin & Peripheral Configuration for STM32_Auxiliary ECU.
 *
 * Microcontroller: STM32F411CEU6 BlackPill (ARM Cortex-M4F @ 96 MHz)
 * System Clock: 96 MHz via 25 MHz HSE PLL (or 16 MHz HSI PLL fallback)
 *
 * Centralizes all pin numbers, GPIO ports, timer channels, and ADC channels.
 * Modifying definitions in this header remaps hardware peripherals without
 * changing any driver or application logic.
 */

#ifndef AUXILIARY_PIN_CONFIG_H
#define AUXILIARY_PIN_CONFIG_H

#include "stm32f4xx_hal.h"

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
/* 2. SERVO GRIPPER PWM CONFIGURATION                                        */
/* ========================================================================= */
/** Primary Servo PWM (Channel 1: PB6, TIM4_CH1) */
#define AUX_GRIPPER_PWM1_PORT           GPIOB
#define AUX_GRIPPER_PWM1_PIN            GPIO_PIN_6
#define AUX_GRIPPER_PWM1_AF             GPIO_AF2_TIM4
#define AUX_GRIPPER_PWM1_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define AUX_GRIPPER_TIMER_INSTANCE      TIM4
#define AUX_GRIPPER_PWM1_CHANNEL        TIM_CHANNEL_1
#define AUX_GRIPPER_TIM_CLK_ENABLE()    __HAL_RCC_TIM4_CLK_ENABLE()

/** Secondary Servo / Wrist PWM (Channel 2: PB7, TIM4_CH2) */
#define AUX_GRIPPER_PWM2_PORT           GPIOB
#define AUX_GRIPPER_PWM2_PIN            GPIO_PIN_7
#define AUX_GRIPPER_PWM2_AF             GPIO_AF2_TIM4
#define AUX_GRIPPER_PWM2_CHANNEL        TIM_CHANNEL_2

/** 50 Hz RC Servo Timing (1 MHz timer clock: 1 us / tick) */
#define AUX_GRIPPER_PWM_PERIOD_TICKS    (19999U) /**< 20 ms period (50 Hz) ARR = 19999 */
#define AUX_GRIPPER_PWM_PRESCALER       (95U)    /**< 96 MHz / 96 = 1 MHz tick resolution */

/** Discrete State Microsecond Setpoints */
#define AUX_GRIPPER_PULSE_NEUTRAL_US    (1500U)  /**< State 0: Neutral / Travel (1.5 ms) */
#define AUX_GRIPPER_PULSE_OPEN_US       (1000U)  /**< State 1: Open envelope (1.0 ms) */
#define AUX_GRIPPER_PULSE_GRIP_US       (2000U)  /**< State 2: Grip / Clamped (2.0 ms) */
#define AUX_GRIPPER_PULSE_MIN_US        (1000U)  /**< Lower mechanical limit */
#define AUX_GRIPPER_PULSE_MAX_US        (2000U)  /**< Upper mechanical limit */

/* ========================================================================= */
/* 3. STATUS BEACON ALARM CONFIGURATION                                      */
/* ========================================================================= */
/** Audible Buzzer Alarm Stage (PB12) */
#define AUX_BEACON_BUZZER_PORT          GPIOB
#define AUX_BEACON_BUZZER_PIN           GPIO_PIN_12
#define AUX_BEACON_BUZZER_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()

/** Visual Strobe LED Alarm Stage (PB13) */
#define AUX_BEACON_STROBE_PORT          GPIOB
#define AUX_BEACON_STROBE_PIN           GPIO_PIN_13
#define AUX_BEACON_STROBE_CLK_ENABLE()  __HAL_RCC_GPIOB_CLK_ENABLE()

/* ========================================================================= */
/* 4. ONBOARD SYSTEM DIAGNOSTICS                                             */
/* ========================================================================= */
/** Active-LOW User Heartbeat LED (PC13 on BlackPill) */
#define AUX_HEARTBEAT_PORT              GPIOC
#define AUX_HEARTBEAT_PIN               GPIO_PIN_13
#define AUX_HEARTBEAT_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE()

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
#define AUX_MICROROS_UART_TX_AF         GPIO_AF7_USART1
#define AUX_MICROROS_UART_RX_PORT       GPIOA
#define AUX_MICROROS_UART_RX_PIN        GPIO_PIN_10
#define AUX_MICROROS_UART_RX_AF         GPIO_AF7_USART1
#define AUX_MICROROS_UART_CLK_ENABLE()  __HAL_RCC_USART1_CLK_ENABLE()
#define AUX_MICROROS_DMA_CLK_ENABLE()   __HAL_RCC_DMA2_CLK_ENABLE()
#define AUX_MICROROS_RX_BUF_SIZE        (2048U)

/* ========================================================================= */
/* 6. HARDWARE TIMEBASE TIMERS                                               */
/* ========================================================================= */
/** Free-running microsecond counter (TIM2, 32-bit @ 1 MHz) */
#define AUX_TIMEBASE_TIMER_INSTANCE     TIM2
#define AUX_TIMEBASE_TIM_CLK_ENABLE()   __HAL_RCC_TIM2_CLK_ENABLE()
#define AUX_TIMEBASE_PRESCALER          (95U)    /**< 96 MHz / 96 = 1 MHz */

#ifdef __cplusplus
}
#endif

#endif /* AUXILIARY_PIN_CONFIG_H */
