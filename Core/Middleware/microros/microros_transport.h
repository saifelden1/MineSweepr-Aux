/**
 * @file microros_transport.h
 * @brief Configurable micro-ROS Serial Transport Layer (UART DMA) for STM32_Auxiliary.
 *
 * Implements high-throughput, non-blocking serial communication for micro-ROS
 * using FreeRTOS CMSIS_V2 synchronization, DMA circular reception on DMA2_Stream2,
 * and DMA normal transmission on DMA2_Stream7.
 *
 * Target: STM32F411CEU6 BlackPill (ARM Cortex-M4 @ 96 MHz)
 */

#ifndef MICROROS_TRANSPORT_H
#define MICROROS_TRANSPORT_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* CONFIGURABLE SERIAL BAUD RATE                                              */
/* ========================================================================== */
#ifndef MICROROS_BAUDRATE
#define MICROROS_BAUDRATE             (921600U) /**< Supported: 115200, 460800, 921600 */
#endif

#ifndef MICROROS_UART_BAUDRATE
#define MICROROS_UART_BAUDRATE        MICROROS_BAUDRATE /**< Backwards-compatible alias */
#endif

/* Buffer sizing and timing constants */
#define MICROROS_RX_DMA_BUFFER_SIZE   (2048U)   /**< 2KB Circular Ring Buffer */
#define MICROROS_RX_BUFFER_SIZE       MICROROS_RX_DMA_BUFFER_SIZE /**< Backwards-compatible alias */
#define MICROROS_UART_DMA_BUF_SIZE    MICROROS_RX_DMA_BUFFER_SIZE /**< Backwards-compatible alias */
#define MICROROS_TX_DMA_BUFFER_SIZE   (512U)    /**< 512B Non-blocking TX Stage */
#define MICROROS_DEFAULT_TIMEOUT_MS   (50U)
#define MICROROS_TX_TIMEOUT_MS        (100U)

/* ========================================================================== */
/* TRANSPORT LIFECYCLE & INITIALIZATION                                       */
/* ========================================================================== */
/**
 * @brief Initialize the micro-ROS transport layer using default USART1/DMA handles.
 * @return true on success, false on error.
 */
bool MicroROS_Transport_Init(void);

/**
 * @brief Initialize transport layer with explicit HAL handles (opaque pointers).
 * @param huart Pointer to UART_HandleTypeDef (e.g. &huart1)
 * @param hdma_rx Pointer to DMA_HandleTypeDef for RX (e.g. &hdma_usart1_rx)
 * @param hdma_tx Pointer to DMA_HandleTypeDef for TX (e.g. &hdma_usart1_tx)
 * @return true on success, false on error.
 */
bool MicroROS_Transport_InitHandles(void *huart, void *hdma_rx, void *hdma_tx);

/**
 * @brief Polymorphic macro allowing both MicroROS_Transport_Init()
 *        and MicroROS_Transport_Init(&huart1, &rx, &tx) without leaking HAL types.
 */
#define _UROS_INIT_SELECT(_0, _1, _2, _3, NAME, ...) NAME
#define MicroROS_Transport_Init(...) \
    _UROS_INIT_SELECT(_0, ##__VA_ARGS__, MicroROS_Transport_InitHandles, _err2, _err1, MicroROS_Transport_Init)(__VA_ARGS__)

/**
 * @brief Open transport channel and start circular RX DMA.
 * @return true on success.
 */
bool MicroROS_Transport_Open(void);

/**
 * @brief Close transport channel and stop RX DMA.
 * @return true on success.
 */
bool MicroROS_Transport_Close(void);

/**
 * @brief Query whether transport is open and active.
 * @return true if open and ready.
 */
bool MicroROS_Transport_IsOpen(void);

/* ========================================================================== */
/* CMSIS_V2 THREAD-SAFE STREAM I/O                                            */
/* ========================================================================== */
/**
 * @brief Non-blocking DMA write with FreeRTOS CMSIS_V2 semaphore synchronization.
 * @param buf Pointer to buffer to transmit.
 * @param len Number of bytes to transmit.
 * @param timeout_ms Maximum wait time in milliseconds.
 * @return Number of bytes successfully transmitted.
 */
size_t MicroROS_Transport_Write(const uint8_t *buf, size_t len, uint32_t timeout_ms);

/**
 * @brief Thread-safe read from circular RX buffer.
 * @param buf Destination buffer.
 * @param len Maximum bytes to read.
 * @param timeout_ms Maximum wait time in milliseconds (0 for non-blocking).
 * @return Number of bytes actually read.
 */
size_t MicroROS_Transport_Read(uint8_t *buf, size_t len, uint32_t timeout_ms);

/**
 * @brief Query number of unread bytes available in the DMA circular buffer.
 * @return Available byte count [0 .. MICROROS_RX_DMA_BUFFER_SIZE].
 */
size_t MicroROS_Transport_Available(void);

/* ========================================================================== */
/* LOW-LEVEL INTERRUPT CALLBACKS (Called from stm32f4xx_it.c)                 */
/* ========================================================================== */
void MicroROS_Transport_OnUartIdle(void);
void MicroROS_Transport_OnTxCplt(void);
void MicroROS_Transport_OnRxCplt(void);
void MicroROS_Transport_OnError(void);

/**
 * @brief Backwards-compatible alias for OnUartIdle.
 */
static inline void MicroROS_Transport_IdleCallback(void)
{
    MicroROS_Transport_OnUartIdle();
}

#ifdef __cplusplus
}
#endif

#endif /* MICROROS_TRANSPORT_H */
