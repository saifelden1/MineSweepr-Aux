/**
 * @file microros_transport.c
 * @brief Configurable micro-ROS UART DMA Circular Transport implementation for STM32_Auxiliary.
 *
 * Implements non-blocking 2048-byte circular RX buffer with USART IDLE detection
 * and DMA normal TX with FreeRTOS CMSIS_V2 semaphore synchronization.
 */

#include "microros_transport.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <string.h>

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

static UART_HandleTypeDef *s_huart = &huart1;
static DMA_HandleTypeDef  *s_hdma_rx = &hdma_usart1_rx;
static DMA_HandleTypeDef  *s_hdma_tx = &hdma_usart1_tx;

static uint8_t s_rx_dma_buffer[MICROROS_RX_DMA_BUFFER_SIZE] __attribute__((aligned(4)));
static uint8_t s_tx_dma_buffer[MICROROS_TX_DMA_BUFFER_SIZE] __attribute__((aligned(4)));
static volatile size_t s_rx_tail = 0;
static volatile bool s_tx_busy = false;
static bool s_is_open = false;

/* OS Synchronization Primitives */
static osSemaphoreId_t s_tx_semaphore = NULL;
static const osSemaphoreAttr_t s_tx_semaphore_attr = {
    .name = "MicroROS_Aux_TxSem"
};

static osMutexId_t s_rx_mutex = NULL;
static const osMutexAttr_t s_rx_mutex_attr = {
    .name = "MicroROS_Aux_RxMutex"
};

static osMutexId_t s_tx_mutex = NULL;
static const osMutexAttr_t s_tx_mutex_attr = {
    .name = "MicroROS_Aux_TxMutex"
};

bool (MicroROS_Transport_Init)(void)
{
    if (s_huart == NULL)
    {
        return false;
    }

    s_rx_tail = 0;
    s_tx_busy = false;

    memset(s_rx_dma_buffer, 0, sizeof(s_rx_dma_buffer));
    memset(s_tx_dma_buffer, 0, sizeof(s_tx_dma_buffer));

    if (s_tx_semaphore == NULL)
    {
        s_tx_semaphore = osSemaphoreNew(1, 0, &s_tx_semaphore_attr);
    }
    if (s_rx_mutex == NULL)
    {
        s_rx_mutex = osMutexNew(&s_rx_mutex_attr);
    }
    if (s_tx_mutex == NULL)
    {
        s_tx_mutex = osMutexNew(&s_tx_mutex_attr);
    }

    /* Configure and link TX DMA if not already linked by MSP */
    if (s_huart->hdmatx == NULL && s_hdma_tx != NULL)
    {
        __HAL_RCC_DMA2_CLK_ENABLE();
        s_hdma_tx->Instance = DMA2_Stream7;
        s_hdma_tx->Init.Channel = DMA_CHANNEL_4;
        s_hdma_tx->Init.Direction = DMA_MEMORY_TO_PERIPH;
        s_hdma_tx->Init.PeriphInc = DMA_PINC_DISABLE;
        s_hdma_tx->Init.MemInc = DMA_MINC_ENABLE;
        s_hdma_tx->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        s_hdma_tx->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        s_hdma_tx->Init.Mode = DMA_NORMAL;
        s_hdma_tx->Init.Priority = DMA_PRIORITY_MEDIUM;
        s_hdma_tx->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        HAL_DMA_Init(s_hdma_tx);
        __HAL_LINKDMA(s_huart, hdmatx, *s_hdma_tx);

        HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
    }

    return MicroROS_Transport_Open();
}

bool MicroROS_Transport_InitHandles(void *huart, void *hdma_rx, void *hdma_tx)
{
    if (huart != NULL)   s_huart = (UART_HandleTypeDef*)huart;
    if (hdma_rx != NULL) s_hdma_rx = (DMA_HandleTypeDef*)hdma_rx;
    if (hdma_tx != NULL) s_hdma_tx = (DMA_HandleTypeDef*)hdma_tx;
    return (MicroROS_Transport_Init)();
}

bool MicroROS_Transport_Open(void)
{
    if (s_huart == NULL)
    {
        return false;
    }
    if (s_is_open)
    {
        return true;
    }

    s_rx_tail = 0;
    s_tx_busy = false;

    /* Start non-blocking circular DMA reception */
    if (HAL_UART_Receive_DMA(s_huart, s_rx_dma_buffer, MICROROS_RX_DMA_BUFFER_SIZE) != HAL_OK)
    {
        return false;
    }

    /* Enable USART IDLE line interrupt */
    __HAL_UART_ENABLE_IT(s_huart, UART_IT_IDLE);

    s_is_open = true;
    return true;
}

bool MicroROS_Transport_Close(void)
{
    if (s_huart == NULL || !s_is_open)
    {
        return false;
    }

    __HAL_UART_DISABLE_IT(s_huart, UART_IT_IDLE);
    HAL_UART_DMAStop(s_huart);

    s_is_open = false;
    return true;
}

bool MicroROS_Transport_IsOpen(void)
{
    return s_is_open;
}

size_t MicroROS_Transport_Available(void)
{
    if (s_huart == NULL || s_huart->hdmarx == NULL || !s_is_open)
    {
        return 0;
    }

    /* Hardware DMA counter decrements from buffer size to 0 */
    size_t head = MICROROS_RX_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(s_huart->hdmarx);

    if (head >= s_rx_tail)
    {
        return head - s_rx_tail;
    }
    else
    {
        return (MICROROS_RX_DMA_BUFFER_SIZE - s_rx_tail) + head;
    }
}

size_t MicroROS_Transport_Read(uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    if (buf == NULL || len == 0 || !s_is_open || s_huart == NULL)
    {
        return 0;
    }

    if (s_rx_mutex != NULL)
    {
        uint32_t mutex_timeout = (timeout_ms == 0) ? 0 : timeout_ms;
        if (osMutexAcquire(s_rx_mutex, mutex_timeout) != osOK)
        {
            return 0;
        }
    }

    /* If caller requested timeout and no bytes are ready, yield until available */
    if (MicroROS_Transport_Available() == 0 && timeout_ms > 0)
    {
        uint32_t start_tick = HAL_GetTick();
        while (MicroROS_Transport_Available() == 0)
        {
            if ((HAL_GetTick() - start_tick) >= timeout_ms)
            {
                if (s_rx_mutex != NULL)
                {
                    osMutexRelease(s_rx_mutex);
                }
                return 0;
            }
            if (osKernelGetState() == osKernelRunning)
            {
                osDelay(1);
            }
        }
    }

    size_t available = MicroROS_Transport_Available();
    if (available == 0)
    {
        if (s_rx_mutex != NULL)
        {
            osMutexRelease(s_rx_mutex);
        }
        return 0;
    }

    size_t to_read = (len < available) ? len : available;
    for (size_t i = 0; i < to_read; i++)
    {
        buf[i] = s_rx_dma_buffer[s_rx_tail];
        s_rx_tail = (s_rx_tail + 1) % MICROROS_RX_DMA_BUFFER_SIZE;
    }

    if (s_rx_mutex != NULL)
    {
        osMutexRelease(s_rx_mutex);
    }

    return to_read;
}

size_t MicroROS_Transport_Write(const uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    if (buf == NULL || len == 0 || !s_is_open || s_huart == NULL)
    {
        return 0;
    }

    if (s_tx_mutex != NULL)
    {
        if (osMutexAcquire(s_tx_mutex, timeout_ms) != osOK)
        {
            return 0;
        }
    }

    s_tx_busy = true;

    /* Copy into 32-bit aligned staging buffer if size permits */
    const uint8_t *tx_ptr;
    if (len <= sizeof(s_tx_dma_buffer))
    {
        memcpy(s_tx_dma_buffer, buf, len);
        tx_ptr = s_tx_dma_buffer;
    }
    else
    {
        tx_ptr = buf;
    }

    /* Drain any pending semaphore token */
    if (s_tx_semaphore != NULL)
    {
        while (osSemaphoreAcquire(s_tx_semaphore, 0) == osOK)
        {
            /* drain */
        }
    }

    /* Initiate non-blocking DMA normal transmission */
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(s_huart, (uint8_t*)tx_ptr, (uint16_t)len);
    if (status != HAL_OK)
    {
        /* Fall back to blocking UART transmit if DMA busy */
        status = HAL_UART_Transmit(s_huart, (uint8_t*)tx_ptr, (uint16_t)len, timeout_ms);
        s_tx_busy = false;
        if (s_tx_mutex != NULL)
        {
            osMutexRelease(s_tx_mutex);
        }
        return (status == HAL_OK) ? len : 0;
    }

    /* Wait for transmission completion via FreeRTOS CMSIS_V2 semaphore */
    if (s_tx_semaphore != NULL && osKernelGetState() == osKernelRunning)
    {
        if (osSemaphoreAcquire(s_tx_semaphore, timeout_ms) != osOK)
        {
            /* DMA transmission timed out */
            HAL_UART_AbortTransmit(s_huart);
            s_tx_busy = false;
            if (s_tx_mutex != NULL)
            {
                osMutexRelease(s_tx_mutex);
            }
            return 0;
        }
    }
    else
    {
        /* Busy wait if scheduler has not started yet */
        uint32_t start_tick = HAL_GetTick();
        while (s_tx_busy && ((HAL_GetTick() - start_tick) < timeout_ms))
        {
            /* wait */
        }
        if (s_tx_busy)
        {
            HAL_UART_AbortTransmit(s_huart);
            s_tx_busy = false;
            if (s_tx_mutex != NULL)
            {
                osMutexRelease(s_tx_mutex);
            }
            return 0;
        }
    }

    s_tx_busy = false;
    if (s_tx_mutex != NULL)
    {
        osMutexRelease(s_tx_mutex);
    }

    return len;
}

void MicroROS_Transport_OnUartIdle(void)
{
    /* Handled when USART IDLE line interrupt fires.
     * Hardware circular DMA continues seamlessly in background.
     */
}

void MicroROS_Transport_OnTxCplt(void)
{
    s_tx_busy = false;
    if (s_tx_semaphore != NULL)
    {
        osSemaphoreRelease(s_tx_semaphore);
    }
}

void MicroROS_Transport_OnRxCplt(void)
{
    /* DMA circular buffer wrapped around successfully */
}

void MicroROS_Transport_OnError(void)
{
    if (s_huart != NULL && s_is_open)
    {
        HAL_UART_DMAStop(s_huart);
        s_rx_tail = 0;
        HAL_UART_Receive_DMA(s_huart, s_rx_dma_buffer, MICROROS_RX_DMA_BUFFER_SIZE);
        __HAL_UART_ENABLE_IT(s_huart, UART_IT_IDLE);
    }
}
