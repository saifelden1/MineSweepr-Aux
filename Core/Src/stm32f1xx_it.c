#include "main.h"
#include "usart.h"
#include "microros_transport.h"
extern TIM_HandleTypeDef htim3;
void NMI_Handler(void) { Error_Handler(); }
void HardFault_Handler(void) { Error_Handler(); }
void MemManage_Handler(void) { Error_Handler(); }
void BusFault_Handler(void) { Error_Handler(); }
void UsageFault_Handler(void) { Error_Handler(); }
void DebugMon_Handler(void) {}
void DMA1_Channel4_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart1_tx); }
void DMA1_Channel5_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart1_rx); }
void USART1_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1); MicroROS_Transport_OnUartIdle();
    }
    HAL_UART_IRQHandler(&huart1);
}
void TIM3_IRQHandler(void) { HAL_TIM_IRQHandler(&htim3); }
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *u) { if (u==&huart1) MicroROS_Transport_OnTxCplt(); }
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *u) { if (u==&huart1) MicroROS_Transport_OnRxCplt(); }
void HAL_UART_ErrorCallback(UART_HandleTypeDef *u) { if (u==&huart1) MicroROS_Transport_OnError(); }
