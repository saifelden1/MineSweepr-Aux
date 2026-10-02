/**
 * @file task_microros.h
 * @brief micro-ROS Client Task over UART DMA for STM32_Auxiliary.
 *
 * Manages publishing of:
 *   - /metal_detector/reading (std_msgs/msg/Float32 @ 20 Hz)
 *   - /gripper/status         (std_msgs/msg/Int8 @ 10 Hz)
 *   - /stm32_heartbeat        (std_msgs/msg/Bool @ 1 Hz)
 * And subscription to:
 *   - /gripper/command        (std_msgs/msg/Int8)
 *   - /beacon/alert           (std_msgs/msg/Bool)
 */

#ifndef TASK_MICROROS_H
#define TASK_MICROROS_H

#include <stdint.h>
#include <stdbool.h>
#include "interfaces/detector_interface.h"
#include "microros_client.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MICROROS_TASK_PERIOD_MS     (10U)   /**< 100 Hz executor spin loop */
#define MICROROS_DETECTOR_TICKS     (5U)    /**< 5 * 10 ms = 50 ms -> 20 Hz */
#define MICROROS_GRIPPER_TICKS      (10U)   /**< 10 * 10 ms = 100 ms -> 10 Hz */
#define MICROROS_HEARTBEAT_TICKS    (100U)  /**< 100 * 10 ms = 1000 ms -> 1 Hz */

/**
 * @brief Initialize micro-ROS task and client layers.
 */
void Task_MicroROS_Init(void);

/**
 * @brief FreeRTOS thread entry point for micro-ROS executor and transport management.
 * @param argument Unused.
 */
void StartTaskMicroROS(void *argument);

/**
 * @brief Check if micro-ROS agent connection is active.
 * @return true if connected.
 */
bool Task_MicroROS_IsConnected(void);

#ifdef __cplusplus
}
#endif

#endif /* TASK_MICROROS_H */
