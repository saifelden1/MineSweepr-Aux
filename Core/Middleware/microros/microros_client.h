/**
 * @file microros_client.h
 * @brief micro-ROS Client & Topic Publisher/Subscriber Manager for STM32_Auxiliary.
 *
 * Manages serialization, subscription, and dispatch for ROS 2 topics:
 *   Publishers:
 *     - /metal_detector/reading (0x10, 20 Hz, std_msgs/msg/Float32)
 *     - /gripper/status         (0x11, 10 Hz, std_msgs/msg/Int8)
 *     - /stm32_heartbeat        (0x12, 1 Hz,  std_msgs/msg/Bool)
 *   Subscribers:
 *     - /gripper/command        (0x20, std_msgs/msg/Int8)
 *     - /beacon/alert           (0x21, std_msgs/msg/Bool)
 */

#ifndef MICROROS_CLIENT_H
#define MICROROS_CLIENT_H

#include "microros_msgs.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*gripper_cmd_callback_t)(int8_t cmd);
typedef void (*beacon_alert_callback_t)(bool alert);

typedef struct {
    uint32_t detector_published_count;
    uint32_t gripper_status_published_count;
    uint32_t heartbeat_published_count;
    uint32_t gripper_cmd_received_count;
    uint32_t beacon_alert_received_count;
    uint32_t framing_error_count;
    bool agent_connected;
} microros_aux_stats_t;

/**
 * @brief Initialize micro-ROS client, transport, publishers, and subscribers.
 * @return true on success.
 */
bool MicroROS_Client_Init(void);

/**
 * @brief Register application callback for incoming /gripper/command messages.
 * @param cb Callback function accepting int8_t command (0=Neutral, 1=Open, 2=Grip).
 */
void MicroROS_Client_RegisterGripperCmdCallback(gripper_cmd_callback_t cb);

/**
 * @brief Register application callback for incoming /beacon/alert messages.
 * @param cb Callback function accepting boolean alert state.
 */
void MicroROS_Client_RegisterBeaconAlertCallback(beacon_alert_callback_t cb);

/**
 * @brief Process incoming packets from UART DMA circular buffer.
 * @param timeout_ms Max spin duration in milliseconds.
 */
void MicroROS_Client_SpinSome(uint32_t timeout_ms);

/**
 * @brief Publish /metal_detector/reading topic message (std_msgs/msg/Float32).
 * @param msg Pointer to Float32 message.
 * @return true on success.
 */
bool MicroROS_Client_PublishDetectorReading(const float32_msg_t *msg);

/**
 * @brief Publish /gripper/status topic message (std_msgs/msg/Int8).
 * @param msg Pointer to Int8 message.
 * @return true on success.
 */
bool MicroROS_Client_PublishGripperStatus(const int8_msg_t *msg);

/**
 * @brief Publish /stm32_heartbeat topic message (std_msgs/msg/Bool).
 * @param msg Pointer to Bool message.
 * @return true on success.
 */
bool MicroROS_Client_PublishHeartbeat(const bool_msg_t *msg);

/**
 * @brief Query current client statistics and health.
 * @param out_stats Destination pointer for statistics struct.
 */
void MicroROS_Client_GetStats(microros_aux_stats_t *out_stats);

/**
 * @brief Check if transport and agent connection is active.
 * @return true if active.
 */
bool MicroROS_Client_IsConnected(void);

#ifdef __cplusplus
}
#endif

#endif /* MICROROS_CLIENT_H */
