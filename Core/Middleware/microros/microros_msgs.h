/**
 * @file microros_msgs.h
 * @brief ROS 2 Message Definitions for STM32_Auxiliary micro-ROS Client.
 *
 * Defines standard message structures for:
 *   - std_msgs/msg/Float32 (/metal_detector/reading)
 *   - std_msgs/msg/Int8    (/gripper/status, /gripper/command)
 *   - std_msgs/msg/Bool    (/stm32_heartbeat, /beacon/alert)
 */

#ifndef MICROROS_MSGS_H
#define MICROROS_MSGS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* 1. Topic Identifiers for STM32_Auxiliary                                  */
/* ------------------------------------------------------------------------- */
#define TOPIC_ID_METAL_DETECTOR_READING (0x10U) /**< Pub: /metal_detector/reading (Float32 @ 20 Hz) */
#define TOPIC_ID_GRIPPER_STATUS         (0x11U) /**< Pub: /gripper/status (Int8 @ 10 Hz) */
#define TOPIC_ID_STM32_HEARTBEAT        (0x12U) /**< Pub: /stm32_heartbeat (Bool @ 1 Hz) */
#define TOPIC_ID_GRIPPER_COMMAND        (0x20U) /**< Sub: /gripper/command (Int8) */
#define TOPIC_ID_BEACON_ALERT           (0x21U) /**< Sub: /beacon/alert (Bool) */

/* ------------------------------------------------------------------------- */
/* 2. Message Payload Structures                                             */
/* ------------------------------------------------------------------------- */
#pragma pack(push, 1)

/** std_msgs/msg/Float32 */
typedef struct {
    float data; /**< Normalized coil intensity [0.0f .. 1.0f] */
} float32_msg_t;

/** std_msgs/msg/Int8 */
typedef struct {
    int8_t data; /**< Gripper discrete state: 0=Neutral, 1=Open, 2=Grip */
} int8_msg_t;

/** std_msgs/msg/Bool */
typedef struct {
    bool data; /**< Boolean status / alarm flag */
} bool_msg_t;

#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif /* MICROROS_MSGS_H */
