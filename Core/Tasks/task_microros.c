/**
 * @file task_microros.c
 * @brief micro-ROS Client Task over UART DMA implementation for STM32_Auxiliary.
 */

#include "task_microros.h"
#include "microros_client.h"
#include "task_detector.h"
#include "task_gripper.h"
#include "status_beacon_driver.h"
#include "cmsis_os.h"
#include <string.h>

static void on_gripper_command_received(int8_t cmd)
{
    Task_Gripper_SendCommand((int32_t)cmd);
}

static void on_beacon_alert_received(bool alert)
{
    Status_Beacon_SetRemoteAlert(alert);
}

void Task_MicroROS_Init(void)
{
    MicroROS_Client_RegisterGripperCmdCallback(on_gripper_command_received);
    MicroROS_Client_RegisterBeaconAlertCallback(on_beacon_alert_received);
}

void StartTaskMicroROS(void *argument)
{
    (void)argument;

    Task_MicroROS_Init();
    MicroROS_Client_Init();

    uint32_t detector_tick_cnt = 0;
    uint32_t gripper_tick_cnt = 0;
    uint32_t heartbeat_tick_cnt = 0;

    for (;;)
    {
        /* 1. Spin subscriber dispatcher on UART circular buffer */
        MicroROS_Client_SpinSome(5);

        /* 2. Publish /metal_detector/reading at 20 Hz (every 50 ms) */
        if (++detector_tick_cnt >= MICROROS_DETECTOR_TICKS)
        {
            detector_tick_cnt = 0;
            float32_msg_t det_msg;
            det_msg.data = Task_Detector_GetIntensity();
            MicroROS_Client_PublishDetectorReading(&det_msg);
        }

        /* 3. Publish /gripper/status at 10 Hz (every 100 ms) */
        if (++gripper_tick_cnt >= MICROROS_GRIPPER_TICKS)
        {
            gripper_tick_cnt = 0;
            int8_msg_t grip_msg;
            grip_msg.data = Task_Gripper_GetStatus();
            MicroROS_Client_PublishGripperStatus(&grip_msg);
        }

        /* 4. Publish /stm32_heartbeat at 1 Hz (every 1000 ms) */
        if (++heartbeat_tick_cnt >= MICROROS_HEARTBEAT_TICKS)
        {
            heartbeat_tick_cnt = 0;
            bool_msg_t hb_msg;
            hb_msg.data = true;
            MicroROS_Client_PublishHeartbeat(&hb_msg);
        }

        /* 5. 10 ms base loop period */
        osDelay(MICROROS_TASK_PERIOD_MS);
    }
}

bool Task_MicroROS_IsConnected(void)
{
    return MicroROS_Client_IsConnected();
}
