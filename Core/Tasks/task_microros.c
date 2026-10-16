/**
 * @file task_microros.c
 * @brief micro-ROS Client Task over UART DMA implementation for STM32_Auxiliary.
 */

#include "task_microros.h"
#include "microros_client.h"
#include "task_detector.h"
#include "task_gripper.h"
#include "cmsis_os.h"
#include <string.h>
#include "main.h"

static void on_gripper_command_received(int8_t cmd)
{
    Task_Gripper_SendCommand((int32_t)cmd);
}

void Task_MicroROS_Init(void)
{
    MicroROS_Client_RegisterGripperCmdCallback(on_gripper_command_received);
    /* Remote beacon frames remain parseable but cannot control the local relay. */
    MicroROS_Client_RegisterBeaconAlertCallback(NULL);
}

void StartTaskMicroROS(void *argument)
{
    (void)argument;

    MicroROS_Client_Init();
    Task_MicroROS_Init();

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
            detector_reading_t sample;
            if (Task_Detector_GetLatestReading(&sample) && sample.sample_id>0 &&
                HAL_GetTick()-sample.timestamp_ms<250U) {
                float32_msg_t det_msg = {.data=sample.signal_intensity};
                MicroROS_Client_PublishDetectorReading(&det_msg);
            }
        }

        /* 3. Publish /gripper/status at 10 Hz (every 100 ms) */
        if (++gripper_tick_cnt >= MICROROS_GRIPPER_TICKS)
        {
            gripper_tick_cnt = 0;
            int8_msg_t grip_msg;
            grip_msg.data = Task_Gripper_GetStatus();
            MicroROS_Client_PublishGripperStatus(&grip_msg);
        }

        /* 4. Publish /payload/heartbeat at 1 Hz (every 1000 ms) */
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
