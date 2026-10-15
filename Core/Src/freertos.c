#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "main.h"
#include "task_detector.h"
#include "task_gripper.h"
#include "task_microros.h"
static const osThreadAttr_t detector = {.name="Detector", .stack_size=1024, .priority=osPriorityAboveNormal};
static const osThreadAttr_t gripper = {.name="Gripper", .stack_size=768, .priority=osPriorityAboveNormal};
static const osThreadAttr_t link = {.name="PayloadLink", .stack_size=1536, .priority=osPriorityNormal};
void MX_FREERTOS_Init(void)
{
    Task_Detector_Init(); Task_Gripper_Init();
    if (!osThreadNew(StartTaskDetector, NULL, &detector) ||
        !osThreadNew(StartTaskGripper, NULL, &gripper) ||
        !osThreadNew(StartTaskMicroROS, NULL, &link)) Error_Handler();
}
void vApplicationGetIdleTaskMemory(StaticTask_t **tcb, StackType_t **stack, uint32_t *size)
{
    static StaticTask_t idle;
    static StackType_t storage[configMINIMAL_STACK_SIZE];
    *tcb=&idle; *stack=storage; *size=configMINIMAL_STACK_SIZE;
}
void vApplicationGetTimerTaskMemory(StaticTask_t **tcb, StackType_t **stack, uint32_t *size)
{
    static StaticTask_t timer;
    static StackType_t storage[configTIMER_TASK_STACK_DEPTH];
    *tcb=&timer; *stack=storage; *size=configTIMER_TASK_STACK_DEPTH;
}
