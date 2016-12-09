#ifndef __STARTTASK_H
#define __STARTTASK_H
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"


//开始
//任务优先级
#define START_TASK_PRIO		5
//任务堆栈大小	
#define START_STK_SIZE 		50
//任务句柄
TaskHandle_t StartTask_Handler;
//任务函数
void start_task(void *pvParameters);

//LED0
//任务优先级
#define LED0_TASK_PRIO		5
//任务堆栈大小	
#define LED0_STK_SIZE 		50  
//任务句柄
TaskHandle_t LED0Task_Handler;
//任务函数
void led0_task(void *pvParameters);

//MPU6050
//任务优先级
#define MPU6050_TASK_PRIO		2
//任务堆栈大小	
#define MPU6050_STK_SIZE 		512
//任务句柄
TaskHandle_t MPU6050Task_Handler;
//任务函数
void mpu_task(void *pvParameters);

//can
//任务优先级
#define CAN_TASK_PRIO		3
//任务堆栈大小	
#define CAN_STK_SIZE 		128
//任务句柄
TaskHandle_t CANTask_Handler;
//任务函数
void can_task(void *pvParameters);

//mavlink
//任务优先级
#define MAV_TASK_PRIO		2
//任务堆栈大小	
#define MAV_STK_SIZE 		256
//任务句柄
TaskHandle_t MAVTask_Handler;
//任务函数
void mavlink_task(void *pvParameters);


#endif

