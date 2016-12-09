#ifndef __STARTTASK_H
#define __STARTTASK_H
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"


//��ʼ
//�������ȼ�
#define START_TASK_PRIO		5
//�����ջ��С	
#define START_STK_SIZE 		50
//������
TaskHandle_t StartTask_Handler;
//������
void start_task(void *pvParameters);

//LED0
//�������ȼ�
#define LED0_TASK_PRIO		5
//�����ջ��С	
#define LED0_STK_SIZE 		50  
//������
TaskHandle_t LED0Task_Handler;
//������
void led0_task(void *pvParameters);

//MPU6050
//�������ȼ�
#define MPU6050_TASK_PRIO		2
//�����ջ��С	
#define MPU6050_STK_SIZE 		512
//������
TaskHandle_t MPU6050Task_Handler;
//������
void mpu_task(void *pvParameters);

//can
//�������ȼ�
#define CAN_TASK_PRIO		3
//�����ջ��С	
#define CAN_STK_SIZE 		128
//������
TaskHandle_t CANTask_Handler;
//������
void can_task(void *pvParameters);

//mavlink
//�������ȼ�
#define MAV_TASK_PRIO		2
//�����ջ��С	
#define MAV_STK_SIZE 		256
//������
TaskHandle_t MAVTask_Handler;
//������
void mavlink_task(void *pvParameters);


#endif

