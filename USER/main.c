#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "start_task.h"
#include "led.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "can.h"
#include "lcd.h"
#include "dcm.h"
#include "uart.h"

#include "mavlink.h"
#include "checksum.h"
#include "protocol.h"
#include "common.h"


short aacx = 0,aacy = 0,aacz = 0;								//加速度传感器原始数据
short gyrox = 0,gyroy = 0,gyroz = 0;						//陀螺仪原始数据
short temp = 0;																	//温度
static uint16_t iddata = 0xc0;
float roll,pitch,yaw;														//姿态角
float m_roll,m_pitch,m_yaw;

//句柄
xQueueHandle MsgQueue;

int main(void)
{ 

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
	delay_init(168);				//初始化延时函数
	uart_init(115200);     	//初始化串口
	LED_Init();		        	//初始化LED端口
	LCD_Init();							//LCD初始化
	MPU_Init();							//MPU6050初始化
	mpu_dmp_init();					//dmp库初始化
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_Normal);//CAN初始化,波特率500Kbps
	
	//创建队列
	MsgQueue = xQueueCreate(10, sizeof(float));
	
	//创建开始任务
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄              
    vTaskStartScheduler();          //开启任务调度
}
 
//开始任务任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
    //创建LED0任务
    xTaskCreate((TaskFunction_t )led0_task,     	
                (const char*    )"led0_task",   	
                (uint16_t       )LED0_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )LED0_TASK_PRIO,	
                (TaskHandle_t*  )&LED0Task_Handler);   
								
    //创建MPU6050任务
    xTaskCreate((TaskFunction_t )mpu_task,     
                (const char*    )"mpu_task",   
                (uint16_t       )MPU6050_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )MPU6050_TASK_PRIO,
                (TaskHandle_t*  )&MPU6050Task_Handler);
		//创建CAN任务
    xTaskCreate((TaskFunction_t )can_task,     
                (const char*    )"can_task",   
                (uint16_t       )CAN_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )CAN_TASK_PRIO,
                (TaskHandle_t*  )&CANTask_Handler);

		//创建MAV任务
    xTaskCreate((TaskFunction_t )mavlink_task,     
                (const char*    )"mavlink_task",   
                (uint16_t       )MAV_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )MAV_TASK_PRIO,
                (TaskHandle_t*  )&MAVTask_Handler);

    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}

//LED0任务函数 
void led0_task(void *pvParameters)
{
    while(1){
        LED0=~LED0;
        vTaskDelay(1000);
    }
}

//mpu任务
void mpu_task(void *pvParameters)
{
	while(1){
		LED1 = ~LED1;
		temp=MPU_Get_Temperature();								//得到温度值
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
		
		IMUupdate(aacx,aacy,aacz,gyrox,gyroy,gyroz,&roll,&pitch,&yaw);	//得到姿态角
		
		//往队列写参数
		xQueueSend(MsgQueue, (void *) &aacx, (portTickType) 10);
		
	//printf("roll = %.4f,pitch = %.4f,yaw = %.4f\r\n",roll,pitch,yaw);
		vTaskDelay(1000);
	}
}

//CAN任务
void can_task(void *pvParameters)
{
	float test_aacx;
	//message ID
	uint32_t gyro_id = 0x000007D0;			//0x00000080;
	uint32_t accel_id = 0x000007D1;			//0x00000080;
	while(1){	
		vTaskDelay(800); 																				//ms
		CAN1_Send_Msg(gyro_id,gyrox,gyroy,gyroz,iddata);				//can传输
		CAN1_Send_Msg(accel_id,aacx,aacy,aacz,iddata);					//can传输
		iddata++;																								//transfer ID
		if(iddata > 0xdf)
			iddata = 0xc0;
		
		
		xQueueReceive(MsgQueue, &test_aacx, portMAX_DELAY);
		printf("test = %.4f\r\n",test_aacx);
	}
}

//mavlink任务
void mavlink_task(void *pvParameters)
{	
	
	mavlink_heartbeat_t packet_in ={
		963497464,17,84,151,218,3
	};

	mavlink_heartbeat_t packet1;
	memset(&packet1,0,sizeof(packet1));
	packet1.custom_mode = packet_in.custom_mode;
	packet1.type = packet_in.type;
	packet1.autopilot = packet_in.autopilot;
	packet1.base_mode = packet_in.base_mode;
	packet1.system_status = packet_in.system_status;
	packet1.mavlink_version = packet_in.mavlink_version;
	while(1)
	{		
		mavlink_msg_heartbeat_send(MAVLINK_COMM_1,packet1.type , packet1.autopilot , packet1.base_mode , packet1.custom_mode , packet1.system_status );
		vTaskDelay(500);
	}
}




