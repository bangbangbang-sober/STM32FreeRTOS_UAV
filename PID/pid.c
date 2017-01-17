#include "pid.h"


//位置PID
#if 1

struct _pid{
	float SetSpeed; 			//设定值
	float ActualSpeed; 		//实际值
	float err; 						//偏差值
	float err_last; 			//上一个偏差值
	float Kp,Ki,Kd; 			//比例，积分，微分系数
	float voltage; 				//控制执行器的变量
	float integral;		    //积分值
}pid;

//项目中获取到的参数
void PID_init(){
	//printf("PID_init begin \n");
	pid.SetSpeed=0.0;
	pid.ActualSpeed=0.0;
	pid.err=0.0;
	pid.err_last=0.0;
	pid.voltage=0.0;
	pid.integral=0.0;
	pid.Kp=0.2;					//以下三个参数自己设定
	pid.Ki=0.015;			
	pid.Kd=0.2;				
	//printf("PID_init end \n");
}

float PID_realize(float speed){
	pid.SetSpeed = speed;												//设定值
	pid.err = pid.SetSpeed - pid.ActualSpeed;		//设定值-实际值
	pid.integral += pid.err;										//积分值，偏差累加
	pid.voltage = pid.Kp * pid.err + pid.Ki * pid.integral + pid.Kd * (pid.err - pid.err_last);
	pid.err_last = pid.err;											//上一个偏差值
	pid.ActualSpeed = pid.voltage * (float)1.0;	//算出实际值
	
	return pid.ActualSpeed;											//返回
}

#endif

//增量型pid
#if 0
struct _pid{
	float SetSpeed; 		//设定值
	float ActualSpeed; 	//实际值
	float err; 					//偏差值
	float err_next; 		//上一个偏差值
	float err_last; 		//最上前的偏差值
	float Kp,Ki,Kd; 		//系数
}pid;

void PID_init(){
	pid.SetSpeed=0.0;
	pid.ActualSpeed=0.0;
	pid.err=0.0;
	pid.err_last=0.0;
	pid.err_next=0.0;
	pid.Kp=0.2;
	pid.Ki=0.015;
	pid.Kd=0.2;
}

float PID_realize(float speed){
	pid.SetSpeed=speed;
	pid.err=pid.SetSpeed-pid.ActualSpeed;
	float
	incrementSpeed=pid.Kp*(pid.err-pid.err_next)+pid.Ki*pid.err+pid.Kd*(pid.err-2*pid.err_next+pid.err_last);
	pid.ActualSpeed+=incrementSpeed;
	pid.err_last=pid.err_next;
	pid.err_next=pid.err;
	return pid.ActualSpeed;
}
#endif

//积分分离的pid
#if 0
struct _pid{
	float SetSpeed; 			//设定值	//24V   1100-1900
	float ActualSpeed; 		//实际值
	float err; 						//偏差值
	float err_last; 			//上一个偏差值
	float Kp,Ki,Kd; 			//系数
	float voltage; 				//控制执行器的变量
	float integral;		    //积分值
}pid;

void PID_init(){
	printf("PID_init begin \n");
	pid.SetSpeed=0.0;
	pid.ActualSpeed=0.0;
	pid.err=0.0;
	pid.err_last=0.0;
	pid.voltage=0.0;
	pid.integral=0.0;
	pid.Kp=0.2;							//系数设定
	pid.Ki=0.04;						
	pid.Kd=0.2;								
	printf("PID_init end \n");
}

float PID_realize(float speed){
	pid.SetSpeed=speed;											//设定值
	pid.err=pid.SetSpeed-pid.ActualSpeed;		//设定值-实际值
	int index;
	if(abs(pid.err)>200)
	{
		index=0;
	}else{
		index=1;
		pid.integral+=pid.err;
	}
	pid.voltage=pid.Kp*pid.err+index*pid.Ki*pid.integral+pid.Kd*(pid.
	err-pid.err_last); 										

	pid.err_last=pid.err;									//上一个偏差值
	pid.ActualSpeed=pid.voltage*1.0;			//实际值
	return pid.ActualSpeed;								
}

#endif


//抗积分饱和pid
#if 0
struct _pid{
	float SetSpeed; 		//设定值
	float ActualSpeed; 	//实际值
	float err; 					//偏差值
	float err_last; 		//上一个偏差值
	float Kp,Ki,Kd; 		
	float voltage; 			//控制执行器的变量
	float integral; 		//积分值
	float umax;
	float umin;
}pid;

void PID_init(){
	printf("PID_init begin \n");
	pid.SetSpeed=0.0;
	pid.ActualSpeed=0.0;
	pid.err=0.0;
	pid.err_last=0.0;
	pid.voltage=0.0;
	pid.integral=0.0;
	pid.Kp=0.2;
	pid.Ki=0.1; 							//注意，和上几次相比，这里加大了积分环节的值
	pid.Kd=0.2;
	pid.umax=400;
	pid.umin=-200;
	printf("PID_init end \n");
}

float PID_realize(float speed){
	int index;
	pid.SetSpeed=speed;
	pid.err=pid.SetSpeed-pid.ActualSpeed;
	if(pid.ActualSpeed>pid.umax) 		//灰色底色表示抗积分饱和的实现
	{
		if(abs(pid.err)>200) 					
		{
		index=0;
		}else{
			index=1;
			if(pid.err<0)
			{
			pid.integral+=pid.err;
			}
		}
	}else if(pid.ActualSpeed<pid.umin){
		if(abs(pid.err)>200) 					//积分分离过程
		{
			index=0;
		}else{
			index=1;
			if(pid.err>0)
			{
				pid.integral+=pid.err;
			}
		}
	}else{
			if(abs(pid.err)>200) 				//积分分离过程
			{
				index=0;
			}else{
				index=1;
				pid.integral+=pid.err;
			}
		}
//	pid.voltage=pid.Kp*pid.err+index*pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);
pid.voltage=pid.Kp*pid.err+index*pid.Ki*pid.integral/2+pid.Kd*(pid.err-pid.err_last);//????
	pid.err_last=pid.err;
	pid.ActualSpeed=pid.voltage*1.0;
	return pid.ActualSpeed;
}

#endif



//变积分
#if 0
struct _pid{
	float SetSpeed; 			//set value
	float ActualSpeed; 		//actual value
	float err; 						//err
	float err_last; 			//the last err
	float Kp,Ki,Kd; 			
	float voltage; 				
	float integral;		    
}pid;

void PID_init(){
	printf("PID_init begin \n");
	pid.SetSpeed=0.0;
	pid.ActualSpeed=0.0;
	pid.err=0.0;
	pid.err_last=0.0;
	pid.voltage=0.0;
	pid.integral=0.0;
	pid.Kp=0.4;				
	pid.Ki=0.2;				
	pid.Kd=0.2;				
	printf("PID_init end \n");
}

float PID_realize(float speed){
	float index;
	pid.SetSpeed=speed;
	pid.err=pid.SetSpeed-pid.ActualSpeed;
	if(abs(pid.err)>200) 					//变积分过程
	{
		index=0.0;
	}else if(abs(pid.err)<180){
				index=1.0;
				pid.integral+=pid.err;
			}else{
					index=(200-abs(pid.err))/20;
					pid.integral+=pid.err;
				}
	pid.voltage=pid.Kp*pid.err+index*pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);
	pid.err_last=pid.err;
	pid.ActualSpeed=pid.voltage*1.0;
	return pid.ActualSpeed;
}

#endif


//test_code
#if 0
int main(){
	PID_init();
	int count=0;
	while(count<1000)
	{
	float speed=PID_realize(200.0);
	printf("%f\n",speed);
	count++;
	}
	return 0;
}

#endif





