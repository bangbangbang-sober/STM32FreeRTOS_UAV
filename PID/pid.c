#include "pid.h"


//λ��PID
#if 1

struct _pid{
	float SetSpeed; 			//�趨ֵ
	float ActualSpeed; 		//ʵ��ֵ
	float err; 						//ƫ��ֵ
	float err_last; 			//��һ��ƫ��ֵ
	float Kp,Ki,Kd; 			//���������֣�΢��ϵ��
	float voltage; 				//����ִ�����ı���
	float integral;		    //����ֵ
}pid;

//��Ŀ�л�ȡ���Ĳ���
void PID_init(){
	//printf("PID_init begin \n");
	pid.SetSpeed=0.0;
	pid.ActualSpeed=0.0;
	pid.err=0.0;
	pid.err_last=0.0;
	pid.voltage=0.0;
	pid.integral=0.0;
	pid.Kp=0.2;					//�������������Լ��趨
	pid.Ki=0.015;			
	pid.Kd=0.2;				
	//printf("PID_init end \n");
}

float PID_realize(float speed){
	pid.SetSpeed = speed;												//�趨ֵ
	pid.err = pid.SetSpeed - pid.ActualSpeed;		//�趨ֵ-ʵ��ֵ
	pid.integral += pid.err;										//����ֵ��ƫ���ۼ�
	pid.voltage = pid.Kp * pid.err + pid.Ki * pid.integral + pid.Kd * (pid.err - pid.err_last);
	pid.err_last = pid.err;											//��һ��ƫ��ֵ
	pid.ActualSpeed = pid.voltage * (float)1.0;	//���ʵ��ֵ
	
	return pid.ActualSpeed;											//����
}

#endif

//������pid
#if 0
struct _pid{
	float SetSpeed; 		//�趨ֵ
	float ActualSpeed; 	//ʵ��ֵ
	float err; 					//ƫ��ֵ
	float err_next; 		//��һ��ƫ��ֵ
	float err_last; 		//����ǰ��ƫ��ֵ
	float Kp,Ki,Kd; 		//ϵ��
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

//���ַ����pid
#if 0
struct _pid{
	float SetSpeed; 			//�趨ֵ	//24V   1100-1900
	float ActualSpeed; 		//ʵ��ֵ
	float err; 						//ƫ��ֵ
	float err_last; 			//��һ��ƫ��ֵ
	float Kp,Ki,Kd; 			//ϵ��
	float voltage; 				//����ִ�����ı���
	float integral;		    //����ֵ
}pid;

void PID_init(){
	printf("PID_init begin \n");
	pid.SetSpeed=0.0;
	pid.ActualSpeed=0.0;
	pid.err=0.0;
	pid.err_last=0.0;
	pid.voltage=0.0;
	pid.integral=0.0;
	pid.Kp=0.2;							//ϵ���趨
	pid.Ki=0.04;						
	pid.Kd=0.2;								
	printf("PID_init end \n");
}

float PID_realize(float speed){
	pid.SetSpeed=speed;											//�趨ֵ
	pid.err=pid.SetSpeed-pid.ActualSpeed;		//�趨ֵ-ʵ��ֵ
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

	pid.err_last=pid.err;									//��һ��ƫ��ֵ
	pid.ActualSpeed=pid.voltage*1.0;			//ʵ��ֵ
	return pid.ActualSpeed;								
}

#endif


//�����ֱ���pid
#if 0
struct _pid{
	float SetSpeed; 		//�趨ֵ
	float ActualSpeed; 	//ʵ��ֵ
	float err; 					//ƫ��ֵ
	float err_last; 		//��һ��ƫ��ֵ
	float Kp,Ki,Kd; 		
	float voltage; 			//����ִ�����ı���
	float integral; 		//����ֵ
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
	pid.Ki=0.1; 							//ע�⣬���ϼ�����ȣ�����Ӵ��˻��ֻ��ڵ�ֵ
	pid.Kd=0.2;
	pid.umax=400;
	pid.umin=-200;
	printf("PID_init end \n");
}

float PID_realize(float speed){
	int index;
	pid.SetSpeed=speed;
	pid.err=pid.SetSpeed-pid.ActualSpeed;
	if(pid.ActualSpeed>pid.umax) 		//��ɫ��ɫ��ʾ�����ֱ��͵�ʵ��
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
		if(abs(pid.err)>200) 					//���ַ������
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
			if(abs(pid.err)>200) 				//���ַ������
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



//�����
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
	if(abs(pid.err)>200) 					//����ֹ���
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





