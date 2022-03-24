/* sonartest.c */ 
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>
#define PA NXT_PORT_A
#define PB NXT_PORT_B
#define PC NXT_PORT_C
#define PS1 NXT_PORT_S1
#define PS2 NXT_PORT_S2
#define PS3 NXT_PORT_S3
#define PS4 NXT_PORT_S4
#define PI 3.14159265
#define RAD 57.29
/*
 * LEJOS NXJ development team fixed I2C issue on port S4.
 * This sample also verifies the updated I2C comm. functionality
 */
#define PORT_IN_USE NXT_PORT_S4

/* OSEK declarations */
DeclareCounter(SysTimerCnt);

DeclareTask(sensing);
DeclareTask(drive);
DeclareTask(back);
DeclareTask(stop);
DeclareTask(steering);

DeclareEvent(DREvent);
DeclareEvent(BAEvent);
DeclareEvent(STEvent);

DeclareResource(sen_resource);

/* LEJOS OSEK hooks */
void ecrobot_device_initialize()
{
	ecrobot_init_sonar_sensor(NXT_PORT_S1);
	ecrobot_init_sonar_sensor(NXT_PORT_S2);
}
void ecrobot_device_terminate()
{
	ecrobot_term_sonar_sensor(NXT_PORT_S1);
	ecrobot_term_sonar_sensor(NXT_PORT_S2);
}

/* LEJOS OSEK hook to be invoked from an ISR in category 2 */
void user_1ms_isr_type2(void)
{
  StatusType ercd;

  ercd = SignalCounter(SysTimerCnt); /* Increment OSEK Alarm Counter */
  if(ercd != E_OK)
  {
    ShutdownOS(ercd);
  }
}

int compare(const void *a, const void *b) // For quicksort algorithm
{
  int num1 = *(int *)a;
  int num2 = *(int *)b;

  if(num1 < num2)
    return -1;
  if(num1 > num2)
    return 1;

  return 0;
}

int counter = 0;
int l_buffer[7] = {0, };
int r_buffer[7] = {0, };

int l_sort[7], r_sort[7];

TASK(sensing) 
{

	int l_sonar = 0;
	int r_sonar = 0;
	
	l_sonar = ecrobot_get_sonar_sensor(PS1);
	r_sonar = ecrobot_get_sonar_sensor(PS2);


	GetResource(sen_resource);
	
	l_buffer[counter] = l_sonar;
	r_buffer[counter] = r_sonar;

	memcpy(l_sort, l_buffer, sizeof(int) * 7);
	memcpy(r_sort, r_buffer, sizeof(int) * 7);
	
		
	qsort(l_sort, sizeof(l_sort) / sizeof(int), sizeof(int), compare);	
	qsort(r_sort, sizeof(r_sort) / sizeof(int), sizeof(int), compare);

	// Median 값 추출
	int l = l_sort[3];
	int r = r_sort[3];
	
	counter++;
	if(counter == 7) counter = 0;

	ReleaseResource(sen_resource);

	if(l > 100 & r > 100) // 앞차량이 너무 먼 경우 스티어링 정렬
	{
		int now = nxt_motor_get_count(PA);
		while(now < -2 || now > 2)
		{
			if(now < 0) nxt_motor_set_speed(PA, 20, 1);
			else nxt_motor_set_speed(PA, -20, 1);
			now = nxt_motor_get_count(PA);
		}
		nxt_motor_set_speed(PA, 0, 1);


	}
	if(abs(l - r) < 2) // 두 센서 측정값에 차이가 거의 없는 경우 정렬 (오차범위 +- 1)
	{
		int now = nxt_motor_get_count(PA);
		while(now < -2 || now > 2)
		{
			if(now < 0) nxt_motor_set_speed(PA, 32, 1);
			else nxt_motor_set_speed(PA, -32, 1);
			now = nxt_motor_get_count(PA);
		}
		nxt_motor_set_speed(PA, 0, 1);
	}

	if((l > 25 || r > 25)) // 안전 거리 x ( 18 < x < 25 )
	{
		if(l > 150 && r > 150) // 차량이 너무 멀리 있는 경우 (돌발 상황) 에는 정지
		{
			SetEvent(stop, STEvent);
		}
		else 
		{
			SetEvent(drive, DREvent);	
		}
	}

	else if(l < 18 || r < 18) // 안전 거리 안쪽으로 들어오면 후진
	{
		SetEvent(back, BAEvent);
	}
	else // 안전 거리에 있는 경우 정지
	{
		SetEvent(stop, STEvent);
	}
	TerminateTask();

}

TASK(drive) // 전진
{
	while(WaitEvent(DREvent) == E_OK)
	{	
		if(l_sort[3] > 55 && r_sort[3] > 55){ // 앞차와의 거리가 적당히 멀면 고속
		nxt_motor_set_speed(PB, -45, 1);	
		nxt_motor_set_speed(PC, -45, 1);
		}	
		else { // 그렇지 않으면 저속
		nxt_motor_set_speed(PB, -31, 1);	
		nxt_motor_set_speed(PC, -31, 1);	
		}
		ClearEvent(DREvent);
	}
}
TASK(back) // 후진
{
	while(WaitEvent(BAEvent) == E_OK)
	{

		if(l_sort[3] < 10 && r_sort[3] < 10) { // 앞차와의 거리가 너무 가까우면 고속 후진
			nxt_motor_set_speed(PB, 41, 1);
			nxt_motor_set_speed(PC, 41, 1);
		}
		else {
			nxt_motor_set_speed(PB, 35, 1);
			nxt_motor_set_speed(PC, 35, 1);
		}
		ClearEvent(BAEvent);
	}
}

TASK(stop) // 정지
{
	while(WaitEvent(STEvent) == E_OK)
	{
		nxt_motor_set_speed(PB, 0, 1);
		nxt_motor_set_speed(PC, 0, 1);
		ClearEvent(STEvent);
	}
}

TASK(steering) // 스티어링 제어
{
		int l = l_sort[3];
		int r = r_sort[3];	
		if(l > 80) l = 80;
		if(r > 80) r = 80;		
		double degree = 0;
		double dis = l - r;
		double dis_sonar = 15;		
		int flag = 0;


		// 회전 방향 설정
		if(dis <= -2) flag = 1;
		else if(dis >= 2) flag = -1;
		
		// 회전각 구하기 ( Arctan algorithm )
		double radian = atan2(abs(dis), dis_sonar);
		degree = radian * (180 / PI);


		// 좌/우 회전 최대각 설정
		if(degree > 45) degree = 40;
		if(flag == -1 && degree > 45) degree = 37;

		int now = nxt_motor_get_count(PA);

		if(flag > 0 && now < degree) // 계산된 degree 만큼 좌회전
		{ 
			nxt_motor_set_speed(PA, 20, 1);
			
		}
		else if(flag < 0 && now > degree * flag) // 계산된 degree 만큼 우회전
		{
			nxt_motor_set_speed(PA, -20, 1);
		}   
		
		else nxt_motor_set_speed(PA, 0, 1); // 회전 X
		
		TerminateTask();

}