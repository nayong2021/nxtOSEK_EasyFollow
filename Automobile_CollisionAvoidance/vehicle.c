/* rms.c */ 
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

#define S1 NXT_PORT_S1
#define S2 NXT_PORT_S2
#define PA NXT_PORT_A
#define PB NXT_PORT_B
#define PC NXT_PORT_C
#define RUNTIME_CONNECTION

/* OSEK declarations */
DeclareCounter(SysTimerCnt);
DeclareEvent(SonicActivate);
DeclareResource(SpeedResource);
DeclareTask(IdleTask);
DeclareTask(BtReceiver);
DeclareTask(DistSensing);
DeclareTask(Operator);

/* LEJOS OSEK hooks */

void ecrobot_device_initialize(void)
{
#ifndef RUNTIME_CONNECTION
	ecrobot_init_bt_slave("LEJOS-OSEK");
#endif
	ecrobot_init_sonar_sensor(S1);
	ecrobot_init_sonar_sensor(S2);
	nxt_motor_set_speed(PA, 0, 1);
	nxt_motor_set_speed(PB, 0, 1);
	nxt_motor_set_speed(PC, 0, 1);
}

void ecrobot_device_terminate(void)
{
	ecrobot_term_sonar_sensor(S1);
	ecrobot_term_sonar_sensor(S2);
	nxt_motor_set_speed(PA, 0, 1);
	nxt_motor_set_speed(PB, 0, 1);
	nxt_motor_set_speed(PC, 0, 1);
	ecrobot_term_bt_connection();
}

/* LEJOS OSEK hook to be invoked from an ISR in category 2 */
void user_1ms_isr_type2(void){
	StatusType ercd;
	ercd = SignalCounter(SysTimerCnt);
	/* Increment OSEK Alarm Counter */
	if(ercd != E_OK)
	{
		ShutdownOS(ercd);
	}
}
int status[5] = {0, 0, 0, 0, 0}; //processed signal received from Bluetooth Master
int previous_brake = 0, mode_enable = 0; //previous_brake : previous signal of status[4], mode_enable : flag to show whether mode is on or not
int velocity = 0; //speed of rear wheels

TASK(BtReceiver){
	static U8 bt_receive_buf[32];
	
	ecrobot_read_bt_packet(bt_receive_buf, 32);
	GetResource(SpeedResource);
	for (int i = 3; i < 8; i++)
		status[i-3] = (int)bt_receive_buf[i];
	status[0] = status[0] == 1 ? -1 : (status[0] == 2 ? 1 : 0);
	status[3] = 2 - status[3];
	if(status[4] == 1)
		velocity = 0; //if button A pushed set velocity to 0
	if(previous_brake != 2 && status[4] == 2)
		mode_enable = 1 - mode_enable; //when the moment of button B pushed change mode_enable
	previous_brake = status[4];
	ReleaseResource(SpeedResource);
	display_clear(1);	
	display_goto_xy(0, 0);
	for (int i = 0; i < 5; i++)
		display_int(status[i], 2);
	display_goto_xy(0, 1);
	display_int(previous_brake, 2);
	display_int(mode_enable, 2);
	display_int(velocity, 2);
	display_update(); //display used to debugging
	TerminateTask();
}

TASK(DistSensing){
	int front_dist[7], back_dist[7], i, j, temp;
	while(1){
		for (i = 0; i < 7; i++){
			WaitEvent(SonicActivate);
			ClearEvent(SonicActivate);
			front_dist[i] = ecrobot_get_sonar_sensor(S2);
			WaitEvent(SonicActivate);
			ClearEvent(SonicActivate);
			back_dist[i] = ecrobot_get_sonar_sensor(S1);
		} //sense front, back dist for 7 times
		for (i = 0; i < 7; i++){
			for(j = i; j < 7; j++){
				if(front_dist[i] < front_dist[j]){
					temp = front_dist[i];
					front_dist[i] = front_dist[j];
					front_dist[j] = temp;
				}
				if(back_dist[i] < back_dist[j]){
					temp = back_dist[i];
					back_dist[i] = back_dist[j];
					back_dist[j] = temp;
				}
			}
		} //sort front_dist, back_dist array
		GetResource(SpeedResource);
		if(status[4] != 1){ //when button A is not pushed get in to setting velocity code
			if(mode_enable == 1){ //if crash avoid mode is enable 
				if(status[0] == -1){
						
					velocity = front_dist[3] - (status[2] == 2 ? 20 : 40);
					velocity = velocity < 50 ? velocity : 50;
					velocity = velocity > 0 ? velocity : 0;
					velocity *= -1;
				}
				else{
					velocity = back_dist[3] - (status[2] == 2 ? 20 : 40);
					velocity = velocity < 50 ? velocity : 50;
					velocity = velocity > 0 ? velocity : 0;
				} //set velocity due to distance and direction of car
			}
			else velocity = status[0] == -1 ? -50 : 50; //if crash avoid mode disabled. set velocity to maximum
			if (status[2] == 2) velocity /= 2; //if car speed set to slow divide velocity to half
		}
		ReleaseResource(SpeedResource);
	}
}

TASK(Operator){
	if(ecrobot_get_bt_status() == BT_STREAM){ //if bt is connected
		if(velocity != 0){
			nxt_motor_set_speed(PB, velocity, 1);
			nxt_motor_set_speed(PC, velocity, 1);
		} //if velocity is not 0 set brake flag to 1. If we set brake flag to 0, motor makes abnormal operation. So we made this solution to handle this situation.
		else{
			nxt_motor_set_speed(PB, velocity, status[3]);
			nxt_motor_set_speed(PC, velocity, status[3]);
		} //if velocity == 0 then set brake flag to input status
		if(status[1] == 3){ //when turn left situation

			if(nxt_motor_get_count(PA) >= -40){ //when motor degree didn't exceed maximum turn left degree

				nxt_motor_set_speed(PA, -15, 1);
			}
			else{ //when motor degree exceeded maximum turn left degree
				nxt_motor_set_speed(PA, 0, 1);
			}
		}
		else if(status[1] == 4){ //when turn right situation

			if(nxt_motor_get_count(PA) <= 40){ //when motor degree didn't exceed maximum turn right degree

				nxt_motor_set_speed(PA, 15, 1);
			}
			else{ //when motor degree exceeded maximum turn right degree
				nxt_motor_set_speed(PA, 0, 1);
			}
		}

		if(status[1] == 0){ //to handle to neutrality
			if(nxt_motor_get_count(PA) > 0){ //if motor degree is bigger than 0
				nxt_motor_set_speed(PA, -15, 1); //set motor speed to -15
			}
			else if(nxt_motor_get_count(PA) < 0){ //if motor degree is smaller than 0
				nxt_motor_set_speed(PA, 15, 1); //set motor speed to 15
			}
		}
	}
	TerminateTask();
}

TASK(IdleTask){
	while(1)
	{
#ifdef RUNTIME_CONNECTION
		ecrobot_init_bt_slave("LEJOS-OSEK");
#endif
	}
}
