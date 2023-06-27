#ifndef __UTILFNS_H
#define __UTILFNS_H



#define STATE_IDLE 0
#define STATE_RUN 1

//SECTIONS in each RUN
#define OFF 0
#define START 1
#define RAMP_UP 2
#define PK_RPM_WAIT 3
#define	RAMP_DOWN 4
#define	STOP 5


#define MAX_RPM 1000
#define MIN_RPM 0
#define RAMP_UP_RATE 5
#define RAMP_DOWN_RATE 5



#define START_DELAY_TIME 4
#define PK_PWM_WAIT_TIME 50
#define STOP_DELAY_TIME 5


#define DISABLE_D 1
#define ENABLE_D 2

char Read_inp1Pin(void);
char Read_inp2Pin(void);
void MotorDrive(char index);


#endif

