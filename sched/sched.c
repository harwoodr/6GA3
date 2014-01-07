#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <native/task.h>
#include <native/timer.h>
#include <rtdk.h>
#include <sys/io.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <native/mutex.h>
#include <errno.h>

#define HIGH 52	// high priority 
#define MID 51	// medium priority
#define LOW 50	// low priority 

#define SLIDER 0	// task id for slide pot reading
#define MOTOR 1		// task id for motor settings
#define REFERENCE 2	// task id for reference accelerometer reading

#define	KP 0
#define	KI 1
#define	KD 2

int PID[3] = {25,0,0};

#define NUMTASKS 3

RT_TASK	tasks[NUMTASKS];
RT_TASK_INFO tasks_info[NUMTASKS];
RT_TASK loop;
RT_TASK_INFO loop_info;
const char *task_names[NUMTASKS] = {"slider_task","motor_task","reference_task"};
#define	SPERIOD 500000000	// 5s period for slide pot reading
#define	MPERIOD 500000000	// 1s period for motor settings
#define	RPERIOD 500000000	// 3s period for reference accelerometer reading

#define SSTART	300000000	// 4s delay in starting task
#define MSTART	200000000	// 1s delay in starting task
#define RSTART	100000000	// 0s delay in starting task
RTIME task_periods[NUMTASKS] = {SPERIOD,MPERIOD,RPERIOD};
RTIME task_starts[NUMTASKS] = {SSTART,MSTART,RSTART};
int task_priorities[NUMTASKS] = {LOW,HIGH,HIGH};
int task_lasttime[NUMTASKS];

int addresses[3] = {5,3,4};	//i2c addresses
int handles[3];			// i2c file handles
RT_MUTEX i2c_mutex;		// Mutually Exclusive access to i2c bus.
#define I2CDELAY 50000000

int err;
int sliders[3];
int refs[3];

void errors(){
	if(err == -1){
		rt_fprintf(stdout, "OOPS: %s\r\n", strerror(errno));
		err == 0;
	}
}

void slider_task(){
	while (1) {
		int i,average;	
		//acquire the i2c mutex
		rt_mutex_acquire(&i2c_mutex,TM_INFINITE);
		//get the slider values	
		for(i=0;i<3;i++){
			sliders[i]=wiringPiI2CReadReg8(handles[SLIDER],i);
			if(sliders[i]==-1){
				err=-1;
				errors();
			}
			rt_printf("slider %d = %d\n\r",i,sliders[i]);
			rt_task_sleep(I2CDELAY);
		}	
		average = (sliders[0]+sliders[1]+sliders[2])/3;
		rt_printf("slider...average = %d\n\r",average);
		if (average>148){
			rt_printf("Correcting motor positions upwards.\n\r");
			err = wiringPiI2CWriteReg8(handles[MOTOR],4,75);
			errors();
			rt_task_sleep(I2CDELAY);
		} else if (average<108){
			rt_printf("Correcting motor positions downwards.\n\r");
			err = wiringPiI2CWriteReg8(handles[MOTOR],5,25);
			errors();
			rt_task_sleep(I2CDELAY);
		}
		//release mutex
		rt_mutex_release(&i2c_mutex);	

		rt_task_wait_period(NULL);
	}
}

void motor_task(){
	while (1) {
		int i,accs[3];
		//acquire the i2c mutex
		rt_mutex_acquire(&i2c_mutex,TM_INFINITE);
/*
		for(i=0;i<3;i++){
			err = wiringPiI2CWriteReg8(handles[MOTOR],0,i);
			errors();
			accs[i]=wiringPiI2CReadReg8(handles[MOTOR],i);
			if(accs[i]==-1){
				err=-1;
				errors();
			}		
			rt_printf("motor accelerometers %d = %d\n\r",i,accs[i]);
			rt_task_sleep(I2CDELAY);
		}
*/
		//set set points from reference accelerometers - we don't currently use Z axis though
		for(i=0;i<2;i++){
			err = wiringPiI2CWriteReg8(handles[MOTOR],i+1,refs[i]);
			
			errors();
			rt_task_sleep(I2CDELAY);
		}
		//release mutex
		rt_mutex_release(&i2c_mutex);
		rt_printf("motor...\n\r");
		rt_task_wait_period(NULL);
	}
}

void reference_task(){
	int i;
	while (1) {
		//acquire the i2c mutex
		rt_mutex_acquire(&i2c_mutex,TM_INFINITE);
		//get the reference acceleromter values
		for(i=0;i<3;i++){
			refs[i]=wiringPiI2CReadReg8(handles[REFERENCE],i);
			if(refs[i]==-1){
				err=-1;
				errors();
			}
			
			rt_printf("reference accelerometers %d = %d\n\r",i,refs[i]);
			rt_task_sleep(I2CDELAY);
		}
		//rt_printf("reference...\n\r");
		//release mutex
		rt_mutex_release(&i2c_mutex);
		rt_task_wait_period(NULL);
	}
}

void loops(){
	while (1) {
	int i;
	rt_printf("\n\n\rID\tTask\t\tBase\tCurrent\tContext\tExec\tMode\tNext\tStatus\n\r");
	rt_printf("\t\tPri\tPri\t\tSwitch\tTime\tSwitch\tRun\n\r");
	for(i=0;i<NUMTASKS;i++){
		rt_task_inquire(&tasks[i],&tasks_info[i]);
		rt_printf("%d\t%s\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n\r", i, tasks_info[i].name, tasks_info[i].bprio, tasks_info[i].cprio, tasks_info[i].ctxswitches, tasks_info[i].exectime, tasks_info[i].modeswitches, tasks_info[i].relpoint, tasks_info[i].status);
	}		

	rt_task_inquire(&loop,&loop_info);
	rt_printf("%d\t%s\t\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n\r", 3, loop_info.name, loop_info.bprio, loop_info.cprio, loop_info.ctxswitches, loop_info.exectime, loop_info.modeswitches, loop_info.relpoint, loop_info.status);
		rt_task_wait_period(NULL);
	}
}

void *functions[NUMTASKS] = {slider_task,motor_task,reference_task};

int init_xenomai() {
	int i;
	/* Avoids memory swapping for this program */
	mlockall(MCL_CURRENT|MCL_FUTURE);
	
	// create mutex for i2c communications
	rt_mutex_create(&i2c_mutex,"i2cMutex");
	for(i=0;i<NUMTASKS;i++){
		rt_printf("Initalizing task %s\n\r",task_names[i]);
		task_lasttime[i] = rt_timer_read();
		if (rt_task_create( &tasks[i],		/* task descriptor */
			task_names[i],			/* name */
			0,				/* 0 = default stack size */
			task_priorities[i],		/* priority */
			0 				/* mode flags */

			)!=0)
		{
			printf("rt_task_create error\n");
			return 1;
		}
		rt_printf("Setting task %s to periodic.\n\r",task_names[i]);
		rt_task_set_periodic(&tasks[i], rt_timer_read()+task_starts[i], task_periods[i]);
		rt_printf("Starting task %s.\n\r",task_names[i]);
		rt_task_start(&tasks[i],functions[i],NULL);
		
	}
	rt_printf("Initializing task Loop\n\r");
	if (rt_task_create( &loop,		/* task descriptor */
		"Loop",				/* name */
		0,				/* 0 = default stack size */
		10,				/* priority */
		0 				/* mode flags */
		)!=0)
	{
		printf("rt_task_create error\n");
		return 1;
	}
	rt_printf("Setting task Loop to periodic.\n\r");
	rt_task_set_periodic(&loop, rt_timer_read()+5000000000, 5000000000);
	rt_printf("Starting task Loop.\n\r");
	rt_task_start(&loop,loops,NULL);
	return 0;
}

void init_i2c() {
	int average,i;
	rt_printf("Initializing wiringPi.\n\r");
	wiringPiSetup();
	for(i=0;i<NUMTASKS;i++){
		rt_printf("Initializing wiringPi i2c for task %s.\n\r",task_names[i]);
		handles[i]=wiringPiI2CSetup(addresses[i]);
	}

	//set motor setpoints to 0
	rt_printf("Setting initial motor setpoints.\n\r");
	err = wiringPiI2CWriteReg8(handles[MOTOR],2,0);
	errors();
	rt_task_sleep(I2CDELAY);
	err = wiringPiI2CWriteReg8(handles[MOTOR],3,0);
	errors();
	rt_task_sleep(I2CDELAY);
	//check slide pot positions
	rt_printf("Checking slide pot positions.\n\r");
	for(i=0;i<3;i++){
		sliders[i]=wiringPiI2CReadReg8(handles[SLIDER],i);
		if(sliders[i]==-1){
			err=-1;
			errors();
		}
		rt_task_sleep(I2CDELAY);
	}
	//if sliders are significantly off center - perform slight correction
	average = (sliders[0]+sliders[1]+sliders[2])/3;
	if (average>148){
		rt_printf("Correcting motor positions upwards.\n\r");
		err = wiringPiI2CWriteReg8(handles[MOTOR],4,75);
		errors();
		rt_task_sleep(I2CDELAY);
	} else if (average<108){
		rt_printf("Correcting motor positions downwards.\n\r");
		err = wiringPiI2CWriteReg8(handles[MOTOR],5,25);
		errors();
		rt_task_sleep(I2CDELAY);
	}	
/*
	//set initial PID values
	rt_printf("Setting initial PID values.\n\r");
	for(i=0;i<3;i++){
		err = wiringPiI2CWriteReg8(handles[MOTOR],7+i,PID[i]);
		errors();
		rt_printf("Setting value %d via command %d to \n\r",i,7+i,PID[i]);
		rt_task_sleep(I2CDELAY);
	}	
*/	
	//turn on PID
	rt_printf("Activating PIDs.\n\r");
	err = wiringPiI2CWriteReg8(handles[MOTOR],6,1);
	errors();
	rt_task_sleep(I2CDELAY);
}



int main(int argc, char* argv[]){
	/* Perform auto-init of rt_print buffers if the task doesn't do so */
	rt_print_auto_init(1);
	rt_printf("Starting.\n\r");
	int i;
	// set up wiringPi i2c comms
	init_i2c();
	rt_printf("i2C init complete.\n\r");
	// code to set things to run xenomai
	init_xenomai();
	rt_printf("Xenomai init complete.\n\r");


	while(1){
	pause();
		
	}



}

void cleanup (void)
{
	int i;
	rt_printf("Deleting mutex.\n\r");
	rt_mutex_delete(&i2c_mutex);
	for(i=0;i<NUMTASKS;i++){
		rt_printf("Deleting task %s.\n\r",task_names[i]);
		rt_task_delete(&tasks[i]);
	}
	rt_printf("Deleting task Loop.\n\r");
	rt_task_delete(&loop);

}

