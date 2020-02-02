/**
 * @file daiDrone_Ultrasonic.cpp
 *
 * Example app for ultrasonic distance  measurement with 4 sensors
 * (HRLV-MaxSonar-EZ4 & HRLV-MaxSonar-EZ0) connected to an Arduino
 * Nano that send data to FMU using I2C bus.
 *
 * @author Dan-Marius Dobrea
 * program developed for: HoverGames Challenge 1: Fight Fire with Flyers
  */

#include "daiDrone_Ultrasonic.h"
#include "ArduinoNano_I2Cdriver.h"
//#include <px4_config.h>
#include <px4_log.h>

#include <string.h>
//#include <stdio.h>

/* Variables */
static bool threadUs_should_exit = false;		/**< Daemon exit flag */
static bool threadUs_running     = false;		/**< Daemon status flag */
static int  deamon_Ultrasonic_task;				/**< Handle of deamon task / thread */

bool verbose_Us = false;
sem_t timer_sem_Us;

#define TIMER_US_SIGNAL 		29 	//[0, 31]
#define TIMER_SIGVALUE_INT_US	11

/* Main Thread */
int readingArduinoMini_Ultrasonic (int argc, char *argv[])
{
	// timer variables
	struct sigaction   	act;
	struct sigaction   	oact;
	struct sigevent    	notify;
	struct itimerspec  	timer;
    timer_t            	timerid;

	//I2C
	ArduinoNano_I2C 	commSoundArray;
	I2C_data 			twoWays;

	int status;								//semaphore status

	/* read arguments */
	for (int i = 1; i < argc; i++)
		{
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0)
			{verbose_Us = true;}
		else
			{verbose_Us = false;}
		}

	//initializes the UN-NAMED semaphore semaphore
	sem_init(&timer_sem_Us, 0, 0);

//TIMER:
	/* Set timer timeout action =============================================================================================>*/
    act.sa_sigaction = timer_expiration_US;	//timer timeout notify function
	act.sa_flags     = SA_SIGINFO;

	(void)sigfillset(&act.sa_mask);							//initializes the signal set specified by "act.sa_mask"
	(void)sigdelset (&act.sa_mask, TIMER_US_SIGNAL);		//This function deletes the signal TIMER_US_SIGNAL
															//from the signal set specified by set

	status = sigaction(TIMER_US_SIGNAL, &act, &oact);		//This function allows the calling task ("timer_expiration")
	if (status != OK)										//to specify the action to be associated with a specific signal
		{
		if (verbose_Us) {printf("\n   ERROR: %s: sigaction failed, status=%d\n", __FUNCTION__, status);}
	    goto errorout;
		}

	/* Create the POSIX timer ==============================================================================================>*/
	notify.sigev_notify            = SIGEV_SIGNAL;
	notify.sigev_signo             = TIMER_US_SIGNAL;
	notify.sigev_value.sival_int   = TIMER_SIGVALUE_INT_US;

	status = timer_create(CLOCK_REALTIME, &notify, &timerid);
	if (status != OK)
	  	  {
		  if (verbose_Us) {printf("\n   ERROR: %s: timer_create failed, errno = %d\n",__FUNCTION__, errno);}
		  goto errorout;
	  	  }

	/* Start the POSIX timer ================================================================================================>*/
	timer.it_value.tv_sec     = 2; /* initial value 2 sec. - shouldn't set to 0 */
	timer.it_value.tv_nsec    = 0;
	timer.it_interval.tv_sec  = 1;	//1 second
	timer.it_interval.tv_nsec = 0;

	status = timer_settime(timerid, TIMER_ABSTIME, &timer, NULL);
	if (status != OK)
	  	  {
		  if (verbose_Us) {printf("\n   ERROR: %s: timer_settime failed, errno = %d\n",__FUNCTION__, errno);}
		  goto errorout;
	  	  }

//MAIN:
	//main code of the function thread is here
	while (!threadUs_should_exit)
		{
	    status = sem_wait(&timer_sem_Us);

	    if (status != 0)
	    	{
	    	if (verbose_Us)
	    		{
	    		printf("\n   ERROR: %s: sem_wait ERROR", __FUNCTION__);
	    		if (errno == EINVAL) printf ("\n    The sem input parameter is not valid (EINVAL)!!!");
	    		if (errno == EINTR)  printf ("\n    The wait was interrupt by a signal received by this task (EINTR)!!!");
	    		}
	        continue;
	        }

	    commSoundArray.readSensors (0x55, twoWays.dataChar);
	    printf ("\n US_L = %d | US_C = %d | US_R = %d | US_B = %d \n",
	    		twoWays.dataInt[0]*5, twoWays.dataInt[1]*5, twoWays.dataInt[2]*5, twoWays.dataInt[3]*5);
    	}

errorout:
	sem_destroy(&timer_sem_Us);								//destroy the un-named semaphore "timer_sem_HD"

	threadUs_running = false;

	return 0;
}


static void timer_expiration_US (int signo, siginfo_t *info, void *ucontext)
{
  int status;

  /* Check signo */
  if (signo != TIMER_US_SIGNAL)
  	  {if (verbose_Us) {PX4_INFO ("\n   ERROR: expected signo = %d but received signo=%d\n" , TIMER_US_SIGNAL, signo);}}
  else
  	  {
	  {if (verbose_Us) {PX4_INFO ("\n   Timer fired!");}}
	  status = sem_post(&timer_sem_Us);
      if (status != OK)
      	  {if (verbose_Us) {PX4_INFO ("\n   ERROR: sem_post failed\n");}}
      }
}



/* main */
/******************************************************************************************************************************************
 * The main app only briefly exists to start timer job.
 *
 */
int daiDrone_Ultrasonic_main(int argc, char *argv[])
{
	if (argc < 2) {
		usageUs();
		return 1;
	}

	if (!strcmp(argv[1], "start"))
	{
		if (threadUs_running)
		{
			printf(" The process of I2C data trasfer with the ultrasonic system is already running\n");

			return 0;
		}

		threadUs_should_exit = false;

		//launch a new background Tasks that will run independently from the calling task:
		deamon_Ultrasonic_task = px4_task_spawn_cmd("reading_Sonic_ArdNano",
				SCHED_DEFAULT,
				SCHED_PRIORITY_MAX - 20,
				2048,
				readingArduinoMini_Ultrasonic,
				(argv) ? (char *const *)&argv[2] : (char *const *)nullptr);
		threadUs_running = true;

		return 0;
	}

	if (!strcmp(argv[1], "stop"))
	{
		threadUs_should_exit = true;

		return 0;
	}

	if (!strcmp(argv[1], "status"))
	{
		if (threadUs_running)
			{
			printf("\tUltrasonic detection system is running\n");
			}
		else
			{
			printf("\tUltrasonic detection system is not started\n");
			}

		return 0;
	}

	usageUs();

	return 0;
}

static void usageUs(void)
{
	PX4_INFO("\n Usage of the human detection system:\n\t daiDrone_HD {start|stop|status} -v\n\n");
}
