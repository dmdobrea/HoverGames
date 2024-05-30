/**
 * @file daiDrone_HD.cpp
 *
 * Program used to receive the Human Detection information from the RPi
 *
 * @author Dan-Marius Dobrea
 * Program developed for: HoverGames Challenge 1: Fight Fire with Flyers
 */

#include "daiDrone_HD.h"
#include <px4_log.h>

#include <time.h>
#include <signal.h>
#include <semaphore.h>

#define TIMER_HD_SIGNAL 30 	//[0, 31]
#define TIMER_SIGVALUE_INT_HD	 10


extern "C" __EXPORT int daiDrone_HD_main (int argc, char *argv[]);

static void timer_expiration_HD (int signo, siginfo_t *info, void *ucontext);
static void usageHD(void);

/* Variables */
static bool threadHD_should_exit = false;		/**< Daemon exit flag */
static bool threadHD_running     = false;		/**< Daemon status flag */
static int  deamon_HD_task;				/**< Handle of deamon task / thread */

bool verbose = false;
sem_t timer_sem_HD;

/******************************************************************************************************************************************
 *                                              class dd_RPi_i2c implementation
 */

/* Constructor */
dd_RPi_i2c::dd_RPi_i2c() : I2C("RPi_i2c_test", "/dev/RPi_i2c_test", PX4_I2C_BUS_EXPANSION, RPi_I2CADDR, RPi_BUS_SPEED)
//																				               0x4A    100KHz = 1000*100
{
	dd_RPi_i2c::init();
}

/* Initialization of I2C device */
int dd_RPi_i2c::init()
{
	int ret = OK;

	/* do I2C init (and probe) first */
	ret = I2C::init();

	/* if probe/setup failed, bail now */
	if (ret != OK)
		{
		DEVICE_DEBUG("I2C setup failed");
		return ret;
		};

	I2C::set_bus_clock(1, RPi_BUS_SPEED);		//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	return ret;
}

void dd_RPi_i2c::readDataFromRPi(uint8_t my_cmd, unsigned char *buffReceive)
{
	const uint8_t cmd = my_cmd;

	//I2C::transfer(const uint8_t *send, const unsigned send_len, uint8_t *recv, const unsigned recv_len)
	/*
	if (OK != transfer(&cmd, 1, (uint8_t *) buffReceive, 5))
		{
		PX4_ERR("\nNo Data received");
		}
	else
		{
		// switching read bytes
		PX4_INFO("\nThe data was correctly received from RPi !");
		}
	*/
	if (OK != transfer(&cmd, 1, (uint8_t *) buffReceive, 0))
		{
		;//PX4_ERR("\nTransmission error !");
		}

	usleep(10000);

	if (OK != transfer(&cmd, 0, (uint8_t *) buffReceive, 5))
		{
		;//PX4_ERR("\nReceiving error !");
		}
}


void dd_RPi_i2c::wasteRPiFIFO_data (unsigned char *buffReceive)
{
	const uint8_t cmd = 0;

	if (OK != transfer(&cmd, 0, (uint8_t *) buffReceive, 6))
		{
		PX4_ERR("\nReceiving FIFO waste data error !");
		}
}


/* Main Thread */
int readingRPi_HumanDetection (int argc, char *argv[])
{
	// timer variables
	struct sigaction   act;
	struct sigaction   oact;
	struct sigevent    notify;
	struct itimerspec  timer;
	timer_t            timerid;

	int                status;

	//I2C
	dd_RPi_i2c	   commRPi;
	unsigned char	   buff[10];

	/* read arguments */
	for (int i = 1; i < argc; i++)
		{
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0)
			{verbose = true;}
		else
			{verbose = false;}
		}

	//initializes the UN-NAMED semaphore semaphore
	sem_init(&timer_sem_HD, 0, 0);

	/* code used to consume the forgotten values from the Raspberry Pi FIFO buffer ==========================================>*/
	int contor = 0;
	do{
		contor++;
		if (contor > 15) break;
		commRPi.wasteRPiFIFO_data (buff);
	} while ( (buff[0] == buff[1]) && (buff[1] == buff[2]) && (buff[2] == buff[3]) && (buff[3] == buff[4]) && (buff[4] == buff[5]) );

//TIMER:
	/* Set timer timeout action =============================================================================================>*/
    	act.sa_sigaction = timer_expiration_HD;			//timer timeout notify function
	act.sa_flags     = SA_SIGINFO;

	sigfillset(&act.sa_mask);				//initializes the signal set specified by "act.sa_mask"
	sigdelset (&act.sa_mask, TIMER_HD_SIGNAL);		//This function deletes the signal TIMER_HD_SIGNAL
								//from the signal set specified by set

	status = sigaction(TIMER_HD_SIGNAL, &act, &oact);	//This function allows the calling task ("timer_expiration")
	if (status != OK)					//to specify the action to be associated with a specific signal
		{
		if (verbose) {printf("\n   ERROR: %s: sigaction failed, status=%d\n", __FUNCTION__, status);}
	    	goto errorout;
		}

	/* Create the POSIX timer ==============================================================================================>*/
	notify.sigev_notify            = SIGEV_SIGNAL;
	notify.sigev_signo             = TIMER_HD_SIGNAL;
	notify.sigev_value.sival_int   = TIMER_SIGVALUE_INT_HD;

	status = timer_create(CLOCK_REALTIME, &notify, &timerid);
	if (status != OK)
	  	  {
		  if (verbose) {printf("\n   ERROR: %s: timer_create failed, errno = %d\n",__FUNCTION__, errno);}
		  goto errorout;
	  	  }

	/* Start the POSIX timer ================================================================================================>*/
	timer.it_value.tv_sec     = 2; /* initial value 2 seconds */
	timer.it_value.tv_nsec    = 0;
	timer.it_interval.tv_sec  = 1;	//1 second
	timer.it_interval.tv_nsec = 0;

	status = timer_settime(timerid, TIMER_ABSTIME, &timer, NULL);
	if (status != OK)
	  	  {
		  if (verbose) {printf("\n   ERROR: %s: timer_settime failed, errno = %d\n",__FUNCTION__, errno);}
		  goto errorout;
	  	  }

//MAIN
	//main code of the function thread is here
	while (!threadHD_should_exit)
	{
	    status = sem_wait(&timer_sem_HD);

	    if (status != 0)
	    	{
	    	if (verbose)
	    		{
	    		;
	    		//printf("\n   ERROR: %s: sem_wait ERROR", __FUNCTION__);
	    		//if (errno == EINVAL) printf ("\n    The sem input parameter is not valid (EINVAL)!!!");
	    		//if (errno == EINTR)  printf ("\n    The wait was interrupt by a signal received by this task (EINTR)!!!");
	    		}
	        continue;
	        }

	    commRPi.readDataFromRPi (0x55, buff);
	    //if (verbose)
	    	{
    		printf("\n  Human(s) detected: %d now, %d [-1s], %d [-2s], %d [-3s], %d [-4s], %d",
	    				     buff[0], buff[1], buff[2], buff[3], buff[4], buff[5]);
	    	}

    	if ( buff[5] == 0 && buff[4] != 255)
    		{
    		if ( (buff[0] !=0 && buff[1] != 0) || (buff[1] !=0 && buff[2] != 0) || (buff[0] !=0 && buff[2] != 0) )
    			{
    			int numberHumans = buff[0];

    			if (buff[1] > numberHumans) {numberHumans = buff[1];}
    			if (buff[2] > numberHumans) {numberHumans = buff[2];}

    			if (numberHumans == 1)
    				printf ("\n One human detected! Send a rescue team!\n\n");
    			else
    				printf ("\n Human(s) detected! There are %d human(s) send a rescue team!\n\n", numberHumans);
    			}
    		else
    			{printf ("\n No human(s) detected!\n\n");}
    		}
	}

errorout:
	sem_destroy(&timer_sem_HD);								//destroy the un-named semaphore "timer_sem_HD"

	threadHD_running = false;

	return 0;
}

static void timer_expiration_HD (int signo, siginfo_t *info, void *ucontext)
{
  int status;

  /* Check signo */
  if (signo != TIMER_HD_SIGNAL)
  	  {if (verbose) {PX4_INFO ("\n   ERROR: expected signo = %d but received signo=%d\n" , TIMER_HD_SIGNAL, signo);}}
  else
  	  {
	  if (verbose) {PX4_INFO ("\n   Timer fired!");}
		  
	  status = sem_post(&timer_sem_HD);
		  
      	  if (status != OK)
      	  	{if (verbose) {PX4_INFO ("\n   ERROR: sem_post failed\n");}}
      }
}


/* main */
/******************************************************************************************************************************************
 * The main app only briefly exists to start timer job.
 *
 */
int daiDrone_HD_main(int argc, char *argv[])
{
	if (argc < 2) {
		usageHD();
		return 1;
	}

	if (!strcmp(argv[1], "start"))
	{
		if (threadHD_running)
		{
			printf(" The I2C data trasfer is already running\n");

			return 0;
		}

		threadHD_should_exit = false;

		//launch a new background Tasks
		deamon_HD_task = px4_task_spawn_cmd("reading_HumanDetection_RPi",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 20,
						 2048,
						 readingRPi_HumanDetection,
						 (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);
		threadHD_running = true;

		return 0;
	}

	if (!strcmp(argv[1], "stop"))
	{
		threadHD_should_exit = true;

		return 0;
	}

	if (!strcmp(argv[1], "status"))
	{
		if (threadHD_running)
			{
			printf("\tHuman detection system is running\n");
			}
		else
			{
			printf("\tHuman detection system is not started\n");
			}

		return 0;
	}

	usageHD();

	return 0;
}

static void usageHD(void)
{
	PX4_INFO("\n Usage of the human detection system:\n\t daiDrone_HD {start|stop|status} -v\n\n");
}
