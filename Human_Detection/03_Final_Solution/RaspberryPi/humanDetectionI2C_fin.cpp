/**
 * The Human Detection software module
 * 
 * @author Dan-Marius Dobrea
 * Program developed for: HoverGames Challenge 1: Fight Fire with Flyers
 *
 * For the I2C slave port, the pin on the RPi are as follows:
 * 
 *                      GPIO 18 (pin 12) = SDA 
 *                      GPIO 19 (pin 35) = SCL
 */

/***********************************************************************
 *                             INCLUDES
 */
#include <stdio.h>

#include <unistd.h>         //for sysconf()
#include <sys/types.h>      //both include line are required by getuid()

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <time.h>
#include <signal.h>
#include <pthread.h>

#include <sched.h>

#include <errno.h>

#include <queue>
#include <mutex>
#include <atomic>
#include <semaphore.h>

#include <unistd.h>

#include <signal.h>
#include <sys/types.h>

#include <pigpio.h>
#include <string.h>		    //only for memcpy


using namespace std;
using namespace cv;

/***********************************************************************
 *                          FUNCTIONS
 */
//function used to start the Image Acq. & Processing on different cores
void procesStart (union sigval val);

void *AcqProcessCPU1 (void *arg);
void *AcqProcessCPU2 (void *arg);
void *AcqProcessCPU3 (void *arg);
void *GrabThread     (void *arg);
void shiftRightVector(char *vect); //shift to right a vector of five chars
void releaseResurces (void);
void sig_handler_INT (int num);

  //--- I2C slave functions ---
void *slaveI2C       (void *arg);
void decodStatSlave  (int statLoc);
int  getControlBits  (int I2C_address, char open);
void closeI2C_Slave  (void);

/*********************************************************************
 *                        GLOBAL VARIABLES
 */
int err;
pthread_t tid[5];
int thread1DataReady, thread2DataReady, thread3DataReady;

int64 t1, t2; 
double t_cycle;

VideoCapture capture(0);
Mat img1, img2, img3;
HOGDescriptor hog1, hog2, hog3;
sem_t semCPU1, semCPU2, semCPU3, semClasifDataReady;
char key;
int showResult, timeToQuit, verboseMode;

timer_t timerid;            //timer ID

queue<Mat> buffer;
mutex mtxCam;               //mutex on the imagecam frames
mutex mtxHuD;               //mutex on the Human Detection result buffer
atomic<bool> grabOn;        //this is lock free

//number of humans detected: now - humanDetected[0], 
//                           one second before - humanDetected[1], 
//                           two seconds before - humanDetected[2],
//                           ..................................... 
char humanDetected[6]; 

  //--- I2C global variables ---
const int slaveAddress = 0x4A; 	    // <-- Raspbsrry Pi address <= set a number of your choice
bsc_xfer_t xfer; 				    // Struct to control data flow  


/***********************************************************************
 *                        MAIN FUNCTION
 */
int main(int argc, char **argv)
{
// I. Setting program working mode according to user desires ====================================================        
    
    int opt;
    
    showResult  = 0;    //demo mode
    verboseMode = 0;    //display more internal information
    
    if (argc > 1)
        {
        while ((opt = getopt(argc, argv, "vd?h")) != -1) 
            {
            switch (opt) 
                {
                case 'd':
                    showResult = 1;
                    break;
                case 'v':
                    verboseMode = 1;               
                    break;
               case '?':
                    printf("Usage: %s [-d] [-v]\n     -d = demo mode\n     -v = verbose\n\n", argv[0]);
                    return 0;
                    break;
               case 'h':
                    printf("Usage: %s [-d] [-v]\n     -d = demo mode\n     -v = verbose\n\n", argv[0]);
                    return 0;
                    break;                                        
                default: /* '?' */
                    return 0;
                    break;
                }
            }    
        }
    else        
        printf("\n The default working mode has been selected: no demo mode & no verbose information !\n");
    
    cpu_set_t cpuset;
	
// II. Checking the main program's requirements =================================================================

	if ( getuid() != 0 ) 
        {
        printf("\n You must run this program as root. Exiting.\n\n");
        return -1;
        }  
        
    int num_cores = sysconf(_SC_NPROCESSORS_ONLN);
    if (verboseMode) printf ("\n The number of core of this system is: %d", num_cores);
    if (num_cores < 4)
        {
        printf("\n\n In order to run this program the system must have at least 4 cores. Exiting.\n");
        return -2;
        } 

// III. Making all main initialization ==========================================================================        

    thread1DataReady = 0; thread2DataReady = 0; thread3DataReady = 0;
    humanDetected[0] = humanDetected[1] = humanDetected[2] = humanDetected[3] = humanDetected[4] = humanDetected[5] =0;

    timeToQuit = 0;
    
    sem_init(&semCPU1,            0, 0); 
    sem_init(&semCPU2,            0, 0); 
    sem_init(&semCPU3,            0, 0);
    sem_init(&semClasifDataReady, 0, 0);
    
    //------------------------------------------------------------------
    CPU_ZERO (&cpuset); 	/* clear all CPUs */
    CPU_SET (0, &cpuset); 	/* allow CPU #0  */
    CPU_CLR (1, &cpuset); 	/* forbid CPU #1 */
    CPU_CLR (2, &cpuset); 	/* forbid CPU #2 */
    CPU_CLR (3, &cpuset); 	/* forbid CPU #3 */
    
    err = sched_setaffinity (0, sizeof (cpu_set_t), &cpuset);
    if (err == -1 && verboseMode == 1)
        printf ("\n Setting afinity to CPU0 for the main process error !\n");
    
    //------------------------------------------------------------------
    grabOn.store(true);     //set the imgs grabbing control variable
         
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    if(!capture.isOpened())
        {
        if (verboseMode) printf ("\nFailed to connect to the camera.");
        return -1;
        }
        
    hog1.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector()); 
    hog2.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector()); 
    hog3.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector()); 
    
    // to be sure that I2C starts from a well-known state
    closeI2C_Slave();
   
// IV. Compute the [Immage Acq & Processing - recognition] time =================================================    

    Mat img;
    vector<Rect> found; 
    
    //namedWindow  ("People detector",WINDOW_NORMAL);
    //resizeWindow ("People detector", 1024, 768);
    
    //Mainly the first call will take longer than the subsequent calls.
    //   So, the following sequence of code is  used only to eliminate 
    //this transitory regime.
        capture >> img;
        if(img.empty())
            {
            if (verboseMode) printf ("\nFailed to capture an image");
            return -1;
            }

        hog1.detectMultiScale(img, found, 0, Size(8,8), Size(), 1.05, 2, false);
    //<= here it end transitory regime elimination

   t1 = getTickCount();
    capture >> img;
    if(img.empty())
		{
        if (verboseMode) printf ("\nFailed to capture an image");
        return -1;
        }

	// Run the detector with default parameters. To get a higher hit-rate
    // (and more false alarms, respectively), decrease the hitThreshold and
    // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).
   	hog1.detectMultiScale(img, found, 0, Size(8,8), Size(), 1.05, 2, false);
    
    //if (showResult) imshow("People detector", img);     
   t2 = getTickCount();
       
    
    //main time result here:
    t_cycle = (double)(t2-t1)/getTickFrequency();
    if (verboseMode) printf ("\n Acq. Img & Processing %f [s]", t_cycle);
    t_cycle *= 1.2;     //to compensate the variance of the results
    
// V. Creating threads ==========================================================================================

   //Thread used to grab images
   err = pthread_create (&(tid[3]), NULL, &GrabThread, NULL);
   if (err != 0)
        {printf ("\nI can't create imgs grab thread 1:[%s]", strerror(err));}
   else
        {
        printf ("\n Imgs grab thread created successfully.\n");

		CPU_ZERO(&cpuset);
        CPU_SET (0, &cpuset);
        
        err = pthread_setaffinity_np(tid[3], sizeof(cpu_set_t), &cpuset);
        if (err != 0)
               printf ("\nImgs grab thread set affinity to CPU0 error!");         
        }

   //Thread I => CPU1
   err = pthread_create (&(tid[0]), NULL, &AcqProcessCPU1, NULL);
   if (err != 0)
	    {printf ("\nI can't create thread 1:[%s]", strerror(err));}
   else
        {
        printf ("\n Thread 1 created successfully.\n");

		CPU_ZERO(&cpuset);
        CPU_SET (1, &cpuset);
        
        err = pthread_setaffinity_np(tid[0], sizeof(cpu_set_t), &cpuset);
        if (err != 0)
               printf ("\nThread 1 set affinity to CPU1 error!");         
        }
        
   //Thread II => CPU2
   err = pthread_create (&(tid[1]), NULL, &AcqProcessCPU2, NULL);
   if (err != 0)
	    {printf ("\nI can't create thread 1:[%s]", strerror(err));}
   else
        {
        printf ("\n Thread 2 created successfully.\n");        

		CPU_ZERO(&cpuset);
        CPU_SET (2, &cpuset);
        
        err = pthread_setaffinity_np(tid[1], sizeof(cpu_set_t), &cpuset);
        if (err != 0)
               printf ("\nThread 2 set affinity to CPU2 error!");         
        }

   //Thread III => CPU3
   err = pthread_create (&(tid[2]), NULL, &AcqProcessCPU3, NULL);
   if (err != 0)
	    {printf ("\nI can't create thread 1:[%s]", strerror(err));}
   else
        {
        printf ("\n Thread 3 created successfully.\n");        
        
		CPU_ZERO(&cpuset);
        CPU_SET (3, &cpuset);
        
        err = pthread_setaffinity_np(tid[2], sizeof(cpu_set_t), &cpuset);
        if (err != 0)
               printf ("\nThread 3 set affinity to CPU3 error!");         
        }
        
   //Thread used to manage I2C communication of the RPi as a slave device
   err = pthread_create (&(tid[4]), NULL, &slaveI2C, NULL);
   if (err != 0)
        {if (verboseMode) printf ("\nI can't create I2C slave thread 1:[%s]", strerror(err));}
   else
        {
        printf ("\n I2C slave thread created successfully.\n");

		CPU_ZERO(&cpuset);
        CPU_SET (0, &cpuset);
        
        err = pthread_setaffinity_np(tid[4], sizeof(cpu_set_t), &cpuset);
        if (err != 0)
               if (verboseMode) printf ("\nI2C slave thread set affinity to CPU0 error!");         
        }        

// VI. Timer: create and start ==================================================================================

    pthread_attr_t attr;
    //initializes the thread attributes object (scheduling policy, 
    //scheduling priority, stack size, etc.)
    pthread_attr_init( &attr );		

    struct sched_param parm;
    parm.sched_priority = 10;
    
    //Set the scheduling parameter attributes of the thread 
    //attributes object referred to by "attr" to the values 
    //specified in the buffer pointed to by "param"
    pthread_attr_setschedparam(&attr, &parm);

    struct sigevent sig;
    sig.sigev_notify = SIGEV_THREAD;
    sig.sigev_notify_function = procesStart;
    sig.sigev_value.sival_int = 13;
    sig.sigev_notify_attributes = &attr;

    //create a new timer.
    err = timer_create(CLOCK_REALTIME, &sig, &timerid);
    if (err == 0)
        {
        t_cycle = t_cycle / 3;   
        if (verboseMode)
            {
            printf ("\n t_cycle = %f", t_cycle); 
            printf ("\n");
            }
            
        struct itimerspec in, out;
        in.it_value.tv_sec = 1;
        in.it_value.tv_nsec = 0;
        in.it_interval.tv_sec = floor (t_cycle);               //ms  //us * ns 
        in.it_interval.tv_nsec = (t_cycle - floor (t_cycle)) * 1000 * 1000000;
        
        //issue the periodic timer request here
        err = timer_settime(timerid, 0, &in, &out);
        if (err != 0)
            if (verboseMode) printf("\n Error: timer_settime() failed with %d\n", errno);
        }

// VII. Catching CTRL + C =======================================================================================
    if ( signal (SIGINT, sig_handler_INT) == SIG_ERR )
        if (verboseMode) printf ("\nCan't catch SIGINT. Error! \n");

// VIII. Data display ===========================================================================================
    t1 = t2 = 0;
    
       
    do
    {
        sem_wait(&semClasifDataReady);

        t2 = getTickCount();    
        
        if (thread1DataReady)
            {
            thread1DataReady = 0;
            
            if (showResult == 1)
                {
                ostringstream buf;
                buf << "tp: " << ((double)(t2 - t1))/getTickFrequency();
                putText(img1, buf.str(), Point(10, 30), FONT_HERSHEY_PLAIN, 2.0, Scalar(0, 0, 255), 2, false);
            
                imshow("People detector", img1);     
                }
            }
            
        if (thread2DataReady)
            {
            thread2DataReady = 0;

            if (showResult == 1)
                {
                ostringstream buf;
                buf << "tp: " << ((double)(t2 - t1))/getTickFrequency();
                putText(img2, buf.str(), Point(10, 30), FONT_HERSHEY_PLAIN, 2.0, Scalar(0, 0, 255), 2, false);

                imshow("People detector", img2);     
                }
            }

        if (thread3DataReady)
            {
            thread3DataReady = 0;

            if (showResult == 1)
                {
                ostringstream buf;    
                buf << "tp: " << ((double)(t2 - t1))/getTickFrequency();
                putText(img3, buf.str(), Point(10, 30), FONT_HERSHEY_PLAIN, 2.0, Scalar(0, 0, 255), 2, false);

                imshow("People detector", img3);     
                }
            }
            
        t1 = t2;  
        
        if (verboseMode) 
            {
            mtxHuD.lock();    
            printf (" : hd[0s] = %d, hd[-1s] = %d, hd[-2s] = %d, hd[-3s] = %d, hd[-4s] = %d", 
                     humanDetected[0], humanDetected[1], humanDetected[2], humanDetected[3], humanDetected[4]); 
            mtxHuD.unlock();         
            }
           
        key = (char)waitKey(1);
            
    }while( key != 'q');
    

// IX. Releasing all used resources =============================================================================	
    releaseResurces();
    
	exit (0);
}

void releaseResurces(void)
{
    timeToQuit = 1;                 //finish all the threads
    pthread_join (tid[0], NULL);
    pthread_join (tid[1], NULL);
    pthread_join (tid[2], NULL);
    pthread_join (tid[4], NULL);
    
    sem_destroy(&semCPU1);
    sem_destroy(&semCPU2);
    sem_destroy(&semCPU3);
    sem_destroy(&semClasifDataReady);

    //delete the timer
    timer_delete(timerid);
    
    grabOn.store(false);            //stop the grab loop
    pthread_join (tid[3], NULL);    //wait for the grab function to end
    while (!buffer.empty())         //flushing the buffer
        {buffer.pop();}  
           
    if (showResult == 1) destroyWindow("People detector");  
    
    closeI2C_Slave();     
    
    printf("\n");    
}


void sig_handler_INT (int num)
{
    if (verboseMode) printf ("\n Now the application will be closed and the resources released. \n"); 

    releaseResurces();
    exit (0);
}


void procesStart (union sigval val)
{
    static int thread_no;
    
    switch (thread_no)
    {
        case 0:
            sem_post(&semCPU1);  //startThread1 = 1;
            break;
        case 1:
            sem_post(&semCPU2);  //startThread2 = 1;
            break;
        case 2:
            sem_post(&semCPU3);  //startThread3 = 1;
            break;
    }
    
    thread_no++;
    if (thread_no == 3) {thread_no = 0;}
}


void *GrabThread (void *arg)
{
    Mat tmp; 
    
    while (grabOn.load() == true) //this is lock free
        {
        capture >> tmp;  
        
        //if tmp emprty forces the next iteration of the loop  
        //    to take place, skipping any code in between  
        if (tmp.empty()) continue;
        
          //get lock only when we have a frame
          mtxCam.lock();
          //buffer.push(tmp) stores item by reference than avoid
          //    this will create a new cv::Mat for each grab
          buffer.push(Mat(tmp.size(), tmp.type()));
          tmp.copyTo(buffer.back());
          mtxCam.unlock();
          
          sched_yield();
        }
    
    pthread_exit (NULL); 
}

/*********************************************************************
 *                       PROCESSING TASKS
 */
void *AcqProcessCPU1 (void *arg)
{
    vector<Rect> found1;
    int bufSize1;
     
    do
    {
        //wait
        sem_wait(&semCPU1);
        
          mtxCam.lock();              //lock memory for exclusive access
          bufSize1 = buffer.size();   //check how many frames are waiting 
          if (bufSize1 > 0)           //if some 
            {
              //get the newest grabbed frame (queue=FIFO)
              buffer.back().copyTo(img1);   
            
              while (!buffer.empty()) //flushing the buffer
                  {buffer.pop();}
            }
          mtxCam.unlock();            //unlock the memory        
        
        // Run the detector with default parameters. To get a higher hit-rate
        // (and more false alarms, respectively), decrease the hitThreshold and
        // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).
        hog1.detectMultiScale(img1, found1, 0, Size(8,8), Size(), 1.05, 2, false);
        
        mtxHuD.lock();
          shiftRightVector(humanDetected);
          humanDetected[0] = found1.size();
        mtxHuD.unlock();  
        if (verboseMode) printf ("\n Found %d person[s]", found1.size());
        
        if (showResult == 1)
            {
            for (vector<Rect>::iterator i = found1.begin(); i != found1.end(); ++i)
                {
                Rect &r = *i;
                // The HOG detector returns slightly larger rectangles than the real objects,
                // so we slightly shrink the rectangles to get a nicer output.
                r.x     += cvRound(r.width  * 0.1);
                r.width  = cvRound(r.width  * 0.8);
                r.y     += cvRound(r.height * 0.07);
                r.height = cvRound(r.height * 0.8);            
                
                rectangle(img1, r.tl(), r.br(), cv::Scalar(0, 255, 0), 2);
                }    
            }
            
        thread1DataReady = 1;
        sem_post(&semClasifDataReady);
        
    } while (timeToQuit == 0);        
    
    pthread_exit (NULL);
}

void *AcqProcessCPU2 (void *arg)
{
    vector<Rect> found2;
    int bufSize2;

    do 
    {
        //wait
        sem_wait(&semCPU2);
    
          mtxCam.lock();              //lock memory for exclusive access
          bufSize2 = buffer.size();   //check how many frames are waiting 
          if (bufSize2 > 0)           //if some 
            {
              //get the newest grabbed frame (queue=FIFO)
              buffer.back().copyTo(img2);   
            
              while (!buffer.empty()) //flushing the buffer
                  {buffer.pop();}
            }
          mtxCam.unlock();            //unlock the memory        
    
        // Run the detector with default parameters. To get a higher hit-rate
        // (and more false alarms, respectively), decrease the hitThreshold and
        // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).
        hog2.detectMultiScale(img2, found2, 0, Size(8,8), Size(), 1.05, 2, false);
        
        mtxHuD.lock();
          shiftRightVector(humanDetected);
          humanDetected[0] = found2.size();
        mtxHuD.unlock();  
        if (verboseMode) printf ("\n Found %d person[s]", found2.size());

        if (showResult == 1)
            {
            for (vector<Rect>::iterator j = found2.begin(); j != found2.end(); ++j)
                {
                Rect &r = *j;
                // The HOG detector returns slightly larger rectangles than the real objects,
                // so we slightly shrink the rectangles to get a nicer output.
                r.x     += cvRound(r.width  * 0.1);
                r.width  = cvRound(r.width  * 0.8);
                r.y     += cvRound(r.height * 0.07);
                r.height = cvRound(r.height * 0.8);            
                
                rectangle(img2, r.tl(), r.br(), cv::Scalar(0, 255, 0), 2);
                }    
            }
            
        thread2DataReady = 1;
        sem_post(&semClasifDataReady);
    
    } while (timeToQuit == 0);
    
    pthread_exit (NULL);
}

void *AcqProcessCPU3 (void *arg)
{
    vector<Rect> found3;
    int bufSize3;
    
    do 
    {
        //wait
        sem_wait(&semCPU3);
    
          mtxCam.lock();              //lock memory for exclusive access
          bufSize3 = buffer.size();   //check how many frames are waiting 
          if (bufSize3 > 0)           //if some 
            {
              //get the newest grabbed frame (queue=FIFO)
              buffer.back().copyTo(img3);   
            
              while (!buffer.empty()) //flushing the buffer
                  {buffer.pop();}
            }
          mtxCam.unlock();            //unlock the memory        

        // Run the detector with default parameters. To get a higher hit-rate
        // (and more false alarms, respectively), decrease the hitThreshold and
        // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).
        hog3.detectMultiScale(img3, found3, 0, Size(8,8), Size(), 1.05, 2, false);
        
        mtxHuD.lock();
          shiftRightVector(humanDetected);
          humanDetected[0] = found3.size();
        mtxHuD.unlock();  
        if (verboseMode) printf ("\n Found %d person[s]", found3.size());
        
        if (showResult == 1)
            {
            for (vector<Rect>::iterator k = found3.begin(); k != found3.end(); ++k)
                {
                Rect &r = *k;
                // The HOG detector returns slightly larger rectangles than the real objects,
                // so we slightly shrink the rectangles to get a nicer output.
                r.x     += cvRound(r.width  * 0.1);
                r.width  = cvRound(r.width  * 0.8);
                r.y     += cvRound(r.height * 0.07);
                r.height = cvRound(r.height * 0.8);            
                
                rectangle(img3, r.tl(), r.br(), cv::Scalar(0, 255, 0), 2);
                }    
            }

        thread3DataReady = 1;
        sem_post(&semClasifDataReady);
        
    } while (timeToQuit == 0);                
        
    pthread_exit (NULL);
}


void shiftRightVector (char *vect)
{
    vect[4] = vect[3];
    vect[3] = vect[2];
    vect[2] = vect[1];
    vect[1] = vect[0];
}


/*********************************************************************
 *                       I2C SLAVE FUNCTIONS
 */
void *slaveI2C (void *arg)
{
    int status;
    xfer.txCnt = 0;
    
    nice (-10);
	
    gpioInitialise();
    if (verboseMode) printf ("\n    Initialized GPIOs! Status before init BSC (Broadcom Serial Controller)");

    // Close old device (if any) to avoid 
    // conflicts when restarting and reset the FIFO
    xfer.control = getControlBits(slaveAddress, 0); 	
    bscXfer(&xfer);

    // Set I2C slave Address to "slaveAddress"
    xfer.control = getControlBits(slaveAddress, 1);
    status = bscXfer(&xfer); 
    // Should now be visible in I2C-Scanners
    
    if (verboseMode)
        {
        printf ("\nStatus after init\n");
        printf ("\n    # of bytes successfully copied to transmit FIFO: %d", ((status & 0x1F0000) >> 16) );
        printf ("\n    # of bytes in receieve FIFO: %d", ((status & 0xF800) >> 11));
        int transmitInFIFO = ((status & 0xC0) >> 6); 
        printf ("\n    # of bytes in transmit FIFO: %d", transmitInFIFO);
        printf ("\n\n");
        }
    
    if (status >= 0)
		{
        if (verboseMode) printf ("I2C Slave configured!\n");
        
        while(timeToQuit == 0)
			{
            //check continuously when on the I2C slave port a new byte is received    
            bscXfer(&xfer);
                                   
            if(xfer.rxCnt > 0) 
				{
				if ( (xfer.rxCnt == 1) && (xfer.rxBuf[0] == 0x55))
					{
                    if (verboseMode)
                        {
                        printf ("\n\n ==============================================================");	    
                        printf ("\n Received the expected command {0x55} => the data will be send!");	
                        }
					
                    mtxHuD.lock();
                        humanDetected [5] = 0;
                        memcpy(xfer.txBuf, humanDetected, 6); 
                    mtxHuD.unlock();    
                    xfer.txCnt = 5;
                    
                    //usleep(300);                    //to have enough time to write in TX buffer
										
					if (verboseMode) printf ("\n Data succesfully sent!");
					
					status = bscXfer(&xfer);        //here data will be send
                    xfer.txCnt = 0;
					decodStatSlave (status);
  					}
				else
                    {if (verboseMode) printf ("\n ERROR: the expected command {0x55} was not received !");}
				}
			}
		}
    else
        {if (verboseMode) printf( "Failed to open BSC (Broadcom Serial Controller) as a slave!!!\n");}
    
    pthread_exit (NULL); 
}


void decodStatSlave (int statLoc)
{
    printf ("\n\n I2C status:");
    printf ("\n    # of bytes successfully copied to transmit FIFO: %d", ((statLoc & 0x1F0000) >> 16) );
    printf ("\n    # of bytes in receieve FIFO: %d", ((statLoc & 0xF800) >> 11));
    printf ("\n    # of bytes in transmit FIFO: %d", ((statLoc & 0xC0) >> 6));
    printf ("\n\n");
}


int getControlBits(int I2C_address /* max 127 */, char open) 
{
    /*
    Excerpt from http://abyz.me.uk/rpi/pigpio/cif.html#bscXfer regarding the control bits:

    22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00
    a  a  a  a  a  a  a  -  -  IT HC TF IR RE TE BK EC ES PL PH I2 SP EN

    Bits 0-13 are copied unchanged to the BSC CR register. See pages   
    163-165 of the Broadcom peripherals document for full details. 

    aaaaaaa defines the I2C slave address (only relevant in I2C mode)
    IT  invert transmit status flags                [13]
    HC  enable host control                         [12]
    TF  enable test FIFO                            [11]
    IR  invert receive status flags                 [10]
    RE  enable receive                              [9]
    TE  enable transmit                             [8]
    BK  abort operation and clear FIFOs             [7]
    EC  send control register as first I2C byte     [6]
    ES  send status register as first I2C byte      [5]
    PL  set SPI polarity high                       [4]
    PH  set SPI phase high                          [3]   
    I2  enable I2C mode                             [2]
    SP  enable SPI mode                             [1] 
    EN  enable BSC peripheral                       [0] 
    */

    // Flags like this: 0b/*IT:*/0/*HC:*/0/*TF:*/0/*IR:*/0/*RE:*/0/*TE:*/0/*BK:*/0/*EC:*/0/*ES:*/0/*PL:*/0/*PH:*/0/*I2:*/0/*SP:*/0/*EN:*/0;

    int flags = 0;
    
    if ( open == 3 )
        {flags = /*RE:*/ (1 << 9) | /*TE:*/ (1 << 8) | /*BK:*/ (1 << 7) | /*I2:*/ (1 << 2) | /*EN:*/ (1 << 0);}
    if ( open == 2 )
		{flags = /*I2:*/ (1 << 2) | /*EN:*/ (1 << 0);}
    if ( open == 1 )
        {flags = /*RE:*/ (1 << 9) | /*TE:*/ (1 << 8) | /*I2:*/ (1 << 2) | /*EN:*/ (1 << 0);}
    if ( open == 0 )// Close/Abort
        {flags = /*BK:*/ (1 << 7) | /*I2:*/ (0 << 2) | /*EN:*/ (0 << 0);}

    return ((I2C_address << 16 /*= to the start of significant bits*/) | flags);
}


void closeI2C_Slave (void) 
{
    gpioInitialise();
    printf ("\n I2C => Initialized GPIOs\n");

    xfer.control = getControlBits(slaveAddress, 0);
    bscXfer(&xfer);
    printf (" I2C => Closed slave.\n");

    gpioTerminate();
    printf (" I2C => Terminated GPIOs.\n");
}
