#include <pthread.h>
#include <pigpio.h>
#include <stdio.h>
#include <string.h>		//only for memcpy

void runSlave();
void closeSlave();
void statusSlave ();
void decodStatSlave (int statLoc);
int getControlBits(int, char);

const int slaveAddress = 0x4A; 	// <-- Your address of choice
bsc_xfer_t xfer; 				// Struct to control data flow

int main(int argc, char **argv)
{
	// Chose one of those two lines (comment the other out):
    closeSlave();
    printf ("\n");
    runSlave();
    	
	return 0;
}

int charVar;
char buffDateTx[20];

void runSlave() 
{
    static int cnt;
	charVar = 65;

    xfer.txCnt = 0;
	
    gpioInitialise();
    printf ("Initialized GPIOs! Status before init\n");

    // Close old device (if any) to avoid 
    // conflicts when restarting and reset the FIFO
    xfer.control = getControlBits(slaveAddress, 0); 	
    bscXfer(&xfer);

    // Set I2C slave Address to "slaveAddress"
    xfer.control = getControlBits(slaveAddress, 1);
    int status = bscXfer(&xfer); 
    // Should now be visible in I2C-Scanners
    
    printf ("Status after init\n");
    printf ("\n    # of bytes successfully copied to transmit FIFO: %d", ((status & 0x1F0000) >> 16) );
    printf ("\n    # of bytes in receieve FIFO: %d", ((status & 0xF800) >> 11));
    int transmitInFIFO = ((status & 0xC0) >> 6); 
    printf ("\n    # of bytes in transmit FIFO: %d", transmitInFIFO);
    printf ("\n\n");
    
    if (status >= 0)
		{
        printf ("Slave configured!\n");
        
        while(1)
			{
            status = bscXfer(&xfer);
                                   
            if(xfer.rxCnt > 0) 
				{
				if ( (xfer.rxCnt == 1) && (xfer.rxBuf[0] == 0x55))
					{
                    printf ("\n ========================");	    
					printf ("\n Received the expected command {0x55} => the data will be send!");	
					
					memcpy( buffDateTx, "ABCDE", 5);
					//each time another letter [A{65}...Z{90}]
					buffDateTx[4] = charVar; charVar ++; if (charVar > 90) charVar = 65;
										
					memcpy( xfer.txBuf, buffDateTx, 5); 
					xfer.txCnt = 5;
										
					printf ("\n Data {%s} succesfully sent! {%d}\n", xfer.txBuf, cnt);
					cnt++;						
					
					status = bscXfer(&xfer);
                    xfer.txCnt = 0;
					decodStatSlave (status);
  					}
				else
                    printf ("\n ERROR: the expected command {0x55} was not received !");		
				}
			}
		}
    else
        printf( "Failed to open as a slave!!!\n");
}

void closeSlave() 
{
    gpioInitialise();
    printf ("Initialized GPIOs\n");

    xfer.control = getControlBits(slaveAddress, 0);
    bscXfer(&xfer);
    printf ("Closed slave.\n");

    gpioTerminate();
    printf ( "Terminated GPIOs.\n");
}

void decodStatSlave (int statLoc)
{
    printf ("\n\n I2C status:");
    printf ("\n    # of bytes successfully copied to transmit FIFO: %d", ((statLoc & 0x1F0000) >> 16) );
    printf ("\n    # of bytes in receieve FIFO: %d", ((statLoc & 0xF800) >> 11));
    printf ("\n    # of bytes in transmit FIFO: %d", ((statLoc & 0xC0) >> 6));
    printf ("\n\n");
}

void statusSlave ()
{
	//gpioInitialise();   // => must to be executed previously
	
    xfer.control  = getControlBits(slaveAddress, 2);
    int returnVal = bscXfer(&xfer);
    
    printf ("\n\n I2C status:");
    printf ("\n    # of bytes successfully copied to transmit FIFO: %d", ((returnVal & 0x1F0000) >> 16) );
    printf ("\n    # of bytes in receieve FIFO: %d", ((returnVal & 0xF800) >> 11));
    printf ("\n    # of bytes in transmit FIFO: %d", ((returnVal & 0xC0) >> 6));
    printf ("\n\n");
}

int getControlBits(int address /* max 127 */, char open) 
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

    return ((address << 16 /*= to the start of significant bits*/) | flags);
}

