/**
 * @file dd_ultrasonic.cpp
 *
 * Example app. for communication with a Raspberri Pi (RPi) development system using I2C bus
 * RPi development board is a slave device in this communication !
 *
 * @author Dan-Marius Dobrea
  */

#include "dd_RPi_i2cTest.h"
#include <px4_log.h>

extern "C" __EXPORT int dd_RPi_i2cTest_main (int argc, char *argv[]);

/* Constructor */
dd_RPi_i2cTest::dd_RPi_i2cTest() : I2C("RPi_i2c_test", "/dev/RPi_i2c_test", PX4_I2C_BUS_EXPANSION, RPi_I2CADDR, RPi_BUS_SPEED)
//																				                           0x4A    100KHz = 1000*100
{
	dd_RPi_i2cTest::init();
}

/* Initialization of I2C device */
int dd_RPi_i2cTest::init()
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

	return ret;
}

void dd_RPi_i2cTest::readDataFromRPi(uint8_t my_cmd, unsigned char *buffReceive)
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
		PX4_ERR("\nTransmission error !");
		}

	usleep(2000);

	if (OK != transfer(&cmd, 0, (uint8_t *) buffReceive, 5))
		{
		PX4_ERR("\nReceiving error !");
		}
}


void dd_RPi_i2cTest::wasteRPiFIFO_data (unsigned char *buffReceive)
{
	const uint8_t cmd = 0;

	if (OK != transfer(&cmd, 0, (uint8_t *) buffReceive, 5))
		{
		PX4_ERR("\nReceiving FIFO waste data error !");
		}
}


/* main */
int dd_RPi_i2cTest_main(int argc, char *argv[])
{
	int contor;
	unsigned char buff[50];

	PX4_INFO("Hello Hovergames - Drone to RPi I2C communication test was started !");

	dd_RPi_i2cTest	commRPi;

	contor = 0;
	do{
		contor++;
		if (contor > 15) break;
		commRPi.wasteRPiFIFO_data (buff);
	} while ( (buff[0] == buff[1]) && (buff[1] == buff[2]) && (buff[2] == buff[3]) && (buff[3] == buff[4]));

	for (int i=0; i<10; i++)
		{
		commRPi.readDataFromRPi (0x55, buff); buff[5] = 0;
		PX4_INFO("\nHovergames daiDrone received from RPi: %s \n", buff); 			// print in console 
		sleep(1);
		}

	PX4_INFO("\n\nHovergames daiDrone exit"); 			// print in console 

	return 0;
} /* end: main */
