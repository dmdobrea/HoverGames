/**
 * @file dd_ultrasonic.cpp
 *
 * Example app for ultrasonic distance measurement with HRLV-MaxSonar-EZ4 & HRLV-MaxSonar-EZ0 using I2C
 *
 * @author Dan-Marius Dobrea 
  */

#include "dd_ultrasonic.h"
#include <px4_log.h>

extern "C" __EXPORT int dd_ultrasonic_main (int argc, char *argv[]);

/* Constructor */
dd_ultraS::dd_ultraS() : I2C("MaxSonarEZ4", "/dev/MaxSonarEZ4", PX4_I2C_BUS_EXPANSION, MaxSonarEZ4_I2CADDR, MaxSonarEZ4_BUS_SPEED)
//																				                0x3d 	       100KHz = 1000*100
{
	dd_ultraS::init();
}

/* Initialization of I2C device */
int dd_ultraS::init()
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

void dd_ultraS::readSensors(uint8_t my_cmd, double *usL, double *usC, double *usR, double *usB)
{
	double temp = 0;
	const uint8_t cmd = my_cmd;
	uint8_t data[4] = {0};
	data[0] = 12; data[1] = 34; data[2] = 56; data[3] = 0x00;

	temp = 0;
	//I2C::transfer(const uint8_t *send, const unsigned send_len, uint8_t *recv, const unsigned recv_len)
	if (OK != transfer(&cmd, 1, data, 3))
		{
		PX4_ERR("No Data received");
		temp = -1;
		*usB = temp;
		}
	else
		{
		// switching read bytes
		PX4_INFO("The text received: %s", data);
		}
}


/* main */
int dd_ultrasonic_main(int argc, char *argv[])
{
	PX4_INFO("Hello Hovergames from the Ultrasonic Sensor System (USS)!");
	
	dd_ultraS	ultrasonicData;
	
	double sensL, sensC, sensR, sensB;
	
	ultrasonicData.readSensors (0x55, &sensL, &sensC, &sensR, &sensB);
	ultrasonicData.readSensors (0x56, &sensL, &sensC, &sensR, &sensB);
	ultrasonicData.readSensors (0x57, &sensL, &sensC, &sensR, &sensB);

	PX4_INFO("Hovergames daiDrone exit"); 			// print in console 

	return 0;
} /* end: main */
