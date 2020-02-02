#include "ArduinoNano_I2Cdriver.h"

/* Constructor */
ArduinoNano_I2C::ArduinoNano_I2C() : I2C("MaxSonarEZ4", "/dev/MaxSonarEZ4", PX4_I2C_BUS_EXPANSION, MaxSonarEZ4_I2CADDR, MaxSonarEZ4_BUS_SPEED)
//																				                           0x3d 	       100KHz = 1000*100
{
	ArduinoNano_I2C::init();
}

/* Initialization of I2C device */
int ArduinoNano_I2C::init()
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

void ArduinoNano_I2C::readSensors(uint8_t my_cmd, unsigned char *buff)
{
	const uint8_t cmd = my_cmd;
	uint8_t data[8] = {0};

	//I2C::transfer(const uint8_t *send, const unsigned send_len, uint8_t *recv, const unsigned recv_len)
	if (OK != transfer(&cmd, 1, data, 8))
		{
		PX4_ERR("No Data received");
		}
	else
		{
		// switching read bytes
		//PX4_INFO("The text received: %s", data);
		memcpy (buff, data, 8);
		}
}
