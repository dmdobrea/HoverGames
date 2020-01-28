/*
 * daiDrone_HD.h
 *
 *  Created on: Jan 15, 2020
 *      Author: Dobrea Dan Marius
 */

#include <px4_i2c.h>

#include <drivers/device/i2c.h>					//if it is not included, error: 'device' has not been declared

#define RPi_I2CADDR    0x4A
#define RPi_BUS_SPEED  1000 * 400

class dd_RPi_i2c : public device::I2C
{
  public:
	/**
	 * @ Constructor
	 *
	 */
	dd_RPi_i2c();
	virtual ~dd_RPi_i2c() = default;

	/**
	 * Initialization of the I2C device.
	 *
	 * @return ret status of initialization
	 *
	 */
	int init();

	/**
	 * Read the object temperature in degrees Celsius.
	 *
	 * @return temp read object temperature
	 *
	 */
	void readDataFromRPi (uint8_t my_cmd, unsigned char *buffReceive);
	void wasteRPiFIFO_data (unsigned char *buffReceive);

  private:
	/**
	 * Read the temerature in degrees Celsius.
	 *
	 * @param reg register of the temperature to be read
	 * @return temp read temperature
	 *
	 */
	//double readTemp(uint8_t reg);
};
