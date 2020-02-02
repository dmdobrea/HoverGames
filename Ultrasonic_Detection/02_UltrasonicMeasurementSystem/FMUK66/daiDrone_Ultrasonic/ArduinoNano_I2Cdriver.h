#include <px4_i2c.h>

#include <drivers/device/i2c.h>					//if it is not included, error: 'device' has not been declared

#define MaxSonarEZ4_I2CADDR    0x3d
#define MaxSonarEZ4_BUS_SPEED  1000 * 100

class ArduinoNano_I2C : public device::I2C
{
  public:
	/**
	 * @ Constructor
	 *
	 */
	ArduinoNano_I2C();
	virtual ~ArduinoNano_I2C() = default;

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
	void readSensors(uint8_t my_cmd, unsigned char *buff);

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
