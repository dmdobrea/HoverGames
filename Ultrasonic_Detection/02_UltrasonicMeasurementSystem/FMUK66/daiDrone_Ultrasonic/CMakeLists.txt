px4_add_module(
	MODULE examples__daiDrone_Ultrasonic
	MAIN daiDrone_Ultrasonic
	STACK_MAIN 2000
	COMPILE_FLAGS -O0
	SRCS
		daiDrone_Ultrasonic.cpp
		ArduinoNano_I2Cdriver.cpp
	DEPENDS
	)
