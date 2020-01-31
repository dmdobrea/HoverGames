px4_add_module(
	MODULE examples__dd_RPi_i2cTest
	MAIN dd_RPi_i2cTest
	STACK_MAIN 2000
	COMPILE_FLAGS -O0
	SRCS
		dd_RPi_i2cTest.cpp
	DEPENDS
	)
