px4_add_module(
	MODULE examples__dd_ultrasonic
	MAIN dd_ultrasonic
	STACK_MAIN 2000
	COMPILE_FLAGS -O0
	SRCS
		dd_ultrasonic.cpp
	DEPENDS
	)
