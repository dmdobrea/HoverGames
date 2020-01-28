px4_add_module(
	MODULE examples__daiDrone_HD
	MAIN daiDrone_HD
	STACK_MAIN 2000
	COMPILE_FLAGS -O0
	SRCS
		daiDrone_HD.cpp
	DEPENDS
	)
