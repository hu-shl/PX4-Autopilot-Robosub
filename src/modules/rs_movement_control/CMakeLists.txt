px4_add_module(
	MODULE modules__rs_movement_control
	MAIN rs_movement_control
	STACK_MAIN 1200
	COMPILE_FLAGS
	SRCS
		rs_movement_control.cpp
		rs_movement_control.hpp
	MODULE_CONFIG
		rs_movement_control.yaml
	DEPENDS
		px4_work_queue
	)
