############################################################################
#
#   Copyright (c) 2024 Daniel Montes - daniel.montestolon@slu.edu
#   AirCRAFT Lab - Saint Louis University
#
############################################################################

px4_add_module(
	MODULE modules__mscs
	MAIN mscs
	COMPILE_FLAGS
		#-DDEBUG_BUILD   # uncomment for PX4_DEBUG output
		#-O0             # uncomment when debugging
	SRCS
		MSCS.cpp
	DEPENDS
		px4_work_queue
	)
