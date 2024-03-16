# Base variables
uint64   timestamp			  	# time since system start (microseconds)

# Data Acquisition Variables
uint8    priority				# event priority
uint16   event_type				# MSCS Key Event type
uint16   event_source				# event source
uint16   device				     	# event device
char[64] event_data  	                       	# event data

# Priorities
uint8    PRIORITY_DEBUG	   		= 0	# debug  event priority
uint8    PRIORITY_LOW			= 1     # low    event priority
uint8    PRIORITY_MEDIUM		= 2     # medium event priority
uint8    PRIORITY_HIGH			= 3     # high   event priority

# Event Types
uint16   MSCS_SYSTEM_STARTUP       	= 1   	# system startup
uint16   MSCS_SYSTEM_SHUTDOWN      	= 2   	# system shutdown
uint16   MSCS_DEVICE_CONNECTED     	= 10  	# device connection detected
uint16   MSCS_DEVICE_DISCONNECTED  	= 11  	# device disconnection detected
uint16   MSCS_CALIBRATION_START    	= 30 	# device calibration started
uint16   MSCS_CALIBRATION_END      	= 31  	# device calibration end
uint16   MSCS_COMMAND_SENT      	= 50  	# command sent to device

# Event Sources
uint16   SOURCE_MSCS           		= 1   	# MSCS core
uint16   SOURCE_CLI           		= 2   	# MSCS core

# Devices
uint16   DEVICE_FLIGHT_CONTROLLER   	= 0   	# flight controller
uint16   DEVICE_LEFT_ADP       		= 22   	# left ADP
uint16   DEVICE_RIGHT_ADP       	= 23   	# right ADP

