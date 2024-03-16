# Base variables
uint64   timestamp			  	# time since system start (microseconds)

# Data Acquisition Variables
uint8    node_id    				# MSCS node id
uint8    dynamic_node_id    			# Dynamic node id
uint8    event_type				# MSCS event type
uint8    source_module				# MSCS source module
uint8[5] event_data  	                       	# command arguments
uint8    event_handled  	              	# event handled flag
uint32   event_timeout_ms  	              	# event timeout in milliseconds

uint8    NEW_DEVICE = 1 		        # new device detected in Cyphal bus event
uint8    COMMAND_REQUEST = 2 		        # new command input from CLI

uint8    COMMAND_DATA_ACQUISITION_START = 0 	# start data acquisition
uint8    COMMAND_DATA_ACQUISITION_STOP  = 1 	# stop data acquisition

uint8    SOURCE_MSCS_POLL      = 0 		# event source MSCS poll
uint8    SOURCE_MSCS_COMMANDER = 1 		# event source MSCS commander

uint8    EVENT_NOT_HANDLED     = 0             	# event not handled flag
uint8    EVENT_HANDLED	       = 1    	        # event handled flag
