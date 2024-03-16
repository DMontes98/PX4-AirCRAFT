# Base variables
uint64   timestamp			  	# time since system start (microseconds)

# Data Acquisition Variables
uint8    node_id    				# timestamp at which the data point was acquired
uint16   command				# command
uint8    response				# response from node
uint8[4] command_args  	                       	# command arguments
uint8	 command_handled			# command handled flag
uint32   command_timeout_ms			# command timeout in ms
