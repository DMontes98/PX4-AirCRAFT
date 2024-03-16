# Base variables
uint64 timestamp			  	# time since system start (microseconds)
bool   module_active			  	# is the MSCS module running
uint8  state					# state of the system

# Left ADP
uint8  left_adp_node_id			        # left ADP node id
bool   left_adp_active			  	# is the Left ADP active
uint64 left_adp_last_heartbeat_timestamp  	# last heartbeat received
uint32 left_adp_uptime  	                # node uptime since last boot
uint8  left_adp_mode                     	# current mode of operation
uint8  left_adp_status                     	# node status

# Right ADP
uint8  right_adp_node_id			# right ADP node id
bool   right_adp_active			  	# is the Left ADP active
uint64 right_adp_last_heartbeat_timestamp  	# last heartbeat received
uint32 right_adp_uptime  	                # node uptime since last boot
uint8  right_adp_mode                     	# current mode of operation
uint8  right_adp_status                     	# node status


uint8 MSCS_STATE_STARTUP     = 0
uint8 MSCS_STATE_READY       = 1
uint8 MSCS_STATE_CALIBRATION = 2
