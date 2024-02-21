# Base variables
uint64 timestamp			  	# time since system start (microseconds)
bool   module_active			  	# is the MSCS module running

# Left ADP
bool   left_adp_active			  	# is the Left ADP active
uint64 left_adp_last_heartbeat_timestamp  	# last heartbeat received

# Right ADP
bool   right_adp_active			  	# is the Right ADP active
uint64 right_adp_last_heartbeat_timestamp 	# last heartbeat received
