# Events interface
uint64 timestamp			# time since system start (microseconds)

uint16 id                   # Event ID
uint16 sequence             # Sequence number
uint8[16] arguments         # (optional) arguments, depend on event id 

uint8 log_level_internal    # Log level for internal logging
uint8 log_level_external    # Log level for external (sending via MAVLink)

uint8 ORB_QUEUE_LENGTH = 5

