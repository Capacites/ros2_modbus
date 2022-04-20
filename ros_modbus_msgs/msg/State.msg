std_msgs/Header header            # header info
bool state                        # Data validity

# errors:
# NO_ISSUE = 0,
# INITIALIZING = 1,
# NOT_CONNECTED = 2,
# INVALID_CONFIGURATION_FILE = 3,
# INVALID_IO_TYPE = 4,
# INVALID_IO_DATA_TYPE = 5,
# INVALID_IO_KEY = 6,
# INVALID_IO_TO_WRITE = 7

uint8 error 					  # error number
