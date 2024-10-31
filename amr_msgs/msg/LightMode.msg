# The robot operates in different mode
uint8 LIGHT_MODE_IDLE = 0
uint8 LIGHT_MODE_PAUSE = 1		# in mapping the recorded map is being finalised
uint8 LIGHT_MODE_WAITING = 2		# primary mode when executing a mission (action list)
uint8 LIGHT_MODE_MOVING = 3
uint8 LIGHT_MODE_ERROR = 4

uint8 lightMode
