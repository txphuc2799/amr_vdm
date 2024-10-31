# # The robot has to be in a predefined state
# uint8 ROBOT_STATE_NONE = 0
# uint8 ROBOT_STATE_STARTING = 1
# uint8 ROBOT_STATE_SHUTTINGDOWN = 2
# uint8 ROBOT_STATE_READY = 3		# ready to execute
# uint8 ROBOT_STATE_PAUSE = 4		# pause from executing
# uint8 ROBOT_STATE_EXECUTING = 5		# when running in mission/taxa/bus
# uint8 ROBOT_STATE_ABORTED = 6
# uint8 ROBOT_STATE_COMPLETED = 7		# done executing
# uint8 ROBOT_STATE_DOCKED = 8		# in the dock and charging the batteries
# uint8 ROBOT_STATE_DOCKING = 9
# uint8 ROBOT_STATE_EMERGENCYSTOP = 10	# the robot has emg-stop activated
# uint8 ROBOT_STATE_MANUALCONTROL = 11	# a pause state, where the robot can move
# uint8 ROBOT_STATE_ERROR = 12		# a general error state, requires a error handle

# uint8 robotState
# string robotStateString

uint8 mode
uint8 MODE_IDLE = 0
uint8 MODE_CHARGE = 1
uint8 MODE_PICKUP = 2
uint8 MODE_DROPOFF = 3
uint8 MODE_UNDOCK = 4
uint8 MODE_GOOUT = 5