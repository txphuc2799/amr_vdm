#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Int16, Int16MultiArray, Empty
from sensor_msgs.msg import Range
from amr_msgs.msg import SliderSensorStamped
from amr_driver.mcprotocol.type1e import Type1E


INFO  = rospy.loginfo
WARN  = rospy.logwarn
ERROR = rospy.logerr

class Parameter():
    PLC_port = 8001
    pub_frequency = 10
    left_ultrasonic_frame = 'left_ultrasonic_link'
    right_ultrasonic_frame = 'right_ultrasonic_link'
    min_range = 0.065
    max_range = 0.35
    field_of_view = 0.05236 

class PCReadPLC(Type1E):

    def __init__(self):
        
        super().__init__()
        rospy.init_node("PC_read_PLC")

        self.params = Parameter()
        self.initParams()

        # Parameter of PLC brigde:
        self.PLC_IP_address = rospy.get_param("~IP_addres_PLC", '192.168.0.250')

        self.rate = rospy.Rate(self.params.pub_frequency)
        
        # Connect to PLC
        self.connect_PLC(self.PLC_IP_address, self.params.PLC_port)
        if self._is_connected:
            INFO("PC_controller(READ): is connected to PLC success")
        else:
            ERROR("PC_controller(READ): can't connect to PLC")

        # Only Publishers:
        self.pub_left_ultrasonic  = rospy.Publisher("left_ultrasonic/range", Range, queue_size=5)
        self.pub_right_ultrasonic = rospy.Publisher("right_ultrasonic/range", Range, queue_size=5)
        self.pub_goal_name        = rospy.Publisher("START_AMR", Int16MultiArray, queue_size=5)
        self.pub_cmd_cancel_AMR   = rospy.Publisher("CANCEL_AMR", Bool,queue_size=5)
        self.pub_cmd_pause_AMR    = rospy.Publisher("PAUSE_AMR", Bool, queue_size=5)
        self.pub_cmd_reset_AMR    = rospy.Publisher("RESET_AMR", Empty, queue_size=1)
        self.pub_stop_amr         = rospy.Publisher("STOP_AMR", Bool, queue_size=1)
        self.pub_hand_control     = rospy.Publisher("HAND_CONTROL_AMR", Bool, queue_size=5)
        self.pub_EMS              = rospy.Publisher("emergency_stop", Bool, queue_size=5)
        self.pub_initialpose      = rospy.Publisher("pose_estimation", Int16, queue_size=1)
        self.pub_cart_sensor      = rospy.Publisher("cart_sensor_state", SliderSensorStamped, queue_size=5)
        self.pub_max_slider       = rospy.Publisher("slider_sensor_state", SliderSensorStamped, queue_size=5)
        self.pub_pickup_current_state = rospy.Publisher("pickup_current_state", Bool, queue_size=1)
        self.pub_drop_current_state   = rospy.Publisher("drop_current_state", Bool, queue_size=1)
        
        # Avariables:
        self.Emergency_STOP_state = 0
        self.START_state = 0
        self.Pause_AMR_state = 0
        self.RESET_AMR_state = 0
        self.hand_control_state = 0
        self.pause_timer = 0
        self.pause_off_delay = 2
        self.initialpose_state = 0
        self.pickup_current_state = 0
        self.drop_current_state = 0
        self.cart_sensor_state = [0,0]
        self.slider_sensor_state = [0,0]
        self.goal_name = Int16MultiArray()

    def initParams(self):
        print(f"NODE: PCReadPLC")
        print("PARAMETERS")

        param_names = [attr for attr in dir(self.params) if not callable(getattr(self.params, attr)) and not attr.startswith("__")]

        # get private rosparam, if none use default
        for param_name in param_names:
            param_val = rospy.get_param("PCReadPLC" + "/" + param_name, getattr(self.params, param_name))
            print(f"* /{param_name}: {param_val}")
            setattr(self.params, param_name, param_val)

    def handleUltrasonicSensor(self, data):

        for i in range(0,len(data)):
            data_msg = Range()
            data_msg.header.stamp = rospy.Time.now()
            data_msg.radiation_type = 0
            data_msg.field_of_view = self.params.field_of_view
            data_msg.min_range = self.params.min_range
            data_msg.max_range = self.params.max_range
            if data[i] >= 4000:
                data_msg.range = float("Inf")
            elif data[i] <= 5:
                data_msg.range = float("-Inf")
            else:
                data_msg.range = (65 + 0.07125 * data[i]) / 1000 

            if i == 0:
                data_msg.header.frame_id = self.params.left_ultrasonic_frame
                self.pub_left_ultrasonic.publish(data_msg)
            else:
                data_msg.header.frame_id = self.params.right_ultrasonic_frame
                self.pub_right_ultrasonic.publish(data_msg)

   
    def run(self):
        while not rospy.is_shutdown():
            # bit_array[0]  - M400: EMS bit
            # bit_array[1]  - M401: plc_control bit
            # bit_array[2]  - M402: pause bit
            # bit_array[3]  - M403: cancel bit
            # bit_array[4]  - M404: high current pickup bit
            # bit_array[5]  - M405: start bit
            # bit_array[6]  - M406: initialpose bit
            # bit_array[7]  - M407: left_checker_sensor
            # bit_array[8]  - M408: right_checker_sensor
            # bit_array[9]  - M409: original_slider_sensor
            # bit_array[10] - M410: max_slider_sensor
            # bit_array[11] - M411: manual hand control
            # bit_array[12] - M412: high current dropoff bit
            # bit_array[11] - M413: reset bit

            bit_array = self.batchread_bitunits("M400", 14)

            # reg_array[0]  - D600: ultrasonic sensor left
            # reg_array[1]  - D601: ultrasonic sensor right
            reg_array = self.batchread_wordunits("D600", 2)
            self.handleUltrasonicSensor(reg_array)

            # bit_array[0] - M400: EMS bit
            if (bit_array[0] != self.Emergency_STOP_state):
                if bit_array[0]:
                    INFO("PLCRead: EMS is ON.")
                    self.pub_stop_amr.publish(True)
                    self.pub_EMS.publish(True)
                    self.Emergency_STOP_state = 1

                else:
                    INFO("PLCRead: EMS is OFF.")
                    self.pub_stop_amr.publish(False)
                    self.pub_EMS.publish(False)
                    self.Emergency_STOP_state = 0

            # bit_array[5] - M405: start bit       
            if bit_array[5] != self.START_state:
                if bit_array[5]:
                    if not bit_array[11]:
                        line_name = self.batchread_wordunits("D502",1)[0]
                        mode_name = self.batchread_wordunits("D500",1)[0]
                        self.goal_name.data = [line_name,mode_name]
                        self.pub_goal_name.publish(self.goal_name)
                        self.START_state = 1
                else:
                    self.START_state = 0
            
            # bit_array[5] - M405: start bit
            if bit_array[5]:
                if not bit_array[11]:
                    # bit_array[2] - M402: pause bit
                    if bit_array[2] != self.Pause_AMR_state:
                        if bit_array[2]:
                            INFO("PLCRead: Pause is ON.")
                            self.pub_cmd_pause_AMR.publish(True)
                            self.pub_stop_amr.publish(True)
                            self.Pause_AMR_state = 1

                        elif not bit_array[11]:
                            if self.pause_timer <= self.pause_off_delay:
                                self.pause_timer += (1 / self.params.pub_frequency)
                            else:
                                INFO("PLCRead: Pause is OFF.")
                                self.pub_cmd_pause_AMR.publish(False)
                                self.pub_stop_amr.publish(False)
                                self.Pause_AMR_state = 0
                    else:
                        self.pause_timer = 0

                    # bit_array[3] - M403: cancel bit
                    if bit_array[3]:
                        WARN("Cancel Bit State: ON")
                        self.pub_cmd_cancel_AMR.publish(True)

                    # bit_array[4] - M404: high pickup current bit
                    if bit_array[4] != self.pickup_current_state:
                        if bit_array[4]:
                            self.pub_pickup_current_state.publish(True)
                            self.pickup_current_state = bit_array[4]
                        else:
                            self.pickup_current_state = 0
                    
                    # bit_array[12] - M412: high dropoff current bit
                    elif (bit_array[12] != self.drop_current_state):
                        if (bit_array[12]):
                            self.pub_drop_current_state.publish(True)
                            self.drop_current_state = bit_array[12]
                        else:
                            self.drop_current_state = 0
            else:
                if self.Pause_AMR_state:
                    self.Pause_AMR_state = 0
                    self.pause_timer = 0
                    self.pub_cmd_pause_AMR.publish(False)

            # bit_array[6] - M406: initialpose bit
            if bit_array[6] != self.initialpose_state:
                if bit_array[6]:
                    initial_pose_No = self.batchread_wordunits("D501",1)[0]
                    self.pub_initialpose.publish(initial_pose_No)
                    self.initialpose_state = 1
                else:
                    self.initialpose_state = 0
                    pass
            
            # bit_array[7] - M407: left_cart_sensor
            # bit_array[8] - M408: right_cart_sensor   
            if bit_array[7:9] != self.cart_sensor_state:
                cart_sensor = SliderSensorStamped()
                cart_sensor.header.stamp = rospy.Time.now()
                cart_sensor.sensor_state.sensor_name = ["left_cart, right_cart"]
                cart_sensor.sensor_state.state = bit_array[7:9]
                self.pub_cart_sensor.publish(cart_sensor)
                self.cart_sensor_state = bit_array[7:9]

            # bit_array[9]  - M409: original slider bit
            # bit_array[10] - M410: max slider bit
            if bit_array[9:11] != self.slider_sensor_state:
                slider_sensor = SliderSensorStamped()
                slider_sensor.header.stamp = rospy.Time.now()
                slider_sensor.sensor_state.sensor_name = ["origin_slider, max_slider"]
                slider_sensor.sensor_state.state = bit_array[9:11]
                self.pub_max_slider.publish(slider_sensor)
                self.slider_sensor_state = bit_array[9:11]

            # bit_array[11] - M411: hand_control_bit
            if bit_array[11] != self.hand_control_state:
                self.pub_hand_control.publish(bool(bit_array[11]))
                self.hand_control_state = bit_array[11]

            # bit_array[13] - M413: reset_bit
            if (bit_array[13] != self.RESET_AMR_state):
                if bit_array[13]:
                    msg = Empty()
                    self.pub_cmd_reset_AMR.publish(msg)
                    WARN("PLC reset AMR!")
                self.RESET_AMR_state = bit_array[13]

            self.rate.sleep()

        
if __name__== '__main__':
    try:
        PC_bridge_PLC = PCReadPLC()
        rospy.loginfo("PCReadPLC node is running!")
        PC_bridge_PLC.run()
    except rospy.ROSInterruptException:
        pass