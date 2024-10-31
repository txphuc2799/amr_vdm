#!/usr/bin/env python3
import rospy
import dynamic_reconfigure.client as dc

from std_msgs.msg import Bool,Int16, Float32
from amr_msgs.msg import CheckerSensorStateStamped

IN = 1
OUT = 1

class SafetyController():

    def __init__(self):

        # Get params from server
        self.pub_frequency = rospy.get_param("pub_frequency", 10)
        self.max_speed = rospy.get_param("max_speed", 0.7)
        self.slow_speed_zone_0 = rospy.get_param("slow_speed_zone_0", 0.2)
        self.slow_speed_zone_1 = rospy.get_param("slow_speed_zone_1", 0.3)
        self.slow_speed_zone_2 = rospy.get_param("slow_speed_zone_2", 0.4)

        # Publishers:
        self.pub_status_protected_field = rospy.Publisher("status_protected_field", Bool, queue_size=5)
        self.pub_speed_limit_safety = rospy.Publisher("speed_limit_safety", Float32, queue_size=5)

        self.rate = rospy.Rate(self.pub_frequency)

        # Move base dynamic reconfigure:
        self.param_oscillation_timeout_obstacles = {'oscillation_timeout': 0.0}
        self.param_oscillation_timeout_normal = {'oscillation_timeout': 15.0}

        # Footprint dynamic reconfigure:
        self.default_footprint = {'footprint': [[0.22,0.22],[-0.6,0.22],[-0.6,-0.22],[0.22,-0.22]]}
        self.big_footprint = {'footprint': [[0.22,0.22],[-0.92,0.22],[-0.92,-0.22],[0.22,-0.22]]}

        # Avariables:
        self.is_running_ = False
        self.is_pausing_ = False
        self.is_turn_off = False
        self.is_turn_off_front = False
        self.is_turn_off_ultrasonic = False
        self.status_field_back = 0
        self.status_field_front = 0
        self.ultrasonic_safety_status = 0
        self.status_field = 0

        # Subscribers:
        rospy.Subscriber("front_scanner_status", Int16, self.frontFieldStatusCb)
        rospy.Subscriber("back_scanner_status", Int16, self.backFieldStatusCb)
        rospy.Subscriber("ultrasonic_safety_status", Int16, self.ultrasonicSafetyCb)
        rospy.Subscriber("state_runonce_nav", Bool, self.runonce_callback)
        rospy.Subscriber('PAUSE_AMR', Bool, self.pause_callback)
        rospy.Subscriber("turn_off_back_safety",Bool,self.turnOffBackSafetyScannerCb)
        rospy.Subscriber("turn_off_front_safety", Bool, self.turnOffFrontSafetyScannerCB)
        rospy.Subscriber("turn_off_ultrasonic_safety", Bool, self.turnOffUltrasonicSafetyCB)
        rospy.Subscriber("slider_sensor_state", CheckerSensorStateStamped, self.switchFootprintCb)

    
    def switchFootprintCb(self, msg:CheckerSensorStateStamped):

        global_footprint_client = dc.Client("/move_base_node/global_costmap")
        local_footprint_client = dc.Client("/move_base_node/local_costmap")

        if msg.sensor_state.data:
            if msg.sensor_state.data[0] == IN:
                global_footprint_client.update_configuration(self.default_footprint)
                local_footprint_client.update_configuration(self.default_footprint)
            elif msg.sensor_state.data[1] == OUT:
                global_footprint_client.update_configuration(self.big_footprint)
                local_footprint_client.update_configuration(self.big_footprint)

    def runonce_callback(self,msg: Bool):
        self.is_running_ = msg.data
        if not self.is_running_:
            self.pub_status_protected_field.publish(False)

    def pause_callback(self,msg: Bool):
        self.is_pausing_ = msg.data
        if self.is_pausing_:
            self.pub_status_protected_field.publish(False)

    def turnOffBackSafetyScannerCb(self,msg: Bool):
        self.is_turn_off = msg.data
    
    def turnOffFrontSafetyScannerCB(self, msg:Bool):
        self.is_turn_off_front = msg.data
        if msg.data:
            self.status_field_front = 0

    def turnOffUltrasonicSafetyCB(self, msg:Bool):
        self.is_turn_off_ultrasonic = msg.data
        if msg.data:
            self.ultrasonic_safety_status = 0

    def configureOscillationTimeOut(self,config):
        if self.is_pausing_:
            return
        self.client_movebase = dc.Client("/move_base_node")
        self.client_movebase.update_configuration(config)

    def frontFieldStatusCb(self,msg: Int16):
        if self.is_turn_off_front:
            return
        self.status_field_front = msg.data

    def backFieldStatusCb(self,msg: Int16):
        if self.is_turn_off:
            return
        self.status_field_back = msg.data
    
    def ultrasonicSafetyCb(self, msg:Int16):
        if self.is_turn_off_ultrasonic:
            return
        self.ultrasonic_safety_status = msg.data

    def main(self):
        pulse_2s = 0
        while not rospy.is_shutdown():
            if not self.is_running_ or self.is_pausing_:
                self.status_field = 0
            else:
                if (self.status_field_front == 1
                    or self.status_field_back == 1
                    or self.ultrasonic_safety_status == 1):
                    pulse_2s = 0
                    if self.status_field != 1:
                        self.pub_status_protected_field.publish(True)
                        self.configureOscillationTimeOut(self.param_oscillation_timeout_obstacles)
                        self.status_field = 1                 
                else:
                    if (self.status_field == 1):
                        if pulse_2s == 20 :
                            self.pub_status_protected_field.publish(False)
                            self.configureOscillationTimeOut(self.param_oscillation_timeout_normal)
                            self.pub_speed_limit_safety.publish(self.slow_speed_zone_0)
                            self.status_field = 0
                            pulse_2s = 0
                        else: 
                            pulse_2s += 1
                
                    elif self.status_field_front == 2:
                        pulse_2s = 0
                        if self.status_field != 2:
                            # self.pub_status_protected_field.publish(False)
                            self.configureOscillationTimeOut(self.param_oscillation_timeout_normal)
                            self.pub_speed_limit_safety.publish(self.slow_speed_zone_1)
                            self.status_field = 2

                    elif self.status_field_front == 3:
                        pulse_2s = 0
                        if self.status_field != 3:
                            # self.pub_status_protected_field.publish(False)
                            self.configureOscillationTimeOut(self.param_oscillation_timeout_normal)
                            self.pub_speed_limit_safety.publish(self.slow_speed_zone_2)
                            self.status_field = 3
                        
                    elif self.status_field != 0:
                        if pulse_2s == 10:
                            # self.pub_status_protected_field.publish(False)
                            self.configureOscillationTimeOut(self.param_oscillation_timeout_normal)
                            self.pub_speed_limit_safety.publish(self.max_speed)
                            self.status_field = 0
                        else: pulse_2s += 1
            
            self.rate.sleep()    


if __name__== '__main__':
    rospy.init_node("safety_controller")
    try:
        safety_controller = SafetyController()
        rospy.loginfo("SafetyController Node is running!")
        safety_controller.main()
        
    except rospy.ROSInterruptException:
        pass