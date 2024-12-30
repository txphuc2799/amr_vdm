#!/usr/bin/env python3
import rospy
import dynamic_reconfigure.client as dc

from std_msgs.msg import Bool, Float32
from amr_msgs.msg import SliderSensorStamped
from amr_msgs.msg import SafetyStatusStamped, SafetyStatus

IN = 1
OUT = 1

class SafetyController():

    def __init__(self):

        self.speed_per_at_warninglv1 = rospy.get_param("~speed_per_at_warninglv1", 0.55)
        self.speed_per_at_warninglv2 = rospy.get_param("~speed_per_at_warninglv2", 0.7)

        # Setup loop frequency
        self.loop_freq_ = 10.0

        # Move base dynamic reconfigure:
        self.timeout_obstacles = {'oscillation_timeout': 0.0}
        self.timeout_normal = {'oscillation_timeout': 15.0}

        # Footprint dynamic reconfigure:
        self.default_footprint = {'footprint': [[0.22,0.22],[-0.6,0.22],[-0.6,-0.22],[0.22,-0.22]]}
        self.big_footprint = {'footprint': [[0.22,0.22],[-0.92,0.22],[-0.92,-0.22],[0.22,-0.22]]}

        # Avariables:
        self.is_running_ = False
        self.is_pausing_ = False
        self.is_turn_off_back_safety_  = False
        self.is_turn_off_front_safety_ = False
        self.is_turn_off_ultrasonic_   = False
        self.back_safety_state_        = SafetyStatus.NORMAL
        self.front_safety_state_       = SafetyStatus.NORMAL
        self.ultrasonic_safety_status_ = SafetyStatus.NORMAL
        self.field_state_      = SafetyStatus.NORMAL
        self.prev_field_state_ = SafetyStatus.NORMAL

        # Create move_base client
        self.client_movebase = dc.Client("/move_base_node")
        self.global_footprint_client = dc.Client("/move_base_node/global_costmap")
        self.local_footprint_client = dc.Client("/move_base_node/local_costmap")

        # Publisher
        self.pub_status_protected_field = rospy.Publisher("status_protected_field", Bool, queue_size=5)
        self.speed_at_field_pub_ = rospy.Publisher("speed_at_field", Float32, queue_size=5)

        # Subscribers:
        rospy.Subscriber("front_scanner_status", SafetyStatusStamped, self.front_safety_status_callback)
        rospy.Subscriber("back_scanner_status", SafetyStatusStamped, self.back_safety_status_callback)
        rospy.Subscriber("ultrasonic_safety_status", SafetyStatusStamped, self.ultrasonic_safety_callback)
        rospy.Subscriber("state_runonce_nav", Bool, self.runonce_callback)
        rospy.Subscriber('PAUSE_AMR', Bool, self.pause_callback)
        rospy.Subscriber("turn_off_back_safety",Bool,self.turn_off_back_safety_callback)
        rospy.Subscriber("turn_off_front_safety", Bool, self.turn_off_front_safety_callback)
        rospy.Subscriber("turn_off_ultrasonic_safety", Bool, self.turn_off_ultrasonic_safety_callback)
        rospy.Subscriber("slider_sensor_state", SliderSensorStamped, self.switch_footprint_callback)

    
    def switch_footprint_callback(self, msg:SliderSensorStamped):
        if msg.sensor_state.state:
            if msg.sensor_state.state[0] == IN:
                self.global_footprint_client.update_configuration(self.default_footprint)
                self.local_footprint_client.update_configuration(self.default_footprint)
            elif msg.sensor_state.state[1] == OUT:
                self.global_footprint_client.update_configuration(self.big_footprint)
                self.local_footprint_client.update_configuration(self.big_footprint)

    def runonce_callback(self,msg: Bool):
        self.is_running_ = msg.data
        if not self.is_running_:
            self.pub_status_protected_field.publish(False)

    def pause_callback(self,msg: Bool):
        self.is_pausing_ = msg.data
        if self.is_pausing_:
            self.pub_status_protected_field.publish(False)

    def turn_off_back_safety_callback(self,msg: Bool):
        self.is_turn_off_back_safety_ = msg.data
        if self.is_turn_off_back_safety_:
            self.back_safety_state_ = SafetyStatus.NORMAL
    
    def turn_off_front_safety_callback(self, msg:Bool):
        self.is_turn_off_front_safety_ = msg.data
        if msg.data:
            self.front_safety_state_ = SafetyStatus.NORMAL

    def turn_off_ultrasonic_safety_callback(self, msg:Bool):
        self.is_turn_off_ultrasonic_= msg.data
        if msg.data:
            self.ultrasonic_safety_status_ = SafetyStatus.NORMAL

    def configureOscillationTimeOut(self,config):
        if self.is_pausing_:
            return
        self.client_movebase.update_configuration(config)

    def front_safety_status_callback(self, msg:SafetyStatusStamped):
        if self.is_turn_off_front_safety_:
            return
        self.front_safety_state_ = msg.safety_status.status

    def back_safety_status_callback(self, msg:SafetyStatusStamped):
        if self.is_turn_off_back_safety_:
            return
        self.back_safety_state_ = msg.safety_status.status
    
    def ultrasonic_safety_callback(self, msg:SafetyStatusStamped):
        if self.is_turn_off_ultrasonic_ :
            return
        self.ultrasonic_safety_status_ = msg.safety_status.status

    def update_velocity(self, field):
        if field == SafetyStatus.WARNING_LV1:
            speed_limit_per = self.speed_per_at_warninglv1  #%
        elif field == SafetyStatus.WARNING_LV2:
            speed_limit_per = self.speed_per_at_warninglv2  #%
        elif field == SafetyStatus.NORMAL:
            speed_limit_per = 1.0   #%

        msg = Float32()
        msg.data = speed_limit_per
        self.speed_at_field_pub_.publish(msg)

    def run(self):
        delay_time = 0.0
        while not rospy.is_shutdown():
            if not self.is_running_ or self.is_pausing_:
                self.field_state_ = SafetyStatus.NORMAL
            else:
                if (self.front_safety_state_ == SafetyStatus.PROTECTED
                    or self.back_safety_state_ == SafetyStatus.PROTECTED
                    or self.ultrasonic_safety_status_ == SafetyStatus.PROTECTED):
                    
                    delay_time = 0.0
                    if self.field_state_ != SafetyStatus.PROTECTED:
                        self.field_state_ = SafetyStatus.PROTECTED
                        self.pub_status_protected_field.publish(True)
                        self.configureOscillationTimeOut(self.timeout_obstacles)
                else:
                    if (self.field_state_ == SafetyStatus.PROTECTED):
                        if delay_time >= 2.0:
                            self.pub_status_protected_field.publish(False)
                            self.field_state_ = SafetyStatus.NORMAL
                            delay_time = 0.0
                        else: 
                            delay_time += 1/self.loop_freq_
                    
                    if (self.front_safety_state_ != SafetyStatus.PROTECTED
                        and self.field_state_ != SafetyStatus.PROTECTED):
                        self.field_state_ = self.front_safety_state_
                    
                    if (self.field_state_ != self.prev_field_state_):
                        delay_time = 0.0
                        self.configureOscillationTimeOut(self.timeout_normal)
                        self.update_velocity(self.front_safety_state_)
                    
                self.prev_field_state_ = self.field_state_
            
            rospy.sleep(1/self.loop_freq_)    


if __name__== '__main__':
    rospy.init_node("safety_controller")
    try:
        safety_controller = SafetyController()
        rospy.loginfo("SafetyController Node is running!")
        safety_controller.run()
        
    except rospy.ROSInterruptException:
        pass
