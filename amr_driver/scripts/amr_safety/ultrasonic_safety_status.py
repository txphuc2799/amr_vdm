#!/usr/bin/env python3
import rospy

from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from amr_msgs.msg import SafetyStatusStamped, SafetyStatus, SafetyZone

class UltrasonicSafety():

    def __init__(self):
        
        # Params
        self.default_distance_ = 0.3
        self.small_distance_   = 0.075
        
        # Setup loop frequency
        self.loop_freq_ = 5.0

        # Variables:
        self.is_running_ = False
        self.is_turn_off_ultrasonic_safety_ = False
        self.left_ultrasonic_range_ = 0.0
        self.right_ultrasonic_range_ = 0.0
        self.obstacle_state_ = SafetyStatus.NORMAL
        self.prev_obstacle_state_ = SafetyStatus.NORMAL
        self.safety_zone_type = SafetyZone.BIG_ZONE
        
        # Publishers:
        self.pub_ultrasonic_safety_status  = rospy.Publisher("ultrasonic_safety_status", SafetyStatusStamped, queue_size=5)

        # Subscribers:
        rospy.Subscriber("left_ultrasonic/range", Range, self.left_ultrasonic_range_callback)
        rospy.Subscriber("right_ultrasonic/range", Range, self.right_ultrasonic_range_callback)
        rospy.Subscriber("safety_zone_type", SafetyZone, self.safety_zone_type_callback)
        rospy.Subscriber("state_runonce_nav", Bool, self.runonce_callback)
        rospy.Subscriber("turn_off_ultrasonic_safety", Bool, self.turn_off_ultrasonic_safety_callback)
    

    def turn_off_ultrasonic_safety_callback(self, msg:Bool):
        self.is_turn_off_ultrasonic_safety_ = msg.data

    def runonce_callback(self, msg:Bool):
        self.is_running_ = msg.data

    def safety_zone_type_callback(self, msg:SafetyZone):
        self.safety_zone_type = msg.zone

    def left_ultrasonic_range_callback(self, msg:Range):
        self.left_ultrasonic_range_ = msg.range

    def right_ultrasonic_range_callback(self, msg:Range):
        self.right_ultrasonic_range_ = msg.range

    def run(self):
        while not rospy.is_shutdown():
            if (not self.is_running_ or
                self.is_turn_off_ultrasonic_safety_):
                self.obstacle_state_ = SafetyStatus.NORMAL
            else:
                if self.safety_zone_type == SafetyZone.SMALL_ZONE:
                    if (self.left_ultrasonic_range_ <= self.small_distance_ 
                        or self.right_ultrasonic_range_ <= self.small_distance_):
                        self.obstacle_state_ = SafetyStatus.PROTECTED

                    elif (self.left_ultrasonic_range_ > self.small_distance_ 
                          and self.right_ultrasonic_range_ > self.small_distance_):
                        self.obstacle_state_ = SafetyStatus.NORMAL       
                else:
                    if (self.left_ultrasonic_range_ <= self.default_distance_
                        or self.right_ultrasonic_range_ <= self.default_distance_):
                        self.obstacle_state_ = SafetyStatus.PROTECTED

                    elif (self.left_ultrasonic_range_ > self.default_distance_ 
                          and self.right_ultrasonic_range_ > self.default_distance_):
                        self.obstacle_state_ = SafetyStatus.NORMAL
                
                if self.obstacle_state_ != self.prev_obstacle_state_:
                    if self.obstacle_state_ == SafetyStatus.PROTECTED:
                        rospy.logwarn("UltrasonicSafety: Detect obstacle!")
                    else:
                        rospy.loginfo("UltrasonicSafety: No obstacle in range.")
                    
                    safety = SafetyStatusStamped()
                    safety.header.frame_id = "ultrasonic_link"
                    safety.header.stamp = rospy.Time.now()
                    safety.safety_status.status = self.obstacle_state_
                    self.pub_ultrasonic_safety_status.publish(safety)
                
                self.prev_obstacle_state_ = self.obstacle_state_
            
            rospy.sleep(1/self.loop_freq_)    


if __name__== '__main__':
    rospy.init_node("ultrasonic_safety")
    try:
        ultrasonic_safety = UltrasonicSafety()
        rospy.loginfo("UltrasonicSafety node is running!")
        ultrasonic_safety.run()

    except rospy.ROSInterruptException:
        pass
