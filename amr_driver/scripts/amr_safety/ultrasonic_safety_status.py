#!/usr/bin/env python3
import rospy

from sensor_msgs.msg import Range
from std_msgs.msg import Int16, Bool


class UltrasonicSafety():

    def __init__(self):
        
        # Params
        self.big_distance   = rospy.get_param("/amr/big_distance", 0.3)
        self.small_distance = rospy.get_param("/amr/small_distance", 0.075)

        self.rate = rospy.Rate(50)

        # Publishers:
        self.pub_ultrasonic_safety_status  = rospy.Publisher("ultrasonic_safety_status", Int16, queue_size=5)

        # Variables:
        self.is_run_once = False
        self.is_turn_off_ultrasonic_safety = False
        self.left_obstacle_state = 0
        self.right_obstacle_state = 0
        self.obstacle_state = 0
        self.prev_obstacle_state = 0
        self.safety_zone_type = 0            # [0: big zone, 1: small zone]

        # Subscribers:
        rospy.Subscriber("left_ultrasonic/range", Range, self.leftUltrasonicSensorCb)
        rospy.Subscriber("right_ultrasonic/range", Range, self.rightUltrasonicSensorCb)
        rospy.Subscriber("safety_zone_type", Int16, self.safetyZoneTypeCb)
        rospy.Subscriber("state_runonce_nav", Bool, self.runOnceStateCb)
        rospy.Subscriber("turn_off_ultrasonic_safety", Bool, self.turnOffUltrasonicSafetyCb)
    

    def turnOffUltrasonicSafetyCb(self, msg:Bool):
        self.is_turn_off_ultrasonic_safety = msg.data

    def runOnceStateCb(self, msg:Bool):
        self.is_run_once = msg.data

    def safetyZoneTypeCb(self, msg:Int16):
        if msg.data == 0 or msg.data == 1:
            msgs = "big" if msg.data == 0 else "small"
            self.safety_zone_type = msg.data
            rospy.logwarn(f"UltrasonicSafety: Switched to ultrasonic {msgs} safety zone")
        else: return

    def leftUltrasonicSensorCb(self, msg:Range):
        self.left_obstacle_state = msg.range

    def rightUltrasonicSensorCb(self, msg:Range):
        self.right_obstacle_state = msg.range


    def run(self):
        while not rospy.is_shutdown():
            if (not self.is_run_once or
                self.is_turn_off_ultrasonic_safety):
                pass
            else:
                if self.safety_zone_type:
                    if (self.left_obstacle_state <= self.small_distance 
                        or self.right_obstacle_state <= self.small_distance):
                        self.obstacle_state = 1
                    else:
                        self.obstacle_state = 0
                    
                    if self.obstacle_state != self.prev_obstacle_state:
                        if self.obstacle_state == 1:
                            self.pub_ultrasonic_safety_status.publish(1)
                            self.prev_obstacle_state = 1
                        else:
                            self.pub_ultrasonic_safety_status.publish(0)
                            self.prev_obstacle_state = 0           
                else:
                    if (self.left_obstacle_state <= self.big_distance
                        or self.right_obstacle_state <= self.big_distance):
                        self.obstacle_state = 1
                    else:
                        self.obstacle_state = 0
                    
                    if self.obstacle_state != self.prev_obstacle_state:
                        if self.obstacle_state == 1:
                            self.pub_ultrasonic_safety_status.publish(1)
                            self.prev_obstacle_state = 1
                        else:
                            self.pub_ultrasonic_safety_status.publish(0)
                            self.prev_obstacle_state = 0
            
            self.rate.sleep()    


if __name__== '__main__':
    rospy.init_node("ultrasonic_safety")
    try:
        ultrasonic_safety = UltrasonicSafety()
        rospy.loginfo("UltrasonicSafety node is running!")
        ultrasonic_safety.run()

    except rospy.ROSInterruptException:
        pass
