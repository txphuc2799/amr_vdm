#!/usr/bin/env python3
import rospy

from sick_safetyscanners.msg import OutputPathsMsg
from std_msgs.msg import Int16, Bool


class FrontScannerSafety():

    def __init__(self):

        # Publishers:
        self.pub_front_scanner_status = rospy.Publisher("front_scanner_status", Int16, queue_size=5)

        # Variables:
        self.obstacle_state = 0 # [0: No obtacles, 1: Zone protected, 2: Zone warning 1, 3: Zone warning 2]
        self.prev_obstacle_state = 0
        self.is_turn_off_front_scanner = False
    
        # Subscribers:
        rospy.Subscriber("/sick_safetyscanners_front/output_paths", OutputPathsMsg, self.frontScannerStatusCb)
        rospy.Subscriber("turn_off_front_safety", Bool, self.turnOffFrontSafetyScannerCb)


    def turnOffFrontSafetyScannerCb(self, msg:Bool):
        self.is_turn_off_front_scanner = msg.data


    def frontScannerStatusCb(self, msg: OutputPathsMsg):
        if self.is_turn_off_front_scanner:
            return
        
        if not msg.status[0]:
            self.obstacle_state = 1
        elif not msg.status[1]:
            self.obstacle_state = 2
        elif not msg.status[2]:
            self.obstacle_state = 3
        else:
            self.obstacle_state = 0

        if self.obstacle_state != self.prev_obstacle_state:
            if self.obstacle_state == 1:
                self.pub_front_scanner_status.publish(1)
                rospy.logwarn("FrontScannerSafety: Detect obstacle!")
                self.prev_obstacle_state = 1
                
            elif self.obstacle_state == 2:
                self.pub_front_scanner_status.publish(2)
                self.prev_obstacle_state = 2

            elif self.obstacle_state == 3:
                self.pub_front_scanner_status.publish(3)
                self.prev_obstacle_state = 3
                
            else:
                self.pub_front_scanner_status.publish(0)
                rospy.loginfo("FrontScannerSafety: No obstacle in field.")
                self.prev_obstacle_state = 0


if __name__== '__main__':
    rospy.init_node("front_scanner_safety_status")
    try:
        front_scanner_safety = FrontScannerSafety()
        rospy.loginfo("FrontScannerSafety node is running!")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
