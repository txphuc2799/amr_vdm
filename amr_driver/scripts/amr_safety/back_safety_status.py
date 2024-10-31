#!/usr/bin/env python3
import rospy

from sick_safetyscanners.msg import OutputPathsMsg
from std_msgs.msg import Int16, Bool


class BackScannerSafety():

    def __init__(self):

        # Publishers:
        self.pub_back_scanner_status  = rospy.Publisher("back_scanner_status", Int16, queue_size=5)

        # Subscribers:
        rospy.Subscriber("/sick_safetyscanners_back/output_paths", OutputPathsMsg, self.backSafetyStatusCb)
        rospy.Subscriber("turn_off_back_safety", Bool, self.turnOffBackSafetyScannerCb)

        # Variables:
        self.is_turn_off_back_scanner = False
        self.obstacle_state = 0 # [0: No obtacles, 1: Zone protected, 2: Zone warning 1]
        self.prev_obstacle_state = 0
    
    def turnOffBackSafetyScannerCb(self, msg:Bool):
        self.is_turn_off_back_scanner = msg.data

    def backSafetyStatusCb(self, msg: OutputPathsMsg):
        
        if self.is_turn_off_back_scanner:
            return
        
        if not msg.status[0]:
            self.obstacle_state = 1
        else:
            self.obstacle_state = 0
        
        if self.obstacle_state != self.prev_obstacle_state:
            if self.obstacle_state == 1:
                self.pub_back_scanner_status.publish(1)
                rospy.logwarn("BackScannerSafety: Detect obstacle!")
                self.prev_obstacle_state = 1
            
            else:
                self.pub_back_scanner_status.publish(0)
                rospy.loginfo("BackScannerSafety: No obstacle in field.")
                self.prev_obstacle_state = 0


if __name__== '__main__':
    rospy.init_node("back_scanner_safety_status")
    try:
        back_scanner_safety = BackScannerSafety()
        rospy.loginfo("BackScannerSafety node is running!")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
