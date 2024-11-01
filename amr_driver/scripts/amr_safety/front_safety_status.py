#!/usr/bin/env python3
import rospy

from sick_safetyscanners.msg import OutputPathsMsg
from std_msgs.msg import Bool
from amr_msgs.msg import SafetyStatusStamped, SafetyStatus


class FrontScannerSafety():

    def __init__(self):
        
        # Variables:
        self.is_turn_off_ = False
        self.is_running_  = False
        self.obstacle_state_      = SafetyStatus.NORMAL
        self.prev_obstacle_state_ = SafetyStatus.NORMAL

        # Publishers:
        self.pub_front_scanner_status = rospy.Publisher("front_scanner_status", SafetyStatusStamped, queue_size=5)
    
        # Subscribers:
        rospy.Subscriber("/sick_safetyscanners_front/output_paths", OutputPathsMsg, self.front_safety_state_callback)
        rospy.Subscriber("turn_off_front_safety", Bool, self.turn_off_front_safety_callback)
        rospy.Subscriber("state_runonce_nav", Bool, self.runonce_callback)

    def runonce_callback(self,msg: Bool):
        self.is_running_ = msg.data

    def turn_off_front_safety_callback(self, msg:Bool):
        self.is_turn_off_ = msg.data


    def front_safety_state_callback(self, msg: OutputPathsMsg):
        if self.is_turn_off_ or not self.is_running_:
            return
        
        if not msg.status[0]:
            self.obstacle_state_ = SafetyStatus.PROTECTED
        elif not msg.status[1]:
            self.obstacle_state_ = SafetyStatus.WARNING_LV1
        elif not msg.status[2]:
            self.obstacle_state_ = SafetyStatus.WARNING_LV2
        else:
            self.obstacle_state_ = SafetyStatus.NORMAL

        if self.obstacle_state_ != self.prev_obstacle_state_:
            if self.obstacle_state_ == SafetyStatus.PROTECTED:
                rospy.logwarn("FrontScannerSafety: Detect obstacle!")
                
            elif self.obstacle_state_ == SafetyStatus.NORMAL:
                rospy.loginfo("FrontScannerSafety: No obstacle in field.")

            safety = SafetyStatusStamped()
            safety.header.frame_id = "front_laser_link"
            safety.header.stamp = rospy.Time.now()
            safety.safety_status.status = self.obstacle_state_
            self.pub_front_scanner_status.publish(safety)

        self.prev_obstacle_state_ = self.obstacle_state_


if __name__== '__main__':
    rospy.init_node("front_scanner_safety_status")
    try:
        front_scanner_safety = FrontScannerSafety()
        rospy.loginfo("FrontScannerSafety node is running!")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
