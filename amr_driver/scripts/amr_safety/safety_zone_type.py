#!/usr/bin/env python3
import rospy

from std_msgs.msg import Bool
from amr_v3_msgs.msg import SafetyZone


class SafetyZoneType():

    def __init__(self):
        # Publishers:
        self.safety_zone_pub_ = rospy.Publisher("safety_zone_type", SafetyZone, queue_size=5)

        # Subscribers:
        rospy.Subscriber("/safety_filter/safety_state", Bool, self.safety_state_callback)

    def pub_safety_zone(self, type):
        msg = SafetyZone()
        msg.zone = type
        self.safety_zone_pub_.publish(msg)

    def safety_state_callback(self, msg: Bool):
        if msg.data:
            self.pub_safety_zone(SafetyZone.SMALL_ZONE)
        else:
            self.pub_safety_zone(SafetyZone.BIG_ZONE)


if __name__== '__main__':
    rospy.init_node("safety_zone_type")
    try:
        safety_zone_type = SafetyZoneType()
        rospy.loginfo("SafetyZoneType node is running!")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
        