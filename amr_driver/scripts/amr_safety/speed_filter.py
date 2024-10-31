#!/usr/bin/env python3
import rospy
import dynamic_reconfigure.client as dc

from std_msgs.msg import Bool


class SpeedFilter():

    def __init__(self):

        # self.dwb_update_vel = dc.Client("/move_base_node/RotationShimController/DWBController")
        self.vpp_update_vel = dc.Client("/move_base_node/RotationShimController/VectorPursuitController")

        # DWB local planner dynamic reconfigure
        self.DWB_slow_speed   = {'max_speed_xy': 0.5,'max_vel_x': 0.5, 'max_vel_theta': 0.5}
        self.DWB_normal_speed = {'max_speed_xy': 0.7,'max_vel_x': 0.7, 'max_vel_theta': 0.7}

        # VPP dynamic reconfigure
        self.VPP_slow_speed_   = {'desired_linear_vel': 0.5}
        self.VPP_normal_speed_ = {'desired_linear_vel': 0.7}
    
        # Subscribers:
        rospy.Subscriber("/binary_state", Bool, self.binary_state_callback)

    def setVelocity(self, msg):
        # if msg:
        #     self.dwb_update_vel.update_configuration(self.DWB_slow_speed)
        #     rospy.loginfo("SpeedFilter: Set controller velocity to slow speed.")
        # else:
        #     self.dwb_update_vel.update_configuration(self.DWB_normal_speed)
        #     rospy.loginfo("SpeedFilter: Set controller velocity to normal speed.")
        
        if msg:
            self.vpp_update_vel.update_configuration(self.VPP_slow_speed_)
            rospy.loginfo("SpeedFilter: Set controller velocity to slow speed.")
        else:
            self.vpp_update_vel.update_configuration(self.VPP_normal_speed_)
            rospy.loginfo("SpeedFilter: Set controller velocity to normal speed.")

    def binary_state_callback(self, msg: Bool):
        self.setVelocity(msg.data)
        

if __name__== '__main__':
    rospy.init_node("speed_limited_zone")
    try:
        speed_limited_zone = SpeedFilter()
        rospy.loginfo("SpeedFilter node is running!")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
        