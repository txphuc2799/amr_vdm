#!/usr/bin/env python3
import rospy
import dynamic_reconfigure.client as dc

from std_msgs.msg import Float32, Bool
from speed_filter_msgs.msg import SpeedLimit


class SpeedFilter():

    def __init__(self):

        self.max_velocity_ = rospy.get_param("~max_velocity", 0.7)

        # Setup loop frequency
        self.loop_freq_ = 5.0

        self.update_velocity_ = dc.Client("/move_base_node/RotationShimController/VectorPursuitController")

        self.is_running_ = False
        self.current_speed_limit_ = 1.0
        self.prev_current_speed_limit_ = 1.0
        self.speed_at_field_ = 1.0
        self.speed_limit_ = 1.0
    
        # Subscribers:
        rospy.Subscriber("/speed_limit", SpeedLimit, self.speed_limit_callback)
        rospy.Subscriber("speed_at_field", Float32, self.speed_at_field_callback)
        rospy.Subscriber("state_runonce_nav", Bool, self.runonce_callback)
    
    def runonce_callback(self,msg: Bool):
        self.is_running_ = msg.data
    
    def speed_at_field_callback(self, msg:Float32):
        self.speed_at_field_ = msg.data

    def update_velocity(self, speed_limit_per):
        self.update_velocity_.update_configuration({'desired_linear_vel': self.max_velocity_*speed_limit_per})
        rospy.loginfo((f"Updated velocity to {round(self.max_velocity_ * speed_limit_per, 2)}m/s "
                       f"from max velocity is {self.max_velocity_}m/s."))

    def speed_limit_callback(self, msg:SpeedLimit):
        self.speed_limit_ = msg.speed_limit

    def run(self):
        while not rospy.is_shutdown():
            if self.is_running_:
                if (self.speed_at_field_ <= 0.95 or self.speed_limit_ <= 0.95):    
                    if self.speed_at_field_ <= self.speed_limit_:
                        speed_limit_per = self.speed_at_field_
                    else:
                        speed_limit_per = self.speed_limit_
                else:
                    speed_limit_per = 1.0
                
                self.current_speed_limit_ = speed_limit_per

                if self.current_speed_limit_ != self.prev_current_speed_limit_:
                    self.update_velocity(speed_limit_per)
                
                self.prev_current_speed_limit_ = self.current_speed_limit_
            
            rospy.sleep(1/self.loop_freq_)
        

if __name__== '__main__':
    rospy.init_node("speed_filter")
    try:
        speed_filter = SpeedFilter()
        rospy.loginfo("SpeedFilter node is running!")
        speed_filter.run()
        
    except rospy.ROSInterruptException:
        pass
        