#!/usr/bin/env python3

import rospy
import actionlib
import dynamic_reconfigure.client
from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseAction
from amr_autodocking.msg import AutoDockingAction


class Controller():
    
    def __init__(self):

        # Variables:
        self.timeout_pause = 0.0
        self.timeout_normal = 15.0
        self.loop_freq_ = 5.0
        self.is_running_ = False
        self.is_canceled_ = False
        self.is_stopped_ = False
        self.prev_state_ = False

        # Client:
        self.move_base_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.move_base_client.wait_for_server()
        self.auto_dock_client = actionlib.SimpleActionClient("autodock_action", AutoDockingAction)
        self.auto_dock_client.wait_for_server()
        
        # Subscribers:
        rospy.Subscriber("CANCEL_AMR", Bool, self.cancel_callback)
        rospy.Subscriber("PAUSE_AMR", Bool, self.pause_callback)
        rospy.Subscriber("state_runonce_nav", Bool, self.runonce_callback)
        rospy.Subscriber("emergency_stop", Bool, self.emergency_stop_callback)

    
    def emergency_stop_callback(self, msg:Bool):
        if self.is_running_:
            self.is_stopped_ = msg.data
    
    def cancel_callback(self, msg: Bool):
        if self.is_running_:
            self.is_canceled_ = msg.data     

    def configure_controller(self,timeout_mvb):
        client_movebase_timeout = dynamic_reconfigure.client.Client("/move_base_node")
        client_movebase_timeout.update_configuration({'oscillation_timeout': timeout_mvb})

    def runonce_callback(self,msg: Bool):
        self.is_running_ = msg.data

    def pause_callback(self, is_pause: Bool):
        if not self.is_running_:
            return
        if is_pause.data:
            self.configure_controller(self.timeout_pause)
            rospy.loginfo("Controller: Infinite waiting for pausing.")
        else:
            self.configure_controller(self.timeout_normal)
            rospy.loginfo(f"Controller: Waiting for timeout {self.timeout_normal}s.")

    def run(self):
        while not rospy.is_shutdown():
            if self.is_running_:
                if self.is_canceled_ or self.is_stopped_:
                    if not self.prev_state_:
                        self.move_base_client.cancel_all_goals()
                        self.auto_dock_client.cancel_all_goals()
                        rospy.loginfo("Controller: Canceled all action clients.")
                        self.prev_state_ = True
                elif not self.is_canceled_ or not self.is_stopped_:
                    self.prev_state_ = False
            else:
                self.prev_state_ = False
            
            rospy.sleep(1/self.loop_freq_)


if __name__ == "__main__":
    rospy.init_node("controller")
    try:
        controller = Controller()
        rospy.loginfo("Controller node is running!")
        controller.run()
    except rospy.ROSInterruptException:
        pass