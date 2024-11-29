#!/usr/bin/env python3

import rospy
import tf2_ros
import yaml

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16, Bool
from std_srvs.srv import Trigger

fixed_position_file = "/home/amr/catkin_ws/src/amr_vdm/amr_waypoint_generator/config/fixed_position.yaml"


class PoseEstimation():

    def __init__(self):
        # Listen to Transfromation
        self.__tfBuffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.__tf_listener = tf2_ros.TransformListener(self.__tfBuffer)

        self.frame_id = "map"
        self.base_frame_id = "base_footprint"

        self.pose = Pose()

        # Publishers:
        self.pub_initial_pose    = rospy.Publisher("/initialpose",
                                                   PoseWithCovarianceStamped, queue_size=5)
        self.pub_is_initial_pose = rospy.Publisher("is_intialpose", Bool, queue_size=5)

        # Subscribers:
        rospy.Subscriber("measured_joint_states", JointState, self.joint_state_callback)
        rospy.Subscriber("pose_estimation", Int16, self.pose_estimation_callback)
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.initial_pose_callback)

        # Variables:
        self.state_flag = False
        self.timer = 0
        self.set_position = 0
        self.flag = False

        # Fixed positions:
        self.fixed_position = self.get_pose_from_yaml("tp2_fixed_positipon")

    
    def get_2d_pose(self):
        """
        Take 2D Pose
        """
        try: 
            trans = self.__tfBuffer.lookup_transform(
                self.frame_id,
                self.base_frame_id,
                rospy.Time(0), timeout=rospy.Duration(0.1))
            
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            rz = trans.transform.rotation.z
            rw = trans.transform.rotation.w

            return x, y, rz, rw
        
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logwarn(f"PoseEstimation: Failed lookup: {self.base_frame_id}, from {self.frame_id}")
            return None
    
    def get_pose_from_yaml(self, position_name: str):
        """
        Get position from yaml file
        """
        assert (type(position_name) == str), "position_name is not str type"

        with open(fixed_position_file, 'r') as file:
            data = yaml.safe_load(file)

            position = data[f'{position_name}']['pose']
            float_position_list = [[float(value) if isinstance(value, (int, float, str)) \
                                  else value for value in sublist] for sublist in position]
                
            return float_position_list

    
    def initial_pose_callback(self, msg: PoseWithCovarianceStamped):
        pose = msg.pose.pose

        self.pose.position.x = pose.position.x
        self.pose.position.y = pose.position.y
        self.pose.orientation.z = pose.orientation.z
        self.pose.orientation.w = pose.orientation.w

    
    def set_by_hand(self, pose):
        self.pose.position.x = pose[0]
        self.pose.position.y = pose[1]
        self.pose.orientation.z = pose[2]
        self.pose.orientation.w = pose[3]

        self.set_pose_estimation(self.pose) 


    def pose_estimation_callback(self, msg: Int16):
        if msg.data == 1:
            self.pub_is_initial_pose.publish(True)
            return
        try:
            self.set_by_hand(self.fixed_position[msg.data-2])
            rospy.loginfo(f"PoseEstimation: Set position {msg.data} successfully!")
        except:
            rospy.logerr("Position is empty, try again...!")
            self.pub_is_initial_pose.publish(True)
            return

        self.pub_is_initial_pose.publish(True)
        rospy.sleep(0.5)
        self.flag = True
    

    def set_pose_estimation(self, pose: Pose):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = pose
        msg.header.frame_id = self.frame_id

        self.pub_initial_pose.publish(msg)
        rospy.loginfo("PoseEstimation: Published position from initialpose topic!")
    

    def joint_state_callback(self, msg: JointState):
        v_left  = msg.velocity[0]
        v_right = msg.velocity[1]

        if (abs(v_left) <= 0.01 and abs(v_right) <= 0.01):
            if self.flag:
                if not self.state_flag:
                    pose = self.get_2d_pose()

                    self.pose.position.x = pose[0]
                    self.pose.position.y = pose[1]
                    self.pose.orientation.z = pose[2]
                    self.pose.orientation.w = pose[3]

                    rospy.logdebug("PoseEstimation: Take the robot's current position")
                    self.set_pose_estimation(self.pose)
                    self.state_flag = True
                
                if self.timer >= 100:
                    self.set_pose_estimation(self.pose)
                    self.timer = 0
                
                else:
                    self.timer += 1
        else:
            self.state_flag = False
            self.timer = 0
    

if __name__ == "__main__":
    rospy.init_node("pose_estimation")
    try:
        pose_estimation = PoseEstimation()
        rospy.loginfo("PoseEstimation node is running!")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
