#!/usr/bin/env python3
import rospy
import time
import math
import yaml
import numpy as np
import actionlib
import tf2_ros
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from amr_autodocking.msg import AutoDockingAction, AutoDockingGoal
from nav_msgs.srv import LoadMap, LoadMapResponse
from std_srvs.srv import Empty, SetBool
from std_msgs.msg import Bool, Int16, Float64, Int16MultiArray
from geometry_msgs.msg import Twist
from tf import transformations as ts
from nav_msgs.msg import Odometry
from typing import Tuple

Pose2D = Tuple[float, float, float]

INFO  = rospy.loginfo
WARN  = rospy.logwarn
ERROR = rospy.logerr

amr_waypoint_file = "/home/amr/catkin_ws/src/amr_vdm/amr_waypoint_generator/config/amr_waypoints.yaml"
map_file = "/home/amr/catkin_ws/src/amr_vdm/amr_gazebo/maps/custom_map_5/custom_map_5.yaml"

class PID:
    def __init__(self, kp, ki, kd, out_min: float = 0.0, out_max: float = 0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.out_min = out_min
        self.out_max = out_max

    def update(self, setpoint, feedback_value, dt):
        dt_ms = dt*1000
        error = setpoint - feedback_value
        self.integral += error * dt_ms
        derivative = (error - self.prev_error) / dt_ms
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        if (self.out_min != 0 or self.out_max != 0):
            return self.clamp(abs(output), self.out_min, self.out_max)
        return output

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint

    def set_tunings(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def clamp(self, value, lower_limit, upper_limit):
        if value > upper_limit:
            return upper_limit
        elif value < lower_limit:
            return lower_limit
        return value
    

class Parameter():
    map_frame_id = "map"
    odom_frame_id = "odom"
    robot_base_frame_id = "base_footprint"
    distance_tolerance = 3.0
    frequency = 100
    tf_expiry = 1.0
    max_x_move = 0.2
    max_w_rot = 0.2 
    min_x_move = 0.15
    min_w_rot = 0.15
    stop_yaw_diff = 0.05
    stop_trans_diff = 0.01              # meters
    max_angular_vel = 0.3              # rad/s
    min_angular_vel = 0.05
    max_linear_vel  = 0.3
    min_linear_vel  = 0.025 
    max_x_out_dock = 0.2
    map_file = ""
    yaw_threshold = 0.35 # rad ~ 20 degrees
    waypoints_to_follow_topic = "/initialpose"
    waypoints_list_topic = "/waypoints"

    k_p = 0.8
    k_i = 0.0
    k_d = 0.01

class AutoNavigation():

    def __init__(self):
        rospy.init_node('auto_navigation')

        self.params = Parameter()
        self.init_params()

        # Listen to Transfromation
        self.__tfBuffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.__tf_listener = tf2_ros.TransformListener(self.__tfBuffer)

        # Params
        self.sleep_period = rospy.Duration(1/20.0)
        self.rate = rospy.Rate(self.params.frequency)

        # list of waypoints to follow
        self.waypoints = []

        # move_base Action Client
        self.move_base_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        INFO("AutoNavigation: Connecting to move base...")
        self.move_base_client.wait_for_server()
        INFO("AutoNavigation: Connected to move base.")

        # Auto docking Client
        self.autodock_client = actionlib.SimpleActionClient("autodock_action", AutoDockingAction)
        INFO("AutoNavigation: Connecting to auto docking...")
        self.autodock_client.wait_for_server()
        INFO("AutoNavigation: Connected to auto docking.")

        # Change map service
        self.change_map_client = rospy.ServiceProxy("/change_map", LoadMap)
        INFO("AutoNavigation: Connecting to change map service...")
        self.change_map_client.wait_for_service()
        INFO("AutoNavigation: Connected to change map service.")

        # Clear costmap service
        self.clear_costmap = rospy.ServiceProxy("/move_base_node/clear_costmaps", Empty)
        INFO("AutoNavigation: Connecting to /move_base_node/clear_costmaps service...")
        self.clear_costmap.wait_for_service()
        INFO("AutoNavigation: Connected to /move_base_node/clear_costmaps service.")

        # Apriltag continuous detection service
        self.apriltag_client = rospy.ServiceProxy("/back_camera/apriltag_ros/enable_detector", SetBool)
        INFO("AutoNavigation: Connecting to /back_camera/apriltag_ros/enable_detector service...")
        self.apriltag_client.wait_for_service()
        INFO("AutoNavigation: Connected to /back_camera/apriltag_ros/enable_detector service.")
        
        # Variables:
        self.start_amr = 0
        self.detect_obstacle_ = False
        self.is_stopped_ = False
        self.is_pausing_  = False
        self.is_running_ = False
        self.dock_name = 0
        self.error = 2   #[0] - Loi cap hang, [1] - Loi lay hang, [2] - Loi vi tri
        self.prev_state = 0

        # Waypoints:
        self.line_55_clr_goal = self.get_pose_from_yaml("line_55_clr_goal")
        self.line_56_clr_goal = self.get_pose_from_yaml("line_56_clr_goal")
        self.line_55_pickup_goal = self.get_pose_from_yaml("line_55_pickup_goal")
        
        self.waypoint_clr_to_line_55 = self.get_pose_from_yaml("waypoint_clr_to_line_55")
        self.waypoint_line_55_back_to_clr = self.get_pose_from_yaml("waypoint_line_55_back_to_clr")

        self.waypoint_clr_to_line_56 = self.get_pose_from_yaml("waypoint_clr_to_line_56")
        self.waypoint_line_56_back_to_clr = self.get_pose_from_yaml("waypoint_line_56_back_to_clr")

        self.charger_goal_ = self.get_pose_from_yaml("charger_goal")
        self.park_goal_ = self.get_pose_from_yaml("amr_park_tp2")

        # Publishers:
        self.pub_pose_array  = rospy.Publisher(self.params.waypoints_list_topic, PoseArray, queue_size=1, latch=True)
        self.pub_goal_name   = rospy.Publisher("goal_name", Int16, queue_size=5)
        self.pub_dock_name   = rospy.Publisher("dock_name", Int16,queue_size=5)
        self.pub_cmd_brake   = rospy.Publisher("cmd_brake", Bool, queue_size=5)
        self.pub_cmd_vel     = rospy.Publisher("/amr/mobile_base_controller/cmd_vel", Twist, queue_size=5)
        self.pub_runonce     = rospy.Publisher("state_runonce_nav", Bool, queue_size=5)
        self._pub_mode_error = rospy.Publisher("error_mode", Int16, queue_size=5)
        self.pub_turn_off_front_safety = rospy.Publisher("turn_off_front_safety", Bool,queue_size=5)
        self.pub_turn_off_ultrasonic_safety = rospy.Publisher("turn_off_ultrasonic_safety", Bool,queue_size=5)
        self.run_controller_pub_ = rospy.Publisher("/run_rs_controller", Bool, queue_size=5)

        # Subscribers:
        rospy.Subscriber("START_AMR", Int16MultiArray, self.run)
        rospy.Subscriber("PAUSE_AMR", Bool, self.pause_callback)
        rospy.Subscriber("CANCEL_AMR", Bool, self.cancel_callback)
        rospy.Subscriber("emergency_stop", Bool, self.emergency_stop_callback)
        rospy.Subscriber("status_protected_field", Bool, self.protected_filed_callback)


    def init_params(self):
        rospy.loginfo("NODE: AutoNavigation")

        param_names = [attr for attr in dir(self.params) if not callable(getattr(self.params, attr)) and not attr.startswith("__")]

        # get private rosparam, if none use default
        for param_name in param_names:
            param_val = rospy.get_param("~" + param_name, getattr(self.params, param_name))
            print(f"AutoNavigation: set param [{param_name}] to [{param_val}]")
            setattr(self.params, param_name, param_val)
    
    def cancel_callback(self, msg: Bool):
        self.is_stopped_ = msg.data

    def pause_callback(self, msg:Bool):
        self.is_pausing_ = msg.data
    
    def emergency_stop_callback(self, msg:Bool):
        self.is_stopped_ = msg.data

    def protected_filed_callback(self, msg:Bool):
        self.detect_obstacle_ = msg.data
    
    def turn_off_brake(self, signal):
        self.pub_cmd_brake.publish(signal)
    
    def publish_mode_error(self, data):
        mode_error = Int16()
        mode_error.data = data
        self._pub_mode_error.publish(mode_error)
    
    def runonce(self, signal):
        self.pub_runonce.publish(signal)
    
    def reset(self):
        self.error = 2
        self.is_stopped_ = False
        self.is_running_ = False
        self.runonce(False)
        self.turn_off_brake(True)
    
    def get_pose_from_yaml(self, position_name: str):
        """
        Get position from yaml file
        """
        assert (type(position_name) == str), "position_name is not str type"

        with open(amr_waypoint_file, 'r') as file:
            data = yaml.safe_load(file)
            try:
                position = data[f'{position_name}']['pose']
                float_position_list = [[float(value) if isinstance(value, (int, float, str)) \
                                    else value for value in sublist] for sublist in position] 
                return float_position_list   
            except:
                return position    

    def get_mat_from_odom_msg(self, msg: Odometry) -> np.ndarray:
        """
        This will return a homogenous transformation of odom pose msg
        :param :    input odom msg
        :return :   homogenous transformation matrix
        """
        _rot = msg.pose.pose.orientation
        _q = (_rot.x, _rot.y, _rot.z, _rot.w)
        _trans = msg.pose.pose.position
        _tr = (_trans.x, _trans.y, _trans.z)
        _tf_mat = ts.concatenate_matrices(
            ts.translation_matrix(_tr), ts.quaternion_matrix(_q))
        return _tf_mat
    
    def sat_proportional_filter(self, input: float, abs_min=0.0, abs_max=10.0, factor=1.0) -> float:
        """
        Simple saturated proportional filter
        :param input                : input value
        :param abs_min and abs_max  : upper and lower bound, abs value
        :param factor               : multiplier factor for the input value
        :return                     : output filtered value, within boundary
        """
        output = 0.0
        input *= factor
        if abs(input) < abs_min:
            if (input < 0):
                output = -abs_min
            else:
                output = abs_min
        elif abs(input) > abs_max:
            if (input > 0):
                output = abs_max
            else:
                output = -abs_max
        else:
            output = input
        return output
    
    def bin_filter(self, input: float, abs_boundary: float) -> float:
        """
        Simple binary filter, will provide abs_ceiling as a binary output,
        according to the 'negativity' of the input value
        :param input        : input value
        :param abs_boundary : abs boundary value
        :return             : output binary value
        """
        output = abs(abs_boundary)
        if input < 0:
            output = -abs(abs_boundary)
        return output

    def get_odom(self) -> np.ndarray:
        """
        Get the current odom of the robot
        :return : 4x4 homogenous matrix, None if not avail
        """
        try:
            return self.get_mat_from_odom_msg(
                rospy.wait_for_message(
                    "/amr/odometry/filtered", Odometry, timeout=1.0)
            )
        except rospy.exceptions.ROSException:
            rospy.logerr(f"AutoNavigation: Failed to get odom")
            return None
        
    
    def apply_2d_transform(self, mat: np.ndarray, transform: Pose2D) -> np.ndarray:
        """
        Apply a 2d transform to a homogenous matrix
        :param mat:         the input 4x4 homogenous matrix
        :param transform :  2d transform which to apply to the mat
        :return :           transformed homogenous transformation matrix
        """
        # req target transformation from base
        q = quaternion_from_euler(0, 0, transform[2])
        tf_mat = ts.concatenate_matrices(
            ts.translation_matrix(
                (transform[0], transform[1], 0)), ts.quaternion_matrix(q))
        return np.matmul(mat, tf_mat)
    

    def compute_tf_diff(self, current_tf: np.ndarray, ref_tf: np.ndarray) -> Pose2D:
        """
        Find the diff of two transformation matrix
        :param :  homogenous transformation of 2 matrices
        :return :  the 2d planer trans fo the 2 inputs; [x, y, yaw]
        """
        tf_diff = np.matmul(ts.inverse_matrix(current_tf), ref_tf)
        trans = ts.translation_from_matrix(tf_diff)
        euler = ts.euler_from_matrix(tf_diff)
        return trans[0], trans[1], euler[2]
    

    def publish_velocity(self, linear_vel=0.0, angular_vel=0.0):
        """
        Command the robot to move, default param is STOP!
        """
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self.pub_cmd_vel.publish(msg)
    

    def rotate_with_odom(self, min_angular_vel, max_angular_vel, rotate: float) -> bool:
        """
        Spot Rotate the robot with odom. Blocking function
        :return : success
        """
        rospy.loginfo(f"AutoNavigation: Turn robot: {rotate:.2f} rad")

        _initial_tf = self.get_odom()
        if _initial_tf is None:
            return False

        # get the tf mat from odom to goal frame
        _goal_tf = self.apply_2d_transform(_initial_tf, (0, 0, rotate))
        _pid = PID(self.params.k_p, self.params.k_i, self.params.k_d, min_angular_vel, max_angular_vel)
        
        prev_time = rospy.Time.now()
        while not rospy.is_shutdown():

            if self.is_stopped_:
                return False
            
            if self.is_pausing_ or self.detect_obstacle_:
                self.rate.sleep()
                continue
            
            else:
                _curr_tf = self.get_odom()
                if _curr_tf is None:
                    return False

                dx, dy, dyaw = self.compute_tf_diff(_curr_tf, _goal_tf)

                if abs(dyaw) < self.params.stop_yaw_diff:
                    self.publish_velocity()
                    rospy.loginfo("AutoNavigation: Done with rotate robot")
                    return True
                
                time_now = rospy.Time.now()
                dt = (time_now - prev_time).to_sec()
                angular_vel_pid = _pid.update(rotate, rotate - dyaw, dt)             
                sign = 1 if rotate > 0 else -1
                angular_vel = sign*angular_vel_pid
                
                self.publish_velocity(angular_vel=angular_vel)
                prev_time = time_now
            rospy.sleep(self.sleep_period)
        exit(0)
    
    def move_with_odom(self, min_speed: float, max_speed: float, forward:float) -> bool:
        """
        Move robot in linear motion with Odom. Blocking function
        :return : success
        """
        rospy.loginfo(f"AutoNavigation: Move robot: {forward:.2f} m")

        _initial_tf = self.get_odom()
        if _initial_tf is None:
            return False

        # get the tf mat from odom to goal frame
        _goal_tf = self.apply_2d_transform(_initial_tf, (forward, 0, 0))
        _pid = PID(self.params.k_p, self.params.k_i, self.params.k_d, min_speed, max_speed)
        # second mat
        prev_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.is_stopped_:
                return False
            
            if self.is_pausing_ or self.detect_obstacle_:
                self.rate.sleep()
                continue

            else:
                _curr_tf = self.get_odom()
                if _curr_tf is None:
                    return False

                dx, dy, dyaw = self.compute_tf_diff(_curr_tf, _goal_tf)

                if abs(dx) < self.params.stop_trans_diff:
                    self.publish_velocity()
                    rospy.loginfo("AutoNavigation: Done with move robot")
                    return True

                # This makes sure that the robot is actually moving linearly
                time_now = rospy.Time.now()
                dt = (time_now - prev_time).to_sec()
                l_vel_pid = _pid.update(forward, forward - dx, dt)
                ang_vel = self.sat_proportional_filter(dyaw, abs_max=self.params.min_angular_vel, factor=0.2)
                l_vel   = self.bin_filter(dx, l_vel_pid)

                self.publish_velocity(linear_vel=l_vel, angular_vel=ang_vel)
                prev_time = time_now
            rospy.sleep(self.sleep_period)
        exit(0)

    def get_2D_pose(self):
        """
        Take 2D Pose
        """
        try:
            trans = self.__tfBuffer.lookup_transform(
                self.params.map_frame_id,
                self.params.robot_base_frame_id,
                rospy.Time.now(), timeout=rospy.Duration(1.0))
            
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z
        
            rx = trans.transform.rotation.x
            ry = trans.transform.rotation.y
            rz = trans.transform.rotation.z
            rw = trans.transform.rotation.w

            orientation = euler_from_quaternion([rx, ry, rz, rw])

            return x, y, orientation[2], rw

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logerr(f"AutoNavigation: Failed lookup: {self.params.robot_base_frame_id}, from {self.params.map_frame_id}")
            return None
        
    
    def pub_waypoint_list(self):
        """Helper method to publish the waypoints that should be followed."""
        try:
            self.pub_pose_array.publish(self.to_pose_array(self.waypoints))
            return True
        except:
            return False
        
    
    # helper methods
    def to_pose_array(self, waypoints):
        """Publish waypoints as a pose array so that you can see them in rviz."""
        poses = PoseArray()
        poses.header.frame_id = self.params.map_frame_id
        poses.poses = [pose for pose in waypoints]
        return poses
    

    def send_move_base_goal(self, pose: Pose):
        """Assemble and send a new goal to move_base"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.params.map_frame_id
        goal.target_pose.pose.position = pose.position
        goal.target_pose.pose.orientation = pose.orientation

        self.move_base_client.send_goal(goal)

    
    def navigate_to_goal(self, pose):
        """
        Navigating robot to a goal on map
        `footprint`: `1` - default footprint (small) | `0` - big footprint
        """
        self.run_controller_pub_.publish(True)

        poses = Pose()
        poses.position.x = float(pose[0])
        poses.position.y = float(pose[1])
        poses.position.z = 0.0
        poses.orientation.x = 0.0
        poses.orientation.y = 0.0
        poses.orientation.z = float(pose[2])
        poses.orientation.w = float(pose[3])

        INFO(f"AutoNavigation: Starting go to goal({pose[0]:.4f}, {pose[1]:.4f})")

        self.send_move_base_goal(poses)

        self.move_base_client.wait_for_result()

        if self.move_base_client.get_state() == GoalStatus.SUCCEEDED:
            INFO("AutoNavigation: Goal is SUCCESS!")
            return True 
        else:
            ERROR("AutoNavigation: Goal is FAILED!")
            self.error = 2
            return False


    def navigate_through_goal(self, poses):
        """
        Navigating robot through goal list on map
        """
        self.run_controller_pub_.publish(True)

        self.waypoints.clear()

        pose_arr = np.array(poses)

        amr_tf = self.get_2D_pose()
        x, y, rz, rw = amr_tf

        amr_pose = np.array([x, y, rz, rw])

        # Calculate distance between two arrays
        distances = np.linalg.norm(pose_arr - amr_pose, axis=1)

        # Find closest array in pose_arr respective to amr_pose
        closest_index = np.argmin(distances)
        try:
            closest_poses = pose_arr[(closest_index+1):]
        except:
            closest_poses = pose_arr[[poses[-1]]]
     
        for i in closest_poses:
            pose = Pose()
            pose.position.x = float(i[0])
            pose.position.y = float(i[1])
            pose.position.z = 0.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = float(i[2])
            pose.orientation.w = float(i[3])
            
            self.waypoints.append(pose)
        
        INFO(f"AutoNavigation: Start moving robot through {len(closest_poses)} waypoints!")

        while not rospy.is_shutdown():

            if not self.waypoints:
                ERROR("AutoNavigation: No more waypoints to follow!")
                return False
            
            goal = self.waypoints[0]

            self.send_move_base_goal(goal)

            if len(self.waypoints) == 1:
                self.move_base_client.wait_for_result()

                if self.move_base_client.get_state() == GoalStatus.SUCCEEDED:
                    INFO("AutoNavigation: Goal is SUCCESS!")
                    return True
                else:
                    self.error = 2
                    return False 
            else:
                distance = 10
                while distance > self.params.distance_tolerance:

                    curr_pose = self.get_2D_pose()
                    x, y, _, _ = curr_pose

                    distance = math.sqrt(pow(goal.position.x - x, 2) + pow(goal.position.y - y, 2))

                    if (self.move_base_client.get_state() == GoalStatus.ABORTED
                        or self.move_base_client.get_state() == GoalStatus.PREEMPTED):
                        self.error = 2
                        return False
                    
                    self.rate.sleep()
                
                self.waypoints.pop(0)

            self.pub_waypoint_list()
            self.rate.sleep()

    
    def change_map(self, map_file):
        """
        Change map service
        """
        try:
            INFO("AutoNavigation: Waiting result from change_map server...")
            resp = self.change_map_client(map_file)
            
            status = resp.result

            if status != LoadMapResponse.RESULT_SUCCESS:
                ERROR('AutoNavigation: Change map request failed!')
                return False
            else:
                INFO('AutoNavigation: Change map request was successful!')
                time.sleep(1.0)
                return True
        
        except rospy.ServiceException as e:
            ERROR("AutoNavigation: Service call failed: %s"%e)

    
    def clear_all_costmap(self):
        """
        Clear all costmaps
        """
        try:
            INFO("AutoNavigation: Requesting clear all costmaps...")
            self.clear_costmap.call()
        
        except rospy.ServiceException as e:
            ERROR("AutoNavigation: Service call failed: %s"%e)

    
    def enable_apriltag_detector(self, signal):
        """
        Dis or enable apriltag continuous detection
        `signal = False`: Disable | `signal = True`: Enable
        """
        msg = "disable" if not signal else "enable"

        try:
            INFO(f"AutoNavigation: Waiting {msg} apriltag detection from server...")
            result = self.apriltag_client.call(signal)

            if result.success:
                INFO("AutoNavigation: " + result.message)
                return True
            else:
                return False
        
        except rospy.ServiceException as e:
            ERROR("AutoNavigation: Service call failed: %s"%e)

    
    def send_auto_docking(self, dock_name:int=None, enable_Apriltag_detector:bool=None):
        """
        Auto docking action
        `dock_name`: L55 - `1`, `3`, `5` or `7` | L56 - `2`, `4`, `6` or `8`
        `enable_Apriltag_detector`: `True` - Enable | `False` - Disable (Default)
        """
        if dock_name is not None:
            self.pub_dock_name.publish(dock_name)
        
        if enable_Apriltag_detector is not None:
            self.enable_apriltag_detector(enable_Apriltag_detector)

        goal = AutoDockingGoal()
        goal.start_autodock = True

        self.autodock_client.send_goal(goal)
        self.autodock_client.wait_for_result()

        if self.autodock_client.get_state() == GoalStatus.SUCCEEDED:
            INFO("AutoNavigation: Auto docking is complete!")
            self.enable_apriltag_detector(False)
            return True
        else:
            ERROR("AutoNavigation: Auto docking is failed!")
            self.enable_apriltag_detector(False)
            return False

    def navigate_to_charger(self):
        WARN("AutoNavigation: Battery is low, returning the charger...")

        if not self.navigate_through_goal(self.get_pose_from_yaml("charging_waypoints")):
            return False
        
        if not self.send_auto_docking():
            return False
        
        return True


    #===============> LINE 55 <================#
    def pickup_at_cleaning_room_line_55(self):
        """
        AMR picks up the line 55 motor order in the cleaning room
        """
        if not self.navigate_to_goal(self.line_55_clr_goal):
            return False
        if not self.send_auto_docking(1, True):
            self.error = 1
            return False
        return True
    
    def deliver_to_line_55(self):
        """
        AMR transports of motor to line 55 after cleaned
        """
        if not self.navigate_through_goal(self.waypoint_clr_to_line_55):
            return False
        if not self.send_auto_docking(3):
            self.error = 0
            return False
        return True

    def pickup_at_line_55(self):
        """
        AMR picks up the unclean motor at line 55
        """
        if not self.navigate_to_goal(self.line_55_pickup_goal):
            return False
        if not self.send_auto_docking(5, True):
            self.error = 1
            return False
        return True

    def deliver_to_cleaning_room_line_55(self):
        """
        AMR transports the unclean motor back to the cleaning room
        """
        if not self.navigate_through_goal(self.waypoint_line_55_back_to_clr):
            return False
        if not self.rotate_with_odom(self.params.min_angular_vel, self.params.max_angular_vel, 182*math.pi/180):
            return False
        if not self.send_auto_docking(7):
            self.error = 0
            return False
        return True
    

    #===============> LINE 56 <================#
    def pickUpLine56ClrOrder(self):
        """
        AMR picks up the line 56 motor order in the cleaning room
        """
        if not self.navigate_to_goal(self.line_56_clr_goal):
            return False
        if not self.send_auto_docking(2, True):
            self.error = 1
            return False
        return True

    def deliverToLine56(self):
        """
        AMR transports of motor to line 56 after cleaned
        """
        if not self.navigate_through_goal(self.waypoint_clr_to_line_56):
            return False
        if not self.rotate_with_odom(self.params.min_angular_vel, self.params.max_angular_vel, -90*math.pi/180):
            return False
        if not self.send_auto_docking(4):
            self.error = 0
            return False
        return True

    def pickUpOrderLine56(self):
        """
        AMR picks up the unclean motor at line 56
        """
        if not self.rotate_with_odom(self.params.min_angular_vel, self.params.max_angular_vel, 90*math.pi/180):
            return False
        if not self.move_with_odom(self.params.min_linear_vel, self.params.max_linear_vel, 1.5):
            return False
        if not self.rotate_with_odom(self.params.min_angular_vel, self.params.max_angular_vel, -90*math.pi/180):
            return False
        if not self.send_auto_docking(6, True):
            self.error = 1
            return False
        if not self.rotate_with_odom(self.params.min_angular_vel, self.params.max_angular_vel, -60*math.pi/180):
            return False
        if not self.move_with_odom((self.params.min_linear_vel, self.params.max_linear_vel - 0.15), -0.3):
            return False
        if not self.rotate_with_odom(self.params.min_angular_vel, self.params.max_angular_vel, -30*math.pi/180):
            return False
        return True

    def deliverToClrLine56(self):
        """
        AMR transports the unclean motor back to the cleaning room
        """
        if not self.navigate_through_goal(self.waypoint_line_56_back_to_clr):
            return False
        if not self.rotate_with_odom(self.params.min_angular_vel, self.params.max_angular_vel, 92*math.pi/180):
            return False
        if not self.send_auto_docking(8):
            self.error = 0
            return False
        return True
    
    
    #===============> Charger <================#
    def navigate_to_charger(self):
        if not self.navigate_to_goal(self.charger_goal_):
            self.error = 2
            return False
        if not self.send_auto_docking(57):
            self.error = 2
            return False
        return True


    #===============> MAIN RUN <===============#
    def line_55_navigation(self, signal):
        if signal == 1:
            if (
                self.pickup_at_cleaning_room_line_55() and
                self.deliver_to_line_55() and
                self.pickup_at_line_55() and
                self.deliver_to_cleaning_room_line_55()
            ):  return True

        elif signal == 2:
            if self.deliver_to_cleaning_room_line_55(): return True   

        else:
            if (
                self.deliver_to_line_55() and
                self.pickup_at_line_55() and
                self.deliver_to_cleaning_room_line_55()
            ):  return True
        
        return False

    
    def run(self, signal: Int16MultiArray):
        self.start_amr = signal.data
        self.is_running_ = True
        self.runonce(True)
        self.turn_off_brake(False)

        if self.prev_state == 57:
            if (not self.move_with_odom(self.params.min_linear_vel, self.params.max_linear_vel, -1.0)
                and not self.rotate_with_odom(self.params.min_angular_vel, self.params.max_angular_vel, 90*math.pi/180)):
                ERROR("AutoNavigation: Navigation is failed!")
                self.error = 2
                self.publish_mode_error(self.error)
                self.reset()
                return False
            self.pub_turn_off_front_safety.publish(False)
            self.pub_turn_off_ultrasonic_safety.publish(False)

        if self.start_amr[0] == 55:
            self.prev_state = 55
            if not self.line_55_navigation(self.start_amr[1]):
                ERROR("AutoNavigation: Navigation is failed!")
                self.publish_mode_error(self.error)
                self.reset()
                return False
        
        elif self.start_amr[0] == 57:
            self.prev_state = 57
            if not self.navigate_to_charger():
                ERROR("AutoNavigation: Navigation is failed!")
                self.publish_mode_error(self.error)
                self.reset()
                return False
            
        INFO("AutoNavigation: Navigation is complete!")
        self.reset()
        return True    
        

if __name__== '__main__':
    try:
        autonav = AutoNavigation()
        INFO("AutoNavigation is running!")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
