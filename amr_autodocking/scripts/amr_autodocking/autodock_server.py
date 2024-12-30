#!/usr/bin/env python3

# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import tf2_ros
import math
import numpy as np
import actionlib
import amr_autodocking.autodock_utils as utils
import dynamic_reconfigure.client as client

from amr_autodocking.autodock_utils import DockState
from amr_autodocking.msg import AutoDockingFeedback
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int16
from amr_autodocking.msg import AutoDockingAction
from amr_autodocking.msg import AutoDockingGoal, AutoDockingResult
from amr_msgs.msg import SliderSensorStamped
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
from amr_autodocking.pid import PID

BOTH = AutoDockingGoal().BOTH
ONLY_LEFT = AutoDockingGoal().ONLY_LEFT
ONLY_RIGHT = AutoDockingGoal().ONLY_RIGHT

class AutodockConfig:
    # [General configure]
    tf_expiry          = 1.0            # sec
    dock_timeout       = 300            # sec
    controller_frequency = 20.0           # hz
    gear_ratio         = 10.33
    encoder_resolution = 82640.0
    wheel_radius       = 0.0625
    slider_timeout     = 15             # sec
    back_laser_offset = 0.52322         # Distance from base_link to back_laser_link when slider at original position
    front_laser_offset = 0.14802 + 0.1025/2
    steer_distance_threshold = 0.97

    # [Frames]
    base_link = "base_link"
    charger_frame = "charger_frame"
    first_frame = "first_frame"
    last_frame = "last_frame"
    parallel_frame = "parallel_frame"
    custom_dock_name = {}
   
    # [Goal threshold]
    stop_yaw_diff   = 0.05              # radian
    stop_trans_diff = 0.02              # meters
    y_tolerance_pid       = 0.02
    yaw_predock_tolerance = 0.05
    max_parallel_offset   = 0.03        # m, will move to parallel.c if exceeded
    predock_tf_samples    = 10          # tf samples to avg, parallel.c validation

    # [Velocity Informations]
    linear_vel_range  = (-0.2, 0.2)
    angular_vel_range = (-0.15, 0.15)
    max_x_with_retry_high_current = 0.05      
    max_angular_vel = 0.3              # rad/s
    min_angular_vel = 0.05
    max_linear_vel  = 0.2
    max_linear_vel_predock = 0.15
    min_linear_vel  = 0.025
    max_x_lastmile   = 0.05             # m/s, for lastmile
    max_x_out_dock   = 0.2        
    max_x_correction = 0.05
    max_x_pid_steer  = 0.02
    min_x_pid_steer  = 0.02
    max_x_pid_lastmile = 0.04
    min_x_pid_lastmile = 0.02
    max_x_go_out_dock = 0.25
    max_x_custom_rotation = 0.1
    max_w_correction = 0.15
    max_w_predock = 0.1
    max_w_rotate_to_dock = 0.18
    max_w_custom_rotation = 0.2

    # PID controller params
    k_p = 0.8
    k_i = 0.0
    k_d = 0.01
    k_p_steer = 2.5
    k_i_steer = 0.0
    k_d_steer = 0.01

    retry_count = 5
    debug_mode = True


class AutoDockServer:
    
    def __init__(self, config: AutodockConfig, run_server: bool):

        self.cfg = config
        self.run_server = run_server

        # param check
        assert (len(self.cfg.linear_vel_range) == 2 and
                len(self.cfg.linear_vel_range) == 2
                ), "linear and angular vel range should have size of 2"
        assert (self.cfg.linear_vel_range[0] < self.cfg.linear_vel_range[1]
                ), "linear vel min should be larger than max"
        assert (self.cfg.angular_vel_range[0] < self.cfg.angular_vel_range[1]
                ), "linear vel min should be larger than max"

        # create_subscriber to tf broadcaster
        self.__tfBuffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.__tf_listener = tf2_ros.TransformListener(self.__tfBuffer)

        # Initialize variables:
        self.left_range = 0.0
        self.right_range = 0.0
        self.is_pausing_  = False
        self.high_motor_pickup_current = False
        self.high_motor_drop_current = False
        self.is_obstacle_detected_ = False
        self.custom_rotation = False
        self.pause_flag = False
        self.slider_sensor_state = [0,0]
        self.cart_sensor_state = (0,0)
        self.tag_frame = ""
        self.time_out_remain = self.cfg.dock_timeout
        self.mode = AutoDockingGoal()
        self.dock_state = DockState.IDLE
        self.start_time = rospy.Time.now()
        self.rate = rospy.Rate(self.cfg.controller_frequency)

        # PID
        self.kp = 2.5
        self.kd = 0.005
        self.last_error = 0.0

        # Dynamic reconfigure for line extraction
        self.line_extraction_client = client.Client("/back_line_extractor")
        self.default_le_params = {'max_range': 1.0, 'max_line_gap': 0.02, 'min_split_dist': 0.02, 'outlier_dist': 0.02}
        self.pickup_le_params = {'max_range': 0.5, 'max_line_gap': 0.02, 'min_split_dist': 0.04, 'outlier_dist': 0.03}
        self.dropoff_le_params = {'max_range': 1.0, 'max_line_gap': 0.02, 'min_split_dist': 0.04, 'outlier_dist': 0.03}

        # Dynamic reconfigure for polygon scanner
        self.polygon_client = client.Client("/back_scanner_filter/back_scanner_filter")
        self.default_polygon_params = {'polygon': [[-0.05, -0.35], [-0.05, 0.35], [1.2, 0.35], [1.2, -0.35]]}
        self.pickup_polygon_params = {'polygon': [[-0.05, -0.25], [-0.05, 0.25], [1.2, 0.25], [1.2, -0.25]]}
        self.dropoff_polygon_params = {'polygon': [[-0.05, -0.5], [-0.05, 0.5], [1.2, 0.5], [1.2, -0.5]]}


        # debug timer for state machine marker
        if self.cfg.debug_mode:
            self.pub_marker = rospy.Publisher('/sm_maker', Marker, queue_size=1)
            self.__timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)

        # Apriltag continuous detection service
        self.apriltag_client = rospy.ServiceProxy("/back_camera/apriltag_ros/enable_detector", SetBool)
        rospy.loginfo("AutoDockServer: Connecting to /back_camera/apriltag_ros/enable_detector service...")
        self.apriltag_client.wait_for_service()
        rospy.loginfo("AutoDockServer: Connected to /back_camera/apriltag_ros/enable_detector service.")

        # Create line extraction service
        self.front_line_extraction_client = rospy.ServiceProxy("/front_line_extractor/enable_detector", SetBool)
        self.back_line_extraction_client = rospy.ServiceProxy("/back_line_extractor/enable_detector", SetBool)
        rospy.loginfo("AutoDockServer: Connecting to front & back line extraction detector service...")
        self.front_line_extraction_client.wait_for_service()
        self.back_line_extraction_client.wait_for_service()
        rospy.loginfo("AutoDockServer: Connected to front & back line extraction detector service.")

        # Publishers
        self.cmd_vel_pub_    = rospy.Publisher("/amr/mobile_base_controller/cmd_vel", Twist, queue_size=5)
        self.cmd_brake_pub_  = rospy.Publisher("cmd_brake", Bool, queue_size=5)
        self.cmd_slider_pub_ = rospy.Publisher("cmd_slider", Int16, queue_size=5)
        self.error_pub_      = rospy.Publisher("error_name", Int16, queue_size=5)
        self.turn_off_back_safety_pub_ = rospy.Publisher("turn_off_back_safety", Bool,queue_size=5)
        self.turn_off_front_safety_pub_ = rospy.Publisher("turn_off_front_safety", Bool,queue_size=5)
        self.turn_off_ultrasonic_safety_pub_ = rospy.Publisher("turn_off_ultrasonic_safety", Bool,queue_size=5)

        # Subscribers
        rospy.Subscriber("PAUSE_AMR", Bool, self.pause_callback)
        rospy.Subscriber("pickup_current_state", Bool, self.pickup_current_callback)
        rospy.Subscriber("drop_current_state", Bool, self.dropoff_current_callback)
        rospy.Subscriber("cart_sensor_state", SliderSensorStamped, self.cart_sensor_state_callback)
        rospy.Subscriber("slider_sensor_state", SliderSensorStamped, self.slider_sensor_state_callback)
        rospy.Subscriber("status_protected_field", Bool, self.protected_field_callback)
        rospy.Subscriber("back_scan_rep177", LaserScan, self.laser_scan_callback)

        # Autodock action
        if run_server:
            self.autodock_action = actionlib.SimpleActionServer("autodock_action",
                                                                AutoDockingAction,
                                                                execute_cb=self.auto_docking_callback,
                                                                auto_start=False)
            self.autodock_action.start()
            self.feedback_msg = AutoDockingFeedback()
    
    def enable_line_detector(self, laser_name:str, signal:bool):
        """
        `laser_name`: "front" or "back"
        """
        msg = "disable" if not signal else "enable"

        try:
            rospy.loginfo(f"AutoDockServer: Waiting {msg} line detector from server...")

            if (laser_name == "front"):
                result = self.front_line_extraction_client.call(signal)
            elif (laser_name == "back"):
                result = self.back_line_extraction_client.call(signal)

            if result.success:
                rospy.loginfo("AutoDockServer: " + result.message)
                rospy.sleep(1.0)
                return True
            else:
                return False
        
        except rospy.ServiceException as e:
            rospy.logerr("AutoDockServer: Service call failed: %s"%e)
    
    def distance2D(self, x, y):
        return (math.sqrt(pow(x, 2) + pow(y, 2)))
    
    def rotate_to_dock(self, angle_to_dock):
        if angle_to_dock != 0:
            return self.rotate_with_odom(self.cfg.min_angular_vel, self.cfg.max_angular_vel, angle_to_dock*math.pi/180)
        return True

    def update_line_extraction_params(self, signal):
        """
        `0`: Default params | `1`: Pickup params | `2`: Dropoff params
        """
        if (signal == 0):
            self.line_extraction_client.update_configuration(self.default_le_params)

        elif(signal == 1):
            self.line_extraction_client.update_configuration(self.pickup_le_params)

        elif(signal == 2):
            self.line_extraction_client.update_configuration(self.dropoff_le_params)

        # Create a delay
        rospy.sleep(0.2)
    
    def update_polygon_params(self, signal):
        """
        `0`: Default params | `1`: Pickup params | `2`: Dropoff params
        """
        if (signal == 0):
            self.polygon_client.update_configuration(self.default_polygon_params)
        
        elif (signal == 1):
            self.polygon_client.update_configuration(self.pickup_polygon_params)

        elif (signal == 2):
            self.polygon_client.update_configuration(self.dropoff_polygon_params)

        # Create a delay
        rospy.sleep(0.2)

    def check_dock_frame(self, laser_frame, tag_frame):
        laser_tf = self.get_tf(laser_frame)
        tag_tf = self.get_tf(tag_frame)

        if laser_tf is None:
            return False
        
        if (laser_tf is not None and tag_tf is not None):
            x, y, yaw    = utils.get_2d_pose(laser_tf)
            x1, y1, yaw1 = utils.get_2d_pose(tag_tf)
        
            return ((abs(x - x1) <= 0.03)
                    and (abs(y - y1) <= 0.02))
        
        return True

    def PIDController(self, dis_y):
        error = dis_y - self.last_error
        angle = self.kp*dis_y + self.kd*error
        self.last_error = dis_y
        
        return angle
                
    def correct_robot(self, offset, signal, correcttion_angle=0, rotate_type=BOTH,
                      factor_30=math.sqrt(3), factor_45=math.sqrt(2), factor_60=2/math.sqrt(3)) -> bool:
        """
        Correcting robot respective to dock frame

        `signal`: Whether the goal is behind? (Depend on x coordinate)

        `correcttion_angle`: Choose the correcttion_angle respect with 30, 45, or 90 degrees

        `correcttion_angle = 30`: Correcting robot with 30 degrees

        `correcttion_angle = 45`: Correcting robot with 45 degrees

        `correcttion_angle = 60`: Correcting robot with 60 degrees

        `correcttion_angle = 90`: Correcting robot with 90 degrees

        Default correcttion_angle = 30 degrees

        `rotate_type = ONLY_RIGHT`: Clockwise rotating
        `rotate_type = ONLY_LEFT`: Counter clockwise rotating
        """        
        if rotate_type == ONLY_RIGHT and offset < 0:
            a = 1
            b = 0
        elif rotate_type == ONLY_LEFT and offset > 0: 
            a = -1
            b = 0
        elif offset > 0:
            a = 1
            b = 1
        elif offset < 0:
            a = -1
            b = 1
        
        if correcttion_angle == 0 or correcttion_angle == 30:
            msg = "30"
            alpha = math.pi/6
            x1 = 2*a*offset
            x2 = -a*b*factor_30*offset
        
        elif correcttion_angle == 45:
            msg = "45"
            alpha = math.pi/4
            x1 = a*factor_45*offset
            x2 = -a*b*offset

        elif correcttion_angle == 60:
            msg = "60"
            alpha = math.pi/3
            x1 = a*factor_60*offset
            x2 = -0.5*a*b*factor_60*offset

        elif correcttion_angle == 90:
            msg = "90"
            alpha = math.pi/2
            x1 = a*offset
            x2 = 0

        self.set_state(DockState.CORRECTION, f"AutoDockServer: Correcting robot {msg} degrees with {offset:.2f}m!")

        if signal > 0:
            a = -1   

        return(
            self.rotate_with_odom(self.cfg.min_angular_vel, self.cfg.max_angular_vel, a * alpha)
            and self.move_with_odom(self.cfg.min_linear_vel,self.cfg.max_linear_vel_predock, x1)
            and self.rotate_with_odom(self.cfg.min_angular_vel, self.cfg.max_angular_vel, -a * alpha)
            and self.move_with_odom(self.cfg.min_linear_vel,self.cfg.max_linear_vel_predock, x2)
        )

    def auto_correction(self, x_distance, y_distance, distance_threshold, laser_offset):
        """
        Robot correct and move backward if distance from back_laser_link to tag_frame > distance_threshold
        """
        safety_distance = abs(x_distance) - (distance_threshold + laser_offset)
        if (laser_offset == self.cfg.front_laser_offset):
            dir = 1
        elif (laser_offset == self.cfg.back_laser_offset):
            dir = -1

        rot_angle = 0.0
        if safety_distance < 0:
            return False
        else:
            rot_angle_min = -math.atan(y_distance / safety_distance)
            sign = 1 if (rot_angle_min > 0) else -1
            if (abs(rot_angle_min) + (5*math.pi / 180)) >= math.pi/2:
                if (abs(rot_angle_min) < math.pi/2):
                    rot_angle = rot_angle_min
                else:
                    rot_angle = sign * (math.pi/2)
                    # return False
            else:
                rot_angle = rot_angle_min + sign*(5*math.pi / 180)

            dis_move = dir*abs(y_distance / math.sin(rot_angle))

            if self.cfg.debug_mode:
                print(f"AutoDockServer: Rotate robot with {rot_angle}rad and move {dis_move}m.")
                
            self.set_state(DockState.CORRECTION, f"AutoDockServer: Correcting robot with angle flex - offset: {y_distance:.2f}m!")

            return (
                self.rotate_with_odom(self.cfg.min_angular_vel, self.cfg.max_angular_vel, rot_angle)
                and self.move_with_odom(self.cfg.min_linear_vel, self.cfg.max_linear_vel_predock, dis_move)
                and self.rotate_with_odom(self.cfg.min_angular_vel, self.cfg.max_angular_vel, -rot_angle)
            )
        
    def custom_rotation_after_undock(self, dock_name:str)->bool:
        if dock_name in self.cfg.custom_dock_name:
            rospy.loginfo("Executing custom rotating dock...")
            for k in self.cfg.custom_dock_name[dock_name]:
                for action, value in self.cfg.custom_dock_name[dock_name][k].items():
                    if action == "rotate":
                        if not self.rotate_with_odom(self.cfg.min_angular_vel, self.cfg.max_angular_vel, value*math.pi/180):
                            return False
                    elif action == "move":
                        if not self.move_with_odom(self.cfg.min_linear_vel, self.cfg.max_linear_vel, value):
                            return False
                    else:
                        rospy.logerr("Autodock: action key is not properly, available key is 'rotate' or move!")
                        return False
        return True

    def enable_apriltag_detector(self, signal):
        """
        `signal = False`: Disable | `signal = True`: Enable
        """
        msg = "disable" if not signal else "enable"

        try:
            rospy.loginfo(f"AutoDockServer: Waiting {msg} apriltag detection from server...")
            result = self.apriltag_client.call(signal)

            if result.success:
                rospy.loginfo("AutoDockServer: " + result.message)
                return True
            else:
                return False
        
        except rospy.ServiceException as e:
            rospy.logerr("AutoDockServer: Service call failed: %s"%e)
    
    def reset(self):
        self.dock_state = DockState.IDLE
        self.is_pausing_  = False
        self.high_motor_pickup_current = False
        self.high_motor_drop_current = False
        self.brake(False)
    
    def reset_after_fail(self):
        self.publish_velocity()
        self.turn_off_back_safety(False)
        self.turn_off_front_safety(False)
        self.turn_off_ultrasonic_safety(False)
        self.enable_line_detector('front', False)
        self.enable_line_detector('back', False)
        self.enable_apriltag_detector(False)

    def reset_high_current(self):
        self.high_motor_pickup_current = False
        self.high_motor_drop_current = False

    def laser_scan_callback(self, msg: LaserScan):
        self.left_range = min(msg.ranges[80:100]) 
        self.right_range = min(msg.ranges[1171:1191])

    def slider_sensor_state_callback(self, msg: SliderSensorStamped):
        self.slider_sensor_state = msg.sensor_state.state

    def protected_field_callback(self, msg: Bool):
        self.is_obstacle_detected_ = msg.data

    def pause_callback(self, msg: Bool):
        self.is_pausing_ = msg.data 

    def cart_sensor_state_callback(self, msg: SliderSensorStamped):
        self.cart_sensor_state = msg.sensor_state.state
    
    def turn_off_back_safety(self, signal):
        self.turn_off_back_safety_pub_.publish(signal)

    def turn_off_front_safety(self, signal):
        self.turn_off_front_safety_pub_.publish(signal)

    def turn_off_ultrasonic_safety(self, signal):
        self.turn_off_ultrasonic_safety_pub_.publish(signal)

    def brake(self, signal):
        self.cmd_brake_pub_.publish(signal)                                                                                                                                                                                                                                                                                                                                                                                                            

    def pub_slider_cmd(self, signal):
        """
        `signal = 1`: Slider go out
        `signal = 2`: Slider go in
        """
        msg = "OUT" if signal == 1 else "IN"
        rospy.loginfo(f"AutoDockServer: PUBLISHING SLIDER MOTOR GO {msg}!")

        self.cmd_slider_pub_.publish(signal)

    def pickup_current_callback(self, msg: Bool):
        self.high_motor_pickup_current = msg.data

    def dropoff_current_callback(self, msg: Bool):
        self.high_motor_drop_current = msg.data

    def start(self, mode, dock_name, tag_ids, angle_to_dock, correction_angle, rotate_type, distance_go_out) -> bool:
        """
        Virtual function. This function will be triggered when autodock request
        is requested
        :return : if action succeeded
        """
        rospy.logwarn("Server implementation has not been specified. "
                      "Do overload the start() function")
        return False

    def set_state(self, state: DockState, printout=""):
        """
        set state of the auto dock server
        :param state:       Current utils.DockState
        :param printout:    Verbose description of the state
        """
        state_str = DockState.to_string(state)
        self.dock_state = state

        if (state == DockState.ERROR):
            rospy.logerr(f"AutoDockServer: State: [{state_str}] | {printout}!")
        
        else:
            rospy.loginfo(f"AutoDockServer: State: [{state_str}] | {printout}")

        if self.run_server:
            self.feedback_msg.state = state
            self.feedback_msg.progress = DockState.to_percent(state)
            self.feedback_msg.status = f"{state_str} | {printout}"
            self.autodock_action.publish_feedback(self.feedback_msg)

    
    def printout_success(self, printout=""):
        rospy.loginfo(f"AutoDockServer: State: [{DockState.to_string(self.dock_state)}] | {printout}")

    
    def printout_error(self, printout=""):
        rospy.logerr(f"AutoDockServer: State: [{DockState.to_string(self.dock_state)}] | {printout}")

    
    def retry(self, dock_tf_name) -> bool:
        """
        Check if not dectect the dock frame, will be retry
        """
        self.set_state(DockState.RETRY, "AutoDockServer: Retrying auto docking...!")

        counter = 1
        while not rospy.is_shutdown():
            if self.do_pause():
                pass
            else:
                dock_tf = self.get_tf(dock_tf_name)
                if dock_tf is None:
                    if counter > self.cfg.retry_count:
                        rospy.logerr("AutoDockServer: Not dectect the dock frame after execute retry!")
                        return False
                    rospy.loginfo(f"AutoDockServer: Retrying again: {counter}/{self.cfg.retry_count}!")
                    counter += 1
                else:
                    return True
            rospy.Rate(5).sleep()


    def check_cancel(self) -> bool:
        """
        Check if to cancel this docking action. This will happen if a
        preempt is requested during server mode. or if a timeout is reached.
        :return : true if cancel is requested. false as default
        """
        if self.run_server and self.autodock_action.is_preempt_requested():
            self.set_state(DockState.CANCEL, 'AutoDockServer: Preempted Requested!')
            return True

        # check if dock_timeout reaches
        if (rospy.Time.now() - self.start_time).secs > self.time_out_remain:
            rospy.logerr('Timeout reaches!')
            self.set_state(self.dock_state, "AutoDockServer: Reach Timeout")
            return True
        return False
    

    def do_pause(self):
        if (self.is_pausing_ or self.is_obstacle_detected_):
            if (not self.pause_flag):
                self.set_state(self.dock_state, "AutoDockServer: Pause Requested!")
                self.pause_flag = True
                self.time_out_remain -= (rospy.Time.now() - self.start_time).secs
                self.start_time = rospy.Time.now()

        else:
            self.pause_flag = False
        return (self.is_pausing_ or self.is_obstacle_detected_)


    def publish_velocity(self, linear_vel=0.0, angular_vel=0.0):
        """
        Command the robot to move, default param is STOP!
        """
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel

        msg.linear.x  = utils.clamp(msg.linear.x,
                                    self.cfg.linear_vel_range[0],
                                    self.cfg.linear_vel_range[1])
        msg.angular.x = utils.clamp(msg.angular.x,
                                    self.cfg.angular_vel_range[0],
                                    self.cfg.angular_vel_range[1])
        self.cmd_vel_pub_.publish(msg)

    
    def get_tf(self,
               target_link=None,
               ref_link=None,
               target_time=None,
               transform_timeout=0.5) -> np.ndarray:
        """
        This will provide the transformation of the marker,
        if ref_link is not provided, we will use robot's base_link as ref
        :param now : this is a hack fix
        :return : 4x4 homogenous matrix, None if not avail
        """
        if ref_link is None:
            ref_link = self.cfg.base_link
        if target_link is None:
            target_link = self.cfg.last_frame
        if target_time is None:
            target_time = rospy.Time.now()

        try:
            return utils.get_mat_from_transfrom_msg(
                self.__tfBuffer.lookup_transform(
                    ref_link, target_link, target_time,
                    rospy.Duration(transform_timeout))
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logerr(f"AutoDockServer: Failed lookup: {target_link}, from {ref_link}")
            return None
        
    
    def get_2D_pose(self, target_link=None, base_link=None):

        if target_link is None:
            target_link = self.cfg.last_frame
        if base_link is None:
            base_link = self.cfg.base_link

        try:
            trans = self.__tfBuffer.lookup_transform(
                base_link,
                target_link,
                rospy.Time(0))

            rotation = euler_from_quaternion([trans.transform.rotation.x,
                                              trans.transform.rotation.y,
                                              trans.transform.rotation.z,
                                              trans.transform.rotation.w])
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logerr(f"AutoDockServer: Failed lookup: {target_link}, from {base_link}")
            return None
        
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        theta = rotation[2]
        
        return x, y, theta


    def get_odom(self) -> np.ndarray:
        """
        Get the current odom of the robot
        :return : 4x4 homogenous matrix, None if not avail
        """
        try:
            return utils.get_mat_from_odom_msg(
                rospy.wait_for_message(
                    "/amr/odometry/filtered", Odometry, timeout=self.cfg.tf_expiry)
            )
        except rospy.exceptions.ROSException:
            rospy.logerr(f"AutoDockServer: Failed to get odom")
            return None


    def retry_if_high_current(self, forward, times=0, limit=None) ->bool:
        """
        Move robot forward when catch high motor current
        `forward`: How far for moving robot
        `times`: How many times for retry
        `limit`: If `times` > `limit` --> ERROR 
        """
        if times == limit:
            rospy.logerr(f"AutoDockServer: The times of high current exceed {limit}!")
            self.error_pub_.publish(1)
            return False
        
        self.set_state(self.dock_state, f"AutoDockServer: Move with encoder {forward}m because high motor current!")
        return (self.move_with_odom(self.cfg.min_linear_vel,self.cfg.max_linear_vel_predock, forward))


    def move_with_odom(self, min_speed: float, max_speed: float ,forward: float) -> bool:
        """
        Move robot in linear motion with Odom. Blocking function
        :return : success
        """
        self.set_state(self.dock_state, f"Move robot: {forward:.2f}m!")

        _initial_tf = self.get_odom()
        if _initial_tf is None:
            return False

        # get the tf mat from odom to goal frame
        _goal_tf = utils.apply_2d_transform(_initial_tf, (forward, 0, 0))
        _pid = PID(self.cfg.k_p, self.cfg.k_i, self.cfg.k_d, min_speed, max_speed)
        # second mat
        prev_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.check_cancel():
                return False

            elif self.do_pause():
                pass

            else:
                if (self.dock_state == DockState.STEER_DOCK or
                    self.dock_state == DockState.LAST_MILE):
                    if (self.high_motor_drop_current or
                        self.high_motor_pickup_current):
                        self.reset_high_current()
                        self.publish_velocity()
                        if not self.retry_if_high_current(0.4):
                            return False
                        return True

                _curr_tf = self.get_odom()
                if _curr_tf is None:
                    return False

                dx, dy, dyaw = utils.compute_tf_diff(_curr_tf, _goal_tf)

                if abs(dx) < self.cfg.stop_trans_diff:
                    self.publish_velocity()
                    rospy.loginfo("AutoDockServer: Done with move robot")
                    return True

                # This makes sure that the robot is actually moving linearly
                time_now = rospy.Time.now()
                dt = (time_now - prev_time).to_sec()
                l_vel_pid = _pid.update(forward, forward - dx, dt)
                ang_vel = utils.sat_proportional_filter(dyaw, abs_max=self.cfg.min_angular_vel, factor=0.2)
                l_vel   = utils.bin_filter(dx, l_vel_pid)

                self.publish_velocity(linear_vel=l_vel, angular_vel=ang_vel)
                prev_time = time_now
            self.rate.sleep()
        exit(0)

    
    def rotate_with_odom(self, min_angular_vel: float, max_angular_vel: float, rotate: float) -> bool:
        """
        Spot Rotate the robot with odom. Blocking function
        :return : success
        `rotate`: How many degrees for rotating
        `v_w`: angular velocity
        """
        self.set_state(self.dock_state, f"Turn robot: {rotate:.2f} rad!")

        _initial_tf = self.get_odom()
        if _initial_tf is None:
            return False

        # get the tf mat from odom to goal frame
        _goal_tf = utils.apply_2d_transform(_initial_tf, (0, 0, rotate))
        _pid = PID(self.cfg.k_p, self.cfg.k_i, self.cfg.k_d, min_angular_vel, max_angular_vel)
        
        prev_time = rospy.Time.now()
        while not rospy.is_shutdown():

            if self.check_cancel():
                return False

            elif self.do_pause():
                pass
            
            else:
                if (self.dock_state == DockState.STEER_DOCK or
                    self.dock_state == DockState.LAST_MILE):
                    if (self.high_motor_drop_current or
                        self.high_motor_pickup_current):
                        self.reset_high_current()
                        self.publish_velocity()
                        return False
                    
                _curr_tf = self.get_odom()
                if _curr_tf is None:
                    return False

                dx, dy, dyaw = utils.compute_tf_diff(_curr_tf, _goal_tf)

                if abs(dyaw) < self.cfg.stop_yaw_diff:
                    self.publish_velocity()
                    rospy.loginfo("AutoDockServer: Done with rotate robot")
                    return True
                
                time_now = rospy.Time.now()
                dt = (time_now - prev_time).to_sec()
                angular_vel_pid = _pid.update(rotate, rotate - dyaw, dt)             
                sign = 1 if rotate > 0 else -1
                angular_vel = sign*angular_vel_pid
                
                self.publish_velocity(angular_vel=angular_vel)
                prev_time = time_now
            self.rate.sleep()
        exit(0)


    def auto_docking_callback(self, goal: AutoDockingGoal):
        self.start_time = rospy.Time.now()
        self.time_out_remain = self.cfg.dock_timeout
        _result = AutoDockingResult()
        _result.is_success = self.start(goal.mode, goal.dock_name, goal.tag_ids,
                                        goal.angle_to_dock, goal.correction_angle,
                                        goal.rotate_type, goal.distance_go_out)
        
        _prev_state = DockState.to_string(self.feedback_msg.state)

        if _result.is_success:
            _duration = rospy.Time.now() - self.start_time
            _result.status = f"AutoDockServer: Succeeded! Took {_duration.secs}s"
            self.autodock_action.set_succeeded(_result)

        elif self.autodock_action.is_preempt_requested():
            _result.is_success = False
            _result.status = f"AutoDockServer: Cancel during [{_prev_state}], " \
                             f"with status: {self.feedback_msg.status}"
            self.autodock_action.set_preempted(_result)
            self.set_state(DockState.IDLE, "Dock Action is canceled")

        else:
            _result.is_success = False
            _result.status = f"AutoDockServer: Failed during [{_prev_state}], " \
                             f"with status: {self.feedback_msg.status}"
            self.autodock_action.set_aborted(_result)
            self.set_state(DockState.IDLE, "Failed execute Dock Action")


    def timer_callback(self, timer):
        # This is mainly for debuging
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.cfg.base_link
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose.position.z = 1.1
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 1
        marker.color.b = 1
        marker.color.a = 1
        marker.text = DockState.to_string(self.dock_state)
        self.pub_marker.publish(marker)