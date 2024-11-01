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
from amr_msgs.msg import CheckerSensorStateStamped
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
from amr_msgs.msg import RobotState
from amr_autodocking.pid import PID


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
    charger_link = "charger_frame"
    first_frame = "first_frame"
    last_frame = "last_frame"
    parallel_frame = "parallel_frame"
   
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
        self.is_pause  = False
        self.high_motor_pickup_current = False
        self.high_motor_drop_current = False
        self.obstacle_status = False
        self.custom_rotation_state = 0
        self.custom_rotation = False
        self.pause_flag = False
        self.slider_sensor_state = [0,0]
        self.cart_sensor_state = (0,0)
        self.tf_tag_name = ""
        self.msg = ""
        self.tag_frame = ""
        self.first_name = ""
        self.dock_name = 0
        self.time_out_remain = self.cfg.dock_timeout
        self.goal = AutoDockingGoal()
        self.robot_state = RobotState()
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
            self.__timer = rospy.Timer(rospy.Duration(0.5), self.timerCB)

        # Move_to_pose client
        # rospy.logwarn("AutoDockServer: Connecting to move to pose service...")
        # rospy.wait_for_service("move_to_pose")
        # rospy.loginfo("AutoDockServer: Connected to move to pose service.")
        # self.move_to_pose_client = rospy.ServiceProxy("move_to_pose", MoveToPose)

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
        self.pub_cmd_vel    = rospy.Publisher("/amr/mobile_base_controller/cmd_vel", Twist, queue_size=5)
        self.pub_cmd_brake  = rospy.Publisher("cmd_brake", Bool, queue_size=5)
        self.pub_cmd_slider = rospy.Publisher("cmd_slider", Int16, queue_size=5)
        self.pub_error      = rospy.Publisher("error_name", Int16, queue_size=5)
        self.pub_turn_off_back_safety = rospy.Publisher("turn_off_back_safety", Bool,queue_size=5)
        self.pub_turn_off_front_safety = rospy.Publisher("turn_off_front_safety", Bool,queue_size=5)
        self.pub_turn_off_ultrasonic_safety = rospy.Publisher("turn_off_ultrasonic_safety", Bool,queue_size=5)

        # Subscribers
        rospy.Subscriber("PAUSE_AMR", Bool, self.pauseAMRCB)
        rospy.Subscriber("pickup_current_state", Bool, self.pickupCurrentStateCB)
        rospy.Subscriber("drop_current_state", Bool, self.dropCurrentStateCB)
        rospy.Subscriber("cart_sensor_state", CheckerSensorStateStamped, self.cartSensorStateCB)
        rospy.Subscriber("slider_sensor_state", CheckerSensorStateStamped, self.sliderSensorStateCB)
        rospy.Subscriber("status_protected_field", Bool, self.protectedFieldCB)
        rospy.Subscriber("back_scan_rep177", LaserScan, self.laserScanCB)
        rospy.Subscriber("custom_rotation", Int16, self.customRotationCB)
        rospy.Subscriber("dock_name", Int16, self.dockNameCB)

        # Autodock action
        if run_server:
            self.autodock_action = actionlib.SimpleActionServer("autodock_action",
                                                                AutoDockingAction,
                                                                execute_cb=self.autoDockActionCB,
                                                                auto_start=False)
            self.autodock_action.start()
            self.feedback_msg = AutoDockingFeedback()

    
    def enableLineDetector(self, laser_name:str, signal:bool):
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

    
    def customRotationCB(self, msg: Int16):
        self.custom_rotation_state = msg.data

    
    def dockNameCB(self, msg:Int16):
        self.dock_name = msg.data

    
    def rotateWithCustomRotation(self, forward):
        return (self.rotateWithOdom(self.cfg.min_angular_vel, self.cfg.max_angular_vel, -math.pi/3) and
                self.moveWithOdom(self.cfg.min_linear_vel, self.cfg.max_linear_vel, forward) and
                self.rotateWithOdom(self.cfg.min_angular_vel, self.cfg.max_angular_vel, -math.pi/6))
    

    def updateLineExtractionParams(self, signal):
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

    
    def updatePolygonParams(self, signal):
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


    def checkLaserFrame(self, laser_tf=None, tag_tf=None, laser_tf_name=None, tag_tf_name=None):
        """
        Check laser frame is close with tag frame
        """
        if (laser_tf_name is not None and tag_tf_name is not None):
            laser_tf_ = self.get_tf(laser_tf_name)
            tag_tf_ = self.get_tf(tag_tf_name)
        
        elif (laser_tf is not None and tag_tf is not None):
            laser_tf_ = laser_tf
            tag_tf_ = tag_tf

        dock_x, dock_y, dock_yaw = utils.get_2d_pose(laser_tf_)
        tag_x, tag_y, tag_yaw    = utils.get_2d_pose(tag_tf_)

        return (self.distance2D(dock_x - tag_x, dock_y - tag_y) <= 0.04)
    

    def PIDController(self, dis_y):
        error = dis_y - self.last_error
        angle = self.kp*dis_y + self.kd*error
        self.last_error = dis_y
        
        return angle

                
    def correct_robot(self, offset, signal, rotate_angle=30, rotate_orientation=0,
                      factor_30=math.sqrt(3), factor_45=math.sqrt(2), factor_60=2/math.sqrt(3)) -> bool:
        """
        Correcting robot respective to dock frame

        `signal`: Whether the goal is behind? (Depend on x coordinate)

        `rotate_angle`: Choose the rotate_angle respect with 30, 45, or 90 degrees
        `rotate_angle = 30`: Correcting robot with 30 degrees
        `rotate_angle = 45`: Correcting robot with 45 degrees
        `rotate_angle = 60`: Correcting robot with 60 degrees
        `rotate_angle = 90`: Correcting robot with 90 degrees

        `rotate_orientation = 1`: Counter clockwise rotating
        `rotate_orientation = 2`: Clockwise rotating
        """
        assert (type(rotate_orientation) == int), "rotate_orientation is not int type!"
        
        if rotate_orientation == 1 and offset < 0:
            a = 1
            b = 0
        elif rotate_orientation == 2 and offset > 0: 
            a = -1
            b = 0
        elif offset > 0:
            a = 1
            b = 1
        elif offset < 0:
            a = -1
            b = 1
        
        if rotate_angle == 30:
            msg = "30"
            alpha = math.pi/6
            x1 = 2*a*offset
            x2 = -a*b*factor_30*offset
        
        elif rotate_angle == 45:
            msg = "45"
            alpha = math.pi/4
            x1 = a*factor_45*offset
            x2 = -a*b*offset

        elif rotate_angle == 60:
            msg = "60"
            alpha = math.pi/3
            x1 = a*factor_60*offset
            x2 = -0.5*a*b*factor_60*offset

        elif rotate_angle == 90:
            msg = "90"
            alpha = math.pi/2
            x1 = a*offset
            x2 = 0

        self.setState(DockState.CORRECTION, f"AutoDockServer: Correcting robot {msg} degrees with {offset:.2f}m!")

        if signal > 0:
            a = -1   

        return(
            self.rotateWithOdom(self.cfg.min_angular_vel, self.cfg.max_angular_vel, a * alpha)
            and self.moveWithOdom(self.cfg.min_linear_vel,self.cfg.max_linear_vel_predock, x1)
            and self.rotateWithOdom(self.cfg.min_angular_vel, self.cfg.max_angular_vel, -a * alpha)
            and self.moveWithOdom(self.cfg.min_linear_vel,self.cfg.max_linear_vel_predock, x2)
        )
    

    def checkDistanceAndCorrection(self, x_distance, y_distance, distance_threshold, laser_offset):
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
                
            self.setState(DockState.CORRECTION, f"AutoDockServer: Correcting robot with angle flex - offset: {y_distance:.2f}m!")

            return (
                self.rotateWithOdom(self.cfg.min_angular_vel, self.cfg.max_angular_vel, rot_angle)
                and self.moveWithOdom(self.cfg.min_linear_vel, self.cfg.max_linear_vel_predock, dis_move)
                and self.rotateWithOdom(self.cfg.min_angular_vel, self.cfg.max_angular_vel, -rot_angle)
            )
        
    
    # def send_goal(self, x, y, theta):
    #     """
    #     Send goal to move_to_pose
    #     """
    #     try:
    #         rospy.loginfo("Waiting the result from server...")
    #         resp = self.move_to_pose_client(x, y, theta)
    #         return resp.is_success
        
    #     except rospy.ServiceException as e:
    #         rospy.logerr("Service call failed: %s"%e)

    
    def enableApriltag(self, signal):
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
    

    def resetVariable(self):
        """
        Reset some values when start autodock
        """
        self.dock_state = DockState.IDLE
        self.is_pause  = False
        self.high_motor_pickup_current = False
        self.high_motor_drop_current = False
        self.custom_rotation = 0
        self.turnOnBrake(False)


    def resetHighCurrent(self):
        self.high_motor_pickup_current = False
        self.high_motor_drop_current = False


    def laserScanCB(self, msg: LaserScan):
        self.left_range = min(msg.ranges[80:100]) 
        self.right_range = min(msg.ranges[1171:1191])


    def sliderSensorStateCB(self, msg: CheckerSensorStateStamped):
        self.slider_sensor_state = msg.sensor_state.data


    def protectedFieldCB(self, msg: Bool):
        self.obstacle_status = msg.data


    def pauseAMRCB(self, msg: Bool):
        self.is_pause = msg.data 


    def cartSensorStateCB(self, msg: CheckerSensorStateStamped):
        self.cart_sensor_state = msg.sensor_state.data
    

    def turnOffBackLaserSafety(self, signal):
        self.pub_turn_off_back_safety.publish(signal)

    def turnOffFrontLaserSafety(self, signal):
        self.pub_turn_off_front_safety.publish(signal)

    def turnOffUltrasonicSafety(self, signal):
        self.pub_turn_off_ultrasonic_safety.publish(signal)
    

    def turnOnBrake(self, signal):
        self.pub_cmd_brake.publish(signal)


    def onOffBrake(self):
        self.turnOnBrake(True)
        rospy.sleep(0.5)
        self.turnOnBrake(False)                                                                                                                                                                                                                                                                                                                                                                                                                                                             


    def sliderCommand(self, signal):
        """
        `signal = 1`: Slider go out
        `signal = 2`: Slider go in
        """
        msg = "OUT" if signal == 1 else "IN"
        rospy.loginfo(f"AutoDockServer: PUBLISHING SLIDER MOTOR GO {msg}!")

        self.pub_cmd_slider.publish(signal)
    

    def pickupCurrentStateCB(self, msg: Bool):
        self.high_motor_pickup_current = msg.data


    def dropCurrentStateCB(self, msg: Bool):
        self.high_motor_drop_current = msg.data


    def start(self) -> bool:
        """
        Virtual function. This function will be triggered when autodock request
        is requested
        :return : if action succeeded
        """
        rospy.logwarn("Server implementation has not been specified. "
                      "Do overload the start() function")
        return False

    def setState(self, state: DockState, printout=""):
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

    
    def printOutSuccess(self, printout=""):
        rospy.loginfo(f"AutoDockServer: State: [{DockState.to_string(self.dock_state)}] | {printout}")

    
    def printOutError(self, printout=""):
        rospy.logerr(f"AutoDockServer: State: [{DockState.to_string(self.dock_state)}] | {printout}")

    
    def retry(self, dock_tf_name) -> bool:
        """
        Check if not dectect the dock frame, will be retry
        """
        self.setState(DockState.RETRY, "AutoDockServer: Retrying auto docking...!")

        counter = 1
        while not rospy.is_shutdown():
            if self.doPause():
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


    def checkCancel(self) -> bool:
        """
        Check if to cancel this docking action. This will happen if a
        preempt is requested during server mode. or if a timeout is reached.
        :return : true if cancel is requested. false as default
        """
        if self.run_server and self.autodock_action.is_preempt_requested():
            self.setState(DockState.CANCEL, 'AutoDockServer: Preempted Requested!')
            return True

        # check if dock_timeout reaches
        if (rospy.Time.now() - self.start_time).secs > self.time_out_remain:
            rospy.logerr('Timeout reaches!')
            self.setState(self.dock_state, "AutoDockServer: Reach Timeout")
            return True
        return False
    

    def doPause(self):
        if (self.is_pause or self.obstacle_status):
            if (not self.pause_flag):
                self.setState(self.dock_state, "AutoDockServer: Pause Requested!")
                self.pause_flag = True
                self.time_out_remain -= (rospy.Time.now() - self.start_time).secs
                self.start_time = rospy.Time.now()

        else:
            self.pause_flag = False
        return (self.is_pause or self.obstacle_status)


    def setSpeed(self, linear_vel=0.0, angular_vel=0.0):
        """
        Command the robot to move, default param is STOP!
        """
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel

        if (msg.linear.x > self.cfg.linear_vel_range[1]):
            msg.linear.x = self.cfg.linear_vel_range[1]
        elif(msg.linear.x < self.cfg.linear_vel_range[0]):
            msg.linear.x = self.cfg.linear_vel_range[0]

        if (msg.angular.x > self.cfg.angular_vel_range[1]):
            msg.angular.x = self.cfg.angular_vel_range[1]
        elif(msg.angular.x < self.cfg.angular_vel_range[0]):
            msg.angular.x = self.cfg.angular_vel_range[0]

        self.pub_cmd_vel.publish(msg)

    
    def get_tf(self,
               target_link=None,
               ref_link=None,
               target_time=None) -> np.ndarray:
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
                    rospy.Duration(self.cfg.tf_expiry))
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


    def retryWithHighMotorCurrent(self, forward, times=0, limit=None) ->bool:
        """
        Move robot forward when catch high motor current
        `forward`: How far for moving robot
        `times`: How many times for retry
        `limit`: If `times` > `limit` --> ERROR 
        """
        if times == limit:
            rospy.logerr(f"AutoDockServer: The times of high current exceed {limit}!")
            self.pub_error.publish(1)
            return False
        
        self.setState(self.dock_state, f"AutoDockServer: Move with encoder {forward}m because high motor current!")
        return (self.moveWithOdom(self.cfg.min_linear_vel,self.cfg.max_linear_vel_predock, forward))


    def moveWithOdom(self, min_speed: float, max_speed: float ,forward: float) -> bool:
        """
        Move robot in linear motion with Odom. Blocking function
        :return : success
        """
        self.setState(self.dock_state, f"Move robot: {forward:.2f}m!")

        _initial_tf = self.get_odom()
        if _initial_tf is None:
            return False

        # get the tf mat from odom to goal frame
        _goal_tf = utils.apply_2d_transform(_initial_tf, (forward, 0, 0))
        _pid = PID(self.cfg.k_p, self.cfg.k_i, self.cfg.k_d, min_speed, max_speed)
        # second mat
        prev_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.checkCancel():
                return False

            elif self.doPause():
                pass

            else:
                if (self.dock_state == DockState.STEER_DOCK or
                    self.dock_state == DockState.LAST_MILE):
                    if (self.high_motor_drop_current or
                        self.high_motor_pickup_current):
                        self.resetHighCurrent()
                        self.setSpeed()
                        if not self.retryWithHighMotorCurrent(0.4):
                            return False
                        return True

                _curr_tf = self.get_odom()
                if _curr_tf is None:
                    return False

                dx, dy, dyaw = utils.compute_tf_diff(_curr_tf, _goal_tf)

                if abs(dx) < self.cfg.stop_trans_diff:
                    self.setSpeed()
                    rospy.loginfo("AutoDockServer: Done with move robot")
                    return True

                # This makes sure that the robot is actually moving linearly
                time_now = rospy.Time.now()
                dt = (time_now - prev_time).to_sec()
                l_vel_pid = _pid.update(forward, forward - dx, dt)
                ang_vel = utils.sat_proportional_filter(dyaw, abs_max=self.cfg.min_angular_vel, factor=0.2)
                l_vel   = utils.bin_filter(dx, l_vel_pid)

                self.setSpeed(linear_vel=l_vel, angular_vel=ang_vel)
                prev_time = time_now
            self.rate.sleep()
        exit(0)

    
    def rotateWithOdom(self, min_angular_vel: float, max_angular_vel: float, rotate: float) -> bool:
        """
        Spot Rotate the robot with odom. Blocking function
        :return : success
        `rotate`: How many degrees for rotating
        `v_w`: angular velocity
        """
        self.setState(self.dock_state, f"Turn robot: {rotate:.2f} rad!")

        _initial_tf = self.get_odom()
        if _initial_tf is None:
            return False

        # get the tf mat from odom to goal frame
        _goal_tf = utils.apply_2d_transform(_initial_tf, (0, 0, rotate))
        _pid = PID(self.cfg.k_p, self.cfg.k_i, self.cfg.k_d, min_angular_vel, max_angular_vel)
        
        prev_time = rospy.Time.now()
        while not rospy.is_shutdown():

            if self.checkCancel():
                return False

            elif self.doPause():
                pass
            
            else:
                if (self.dock_state == DockState.STEER_DOCK or
                    self.dock_state == DockState.LAST_MILE):
                    if (self.high_motor_drop_current or
                        self.high_motor_pickup_current):
                        self.resetHighCurrent()
                        self.setSpeed()
                        return False
                    
                _curr_tf = self.get_odom()
                if _curr_tf is None:
                    return False

                dx, dy, dyaw = utils.compute_tf_diff(_curr_tf, _goal_tf)

                if abs(dyaw) < self.cfg.stop_yaw_diff:
                    self.setSpeed()
                    rospy.loginfo("AutoDockServer: Done with rotate robot")
                    return True
                
                time_now = rospy.Time.now()
                dt = (time_now - prev_time).to_sec()
                angular_vel_pid = _pid.update(rotate, rotate - dyaw, dt)             
                sign = 1 if rotate > 0 else -1
                angular_vel = sign*angular_vel_pid
                
                self.setSpeed(angular_vel=angular_vel)
                prev_time = time_now
            self.rate.sleep()
        exit(0)


    def autoDockActionCB(self, goal: AutoDockingGoal):
        self.start_time = rospy.Time.now()
        self.time_out_remain = self.cfg.dock_timeout
        _result = AutoDockingResult()
        _result.is_success = self.start()
        
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
            self.setState(DockState.IDLE, "Dock Action is canceled")

        else:
            _result.is_success = False
            _result.status = f"AutoDockServer: Failed during [{_prev_state}], " \
                             f"with status: {self.feedback_msg.status}"
            self.autodock_action.set_aborted(_result)
            self.setState(DockState.IDLE, "Failed execute Dock Action")


    def timerCB(self, timer):
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