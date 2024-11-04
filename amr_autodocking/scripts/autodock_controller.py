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
import math
import amr_autodocking.autodock_utils as utils

from amr_autodocking.autodock_server import AutodockConfig, AutoDockServer
from amr_autodocking.autodock_utils import DockState
from apriltag_ros.msg import AprilTagDetectionArray
from amr_autodocking.pid import PID


class AutoDockStateMachine(AutoDockServer):
    """
    Implementation of the AutoDock Server with a Simple State Machine. Also
    this describes the logic of which the control loop of each state.
    """
    def __init__(self,
                 config: AutodockConfig,
                 run_server=True,
                 load_rosparam=False,
                 fake_clock=False):
        if fake_clock:
            rospy.logwarn("AutodockController: WARNING!!!! fake clock is in used, Temporary set use_sim_time to true")
            rospy.set_param("/use_sim_time", True)

        rospy.init_node('auto_dock_node')
        rospy.loginfo("AutodockController node is running!")

        if fake_clock:
            rospy.logwarn("AutodockController: WARNING!!!! fake clock enabled! now disable use_sim_time")
            rospy.set_param("/use_sim_time", False)

        self.cfg = config
        super().__init__(self.cfg, run_server)

        if load_rosparam:
            self.init_params()

        self.dock_state = DockState.IDLE


    def init_params(self):
        print(f"NODE: AutodockController")
        print("PARAMETERS")
        param_names = [attr for attr in dir(self.cfg) if not callable(getattr(self.cfg, attr)) and not attr.startswith("__")]

        # get private rosparam, if none use default
        for param_name in param_names:
            param_val = rospy.get_param("~" + param_name, getattr(self.cfg, param_name))
            print(f"* /{param_name}: {param_val}")
            setattr(self.cfg, param_name, param_val)
            # brute check to ensure numerical config is positive
            if isinstance(param_val, (int, float)):
                assert param_val >= 0, f"[{param_name}] param should be +ve"
        

    def start(self) -> bool:
        rospy.loginfo(f"AutodockController: Start autodock! Will attempt with {self.cfg.retry_count} retry!")

        # Reset some needed values when start docking
        self.reset()

        if (self.dock_name == 57):
            self.enable_line_detector("front", True)
            state = self.robot_state.MODE_CHARGE
        else:
            self.enable_line_detector('back', True)
            
            if (self.dock_name == 1 or self.dock_name == 2 or
                self.dock_name == 5 or self.dock_name == 6):
                state = self.robot_state.MODE_PICKUP
                self.update_line_extraction_params(0)
                self.enable_apriltag_detector(True)
                
                # (TODO) - Sometimes tag is delayed in few miliseconds,
                #        - so waiting for tag with counter.
                wait_for_util_tag_appear = 0
                while True:
                    tag_detections = rospy.wait_for_message("/back_camera/tag_detections",
                                                            AprilTagDetectionArray, timeout=1.0)
                    if (tag_detections is not None):
                        tags = tag_detections.detections

                        min_distance = 100
                        tag_name = ""

                        for tag in tags:
                            bot2dock = self.get_tf(f"tag_frame_{tag.id[0]}")
                            x, y, yaw = utils.get_2d_pose(bot2dock)
                            distance = self.distance2D(x, y)

                            if (distance < min_distance):
                                tag_name = f"tag_frame_{tag.id[0]}"
                                self.tag_frame = tag_name
                                min_distance = distance

                    if tag_name != "":
                        self.tag_frame = tag_name
                        if (self.get_tf(self.cfg.first_frame) is not None and
                            self.check_laser_frame(laser_tf_name=self.cfg.first_frame, tag_tf_name=tag_name)):
                            self.first_name = self.cfg.first_frame
                        else:
                            self.first_name = tag_name
                    else:
                        if wait_for_util_tag_appear == 5:
                            if self.get_tf(self.cfg.first_frame) is None:
                                self.enable_apriltag_detector(False)
                                self.enable_line_detector(False)
                                return False
                            else:
                                self.first_name = self.cfg.first_frame
                                break
                        wait_for_util_tag_appear += 1
                        rospy.sleep(0.5)
                        continue
                    break
            else:
                state = self.robot_state.MODE_DROPOFF
                self.first_name = self.cfg.parallel_frame
                self.update_line_extraction_params(2)
                self.update_polygon_params(2)

        last_frame = self.cfg.last_frame

        while(True):
            if(
                self.pre_dock(state, self.first_name) and
                self.steer_dock(state, self.first_name) and
                self.lastmile_dock(state, last_frame) and
                self.cmd_slider(state) and
                self.go_out_dock(state)
            ):
                if (state == self.robot_state.MODE_CHARGE):
                    self.enable_line_detector("front", False)
                    self.publish_velocity()
                else:
                    self.publish_velocity()
                    self.turn_off_back_safety(False)
                    self.enable_line_detector('back', False)
                self.set_state(DockState.IDLE, "Autodock completed!")
                return True

            # If Dock failed
            if (self.dock_state == DockState.SLIDER_GO_IN or
                self.dock_state == DockState.SLIDER_GO_OUT or
                self.dock_state == DockState.GO_OUT_DOCK):
                self.publish_velocity()
                self.printout_error("Error!")
                self.set_state(DockState.ERROR, "Autodock failed!")
                break    

            self.publish_velocity()
            self.printout_error("Error!")
            self.set_state(DockState.ERROR, "Autodock failed!")

            # check again if it failed because of canceled
            if self.check_cancel():
                break
            
            if self.first_name == self.cfg.parallel_frame:
                if self.retry(self.first_name):
                    self.first_name = self.cfg.parallel_frame
                else: break
            else:
                if (self.retry(self.tag_frame)):
                    self.first_name = self.tag_frame
                elif (self.retry(self.cfg.first_frame)):
                    self.first_name = self.cfg.first_frame
                else:
                    break

        self.reset_after_fail()
        return False
    

    def check_slider_state(self, state: int, sensor_order:int,
                         sensor_check: int, timeout: float) -> bool:
        """
        `sensor_order = 1`: Max position sensor
        `sensor_order = 0`: Original position sensor
        `timeout`: Max time for slider motor reachs original position or max position
        """
        start_time = rospy.Time.now()
        check_pause = False
        timeout_checksensor = 5.0

        self.pub_slider_cmd(state)
        while True:
            if self.check_cancel():
                return False
            
            elif self.do_pause():
                if not check_pause:
                    timeout_used = (rospy.Time.now() - start_time).to_sec() 
                    timeout -= timeout_used
                    timeout_checksensor -= timeout_used
                    check_pause = True
            else:
                if check_pause:
                    self.pub_slider_cmd(state)
                    start_time = rospy.Time.now()
                    check_pause = False

                elif ((rospy.Time.now() - start_time).to_sec() >= timeout):
                    rospy.logerr(f"AutodockController: Slider motor error: Exceed timeout {timeout}s")
                    return False

                elif self.slider_sensor_state[sensor_order] == 1:
                    msg = "max position" if sensor_order == 1 else "original position"
                    rospy.loginfo(f"AutodockController: Slider motor is at {msg}")
                    return True
                
                elif ((rospy.Time.now() - start_time).to_sec() >= timeout_checksensor):
                    if self.slider_sensor_state[sensor_check] == 1:
                        rospy.logerr("AutodockController: Slider motor error: Not working!")
                        return False      
            self.rate.sleep()
                
    
    def steer_dock_with_pickup(self, state, dock_name, max_x, min_x):
        """
        Move straight robot to goal using PID controller
        `max_x`: Max linear velocity
        `min_x`: Min linear velocity
        `tf_name`: Dock tf name
        """
        counter = 0     # How many times for retry_with_high_current
        flag = False

        while not rospy.is_shutdown():
            if self.check_cancel():
                return False
            
            elif self.do_pause():
                pass
            
            elif (self.high_motor_drop_current or
                self.high_motor_pickup_current):
                self.reset_high_current()
                self.publish_velocity()

                if (not self.retry_if_high_current(0.4, counter, 5) and
                    not self.pre_dock(state, dock_name, check_dock_frame=True)):
                    return False

                counter += 1
            
            else:
                if (not flag):
                    dock_tf = self.get_tf(dock_name)
                else:
                    dock_tf = self.get_tf(dock_name)

                if (dock_name == self.cfg.first_frame and dock_tf is None):
                    dock_name = self.tag_frame
                    flag = True
                    continue

                elif ((dock_name == self.tag_frame) and dock_tf is None):
                    return True

                dock_pose = utils.get_2d_pose(dock_tf)
                
                x, y, yaw = dock_pose

                sign = 1 if x > 0 else -1

                if dock_name == self.cfg.first_frame:
                    if (abs(x) - self.cfg.steer_distance_threshold) < 0:
                        return True                 

                angle = self.PIDController(abs(y))
                v = sign*max_x

                if abs(x) < 0.5: v = sign*min_x

                if (y > self.cfg.y_tolerance_pid):
                    w = sign*angle

                elif (y < -self.cfg.y_tolerance_pid):
                    w = -sign*angle

                else:
                    w = 0
                
                # Constrain angle
                if abs(w) > 0.3:
                    w = 0.0
                
                self.publish_velocity(v, w)
            self.rate.sleep()
    
    
    def steer_dock_with_dropoff(self, state, dock_name, max_x, min_x):
        """
        Move straight robot to goal using PID controller
        `max_x`: Max linear velocity
        `min_x`: Min linear velocity
        `tf_name`: Dock tf name
        """
        counter = 0     # How many times for retry_with_high_current
        flag = False
        check_high_current = False

        while not rospy.is_shutdown():
            if self.check_cancel():
                return False
            
            elif self.do_pause():
                pass

            elif (self.high_motor_drop_current or
                  self.high_motor_pickup_current):
                if not check_high_current:
                    check_timer = rospy.Time.now()
                    check_high_current = True
                if ((rospy.Time.now() - check_timer).to_sec() <= 3.0):
                    self.reset_high_current()
                    continue
                self.reset_high_current()
                flag = False
                self.publish_velocity()
                self.update_line_extraction_params(2)

                if (not self.retry_if_high_current(0.4, counter, 5) and
                    not self.pre_dock(state, dock_name)):
                    return False
        
                counter += 1
            else:
                # Check whether back laser in dock (Depend on distance from center of laser to both side of the dock)
                if (self.left_range < 0.3 and self.right_range < 0.3):
                    if not flag:
                        self.update_line_extraction_params(1)
                        start_time = rospy.Time.now()
                        flag = True
                        rospy.loginfo("AutodockController: BackLaser is in dropoff dock!")
                    
                    if (rospy.Time.now() - start_time).to_sec() > 4.5:
                        if not self.high_motor_drop_current:
                            return True
                        
                elif (flag): 
                    rospy.logwarn("AutodockController: BackLaser is out dropoff dock, it's wrong. Please check!")
                    flag = False    # Reset flag for calculate total time

                dock_tf = self.get_tf(dock_name)

                if dock_tf is None:
                    return False

                dock_pose = utils.get_2d_pose(dock_tf)
                
                x, y, yaw = dock_pose

                sign = 1 if x > 0 else -1

                angle = self.PIDController(abs(y))
                v = sign*max_x

                if (y > self.cfg.y_tolerance_pid):
                    w = sign*angle

                elif (y < -self.cfg.y_tolerance_pid):
                    w = -sign*angle

                else:
                    w = 0
                
                # Constrain angle
                if abs(w) > 0.3:
                    w = 0.0
                
                self.publish_velocity(v, w)
            self.rate.sleep()

    def steer_to_charger(self, max_vel_x, min_vel_x):
        _pid = PID(self.cfg.k_p_steer, self.cfg.k_i_steer, self.cfg.k_d_steer)
        prev_time = rospy.Time.now()

        while not rospy.is_shutdown():
            if self.check_cancel():
                return False
            
            elif self.do_pause():
                pass
            
            elif (self.high_motor_drop_current or
                self.high_motor_pickup_current):
                self.reset_high_current()
                self.publish_velocity()

                if (not self.move_with_odom(self.cfg.min_linear_vel, self.cfg.max_linear_vel, -0.5) and
                    not self.pre_dock(self.robot_state.MODE_CHARGE, 57)):
                    rospy.logerr("AutodockController: [Steer to charger]: Can't execute after recovery!")
                    return False
                continue
            
            else:
                dock_tf = self.get_tf(self.cfg.charger_link)
                if (dock_tf is None):
                    rospy.logerr("AutodockController: Can not detect dock frame: %s", self.cfg.charger_link)
                    # dis_move = distance - 0.26
                    # if (dis_move > 0):
                    #     rospy.logwarn(f"[Steer to charger]: Move with odom {distance}m!")
                    #     return (self.move_with_odom(0.02, 0.035, dis_move + self.cfg.stop_trans_diff))
                    # else:
                    #     return True
                    return False
                dock_pose = utils.get_2d_pose(dock_tf)
                x, y, yaw = utils.flip_base_frame(dock_pose)

                distance = math.sqrt(pow(x,2) + pow(y,2))

                if (abs(x) - 0.4 < 0):
                    return True
                
                time_now = rospy.Time.now()
                dt = (time_now - prev_time).to_sec()
                angle = _pid.update(0,y,dt)

                v = max_vel_x

                if abs(x) < 0.5: v = min_vel_x

                w = angle
                
                self.publish_velocity(v, w)
                prev_time = time_now
            self.rate.sleep()
    

    def lastmile_with_dropoff(self) -> bool:
        while not rospy.is_shutdown():
            if self.check_cancel():
                return False
            
            elif self.do_pause():
                pass
            
            elif (self.high_motor_drop_current or
                self.high_motor_pickup_current):
                self.reset_high_current()
                self.publish_velocity()
                self.brake(True)
                return True

            else:
                self.publish_velocity(-self.cfg.max_x_lastmile)
            
            self.rate.sleep()


    def lastmile_with_pickup(self, state, dock_name) -> bool:
        flag = False
        flag_1 = False
        counter = 0     # How many times for retry_with_high_current
        count_lost_dock = 0
        allow_count = True
        start_time = 0.0

        while not rospy.is_shutdown():
            if self.check_cancel():
                return False
            
            elif self.do_pause():
                pass

            elif (self.high_motor_drop_current or
                  self.high_motor_pickup_current):
                if (self.cart_sensor_state == (1,1)
                    or self.cart_sensor_state == (1,0)
                    or self.cart_sensor_state == (0,1)):

                    v = sign*self.cfg.min_x_pid_lastmile
                    
                    if allow_count:
                        start_time = rospy.Time.now()
                        allow_count = False
                    
                    if (self.cart_sensor_state == (1,1)
                        or (rospy.Time.now() - start_time).to_sec() > 2.0):
                        self.reset_high_current()
                        self.brake(True)
                        break
                    elif self.cart_sensor_state == (1,0):
                        w = -0.02
                    elif self.cart_sensor_state == (0,1):
                        w = 0.02
                    self.publish_velocity(v, w)
                    self.rate.sleep()
                    continue

                self.reset_high_current()
                self.publish_velocity()
                self.update_line_extraction_params(0)
                flag = False
                flag_1 = False

                if (not self.retry_if_high_current(0.4, counter, 5) and
                    not self.pre_dock(state, self.first_name, check_dock_frame=True) and
                    not self.steer_dock(state, self.first_name)):
                    return False
                
                counter += 1         

            else:
                if (not flag_1):
                    self.update_line_extraction_params(1)
                    flag_1 = True

                dock_tf = self.get_tf(dock_name)

                if (dock_tf is None):
                    if count_lost_dock <= 5:
                        rospy.logwarn(f"AutodockController: Can not get {dock_name}!, will retry find dock: {count_lost_dock} retry")
                        count_lost_dock += 1
                        self.publish_velocity(-self.cfg.min_x_pid_lastmile,0)
                        continue
                    else:
                        rospy.logerr(f"AutodockController: Maximum retry find dock frame: {dock_name}")
                        return False
                else:
                    count_lost_dock = 0
                
                dock_pose = utils.get_2d_pose(dock_tf)
                x, y, yaw = dock_pose

                sign = 1 if x > 0 else -1

                if not flag:
                    angle = self.PIDController(abs(y))
                    v = sign*self.cfg.max_x_pid_lastmile

                    if (y > self.cfg.y_tolerance_pid):
                        w = sign*angle

                    elif (y < -self.cfg.y_tolerance_pid):
                        w = -sign*angle

                    else:
                        w = 0 
                
                if (self.cart_sensor_state == (1,1)
                    or self.cart_sensor_state == (1,0)
                    or self.cart_sensor_state == (0,1)):

                    flag = True
                    v = sign*self.cfg.min_x_pid_lastmile

                    if self.cart_sensor_state == (1,1):
                        self.brake(True)
                        break
                    elif self.cart_sensor_state == (1,0):
                        w = -0.02
                    elif self.cart_sensor_state == (0,1):
                        w = 0.02
                
                # Constrain angle
                if abs(w) > 0.15:
                    w = 0.0
                
                self.publish_velocity(v, w)
            self.rate.sleep()
        return True
    
    
    #============> MAIN RUN <============#

    def pre_dock(self, state, dock_name, check_dock_frame=False) -> bool:
        self.set_state(DockState.PREDOCK, "Running!")
        
        pose_list = []
        check_yaw = False
        check_y_counter = 0
        check_yaw_counter = 0

        if state == self.robot_state.MODE_CHARGE:
            self.turn_off_front_safety(True)
            self.turn_off_ultrasonic_safety(True)

        while not rospy.is_shutdown():
            if self.check_cancel():
                return False
            
            elif self.do_pause():
                pass
            
            else:
                if (state == self.robot_state.MODE_CHARGE):
                    dock_tf = self.get_tf(self.cfg.charger_link)
                    if dock_tf is None:
                        rospy.logerr("AutodockController: Can not detect dock frame: %s", self.cfg.charger_link)
                    
                    dock_pose = utils.get_2d_pose(dock_tf)
                    if len(pose_list) < self.cfg.predock_tf_samples:
                        pose_list.append(dock_pose)
                        continue

                    avg_pose = utils.avg_2d_poses(pose_list)
                    pose_list = []

                    x, y, yaw = utils.flip_base_frame(avg_pose)

                    # Check yaw
                    # if (not check_yaw and 
                    #    (abs(yaw) > self.cfg.yaw_predock_tolerance)):
                    if ((check_yaw_counter < 2) and 
                        (abs(yaw) > self.cfg.yaw_predock_tolerance)):
                        if not self.rotate_with_odom(self.cfg.min_angular_vel, self.cfg.max_angular_vel, yaw):
                            return False
                        # check_yaw = True
                        check_yaw_counter += 1
                        continue
                    
                    if ((check_y_counter < 3) and
                        (abs(y) > self.cfg.max_parallel_offset)):
                            if (not self.auto_correction(x, y, 0.2, self.cfg.front_laser_offset) and
                                not self.correct_robot(y,x)):
                                return False
                            check_y_counter += 1
                            check_yaw_counter = 0
                            self.set_state(DockState.PREDOCK, "")
                            continue
                else:
                    if ((check_y_counter == 1 or check_dock_frame)
                        and dock_name != self.cfg.parallel_frame):
                        laser_tf = self.get_tf(self.cfg.first_frame)
                        tag_tf = self.get_tf(self.tag_frame)
                        if (laser_tf is not None and tag_tf is not None):
                            if self.check_laser_frame(laser_tf=laser_tf, tag_tf=tag_tf):
                                dock_tf = laser_tf
                                self.first_name = self.cfg.first_frame
                            else:
                                dock_tf = tag_tf
                                self.first_name = self.tag_frame
                        elif tag_tf is None and laser_tf is None:
                            return False
                        else:
                            if laser_tf is not None:
                                dock_tf = laser_tf
                                self.first_name = self.cfg.first_frame
                            else:
                                dock_tf = tag_tf
                                self.first_name = self.tag_frame    
                    else:
                        dock_tf = self.get_tf(dock_name)
                    
                    if dock_tf is None:
                        rospy.logerr(f"AutodockController: Can not detect dock name: {dock_name}!")
                        return False
        
                    dock_pose = utils.get_2d_pose(dock_tf)

                    if len(pose_list) < self.cfg.predock_tf_samples:
                        pose_list.append(dock_pose)
                        continue

                    avg_pose = utils.avg_2d_poses(pose_list)
                    pose_list = []

                    x, y, yaw = avg_pose

                    # Check yaw
                    # if not check_yaw:
                    if check_yaw_counter < 2:
                        if (abs(yaw) > self.cfg.yaw_predock_tolerance):
                            if not self.rotate_with_odom(self.cfg.min_angular_vel, self.cfg.max_angular_vel, yaw): return False
                            # check_yaw = True
                            check_yaw_counter += 1
                            continue

                    # Check y
                    if check_y_counter <= 2:
                        if abs(y) > self.cfg.max_parallel_offset:
                            if (state == self.robot_state.MODE_PICKUP):
                                if (not self.auto_correction(x, y, 0.25, self.cfg.back_laser_offset) and
                                    not self.correct_robot(y, x)):
                                    return False
                            
                                check_y_counter += 1
                                check_yaw_counter = 0
                                self.update_polygon_params(1)
                                continue

                            elif (state == self.robot_state.MODE_DROPOFF):
                                rotate_orientation = 2 if (self.dock_name == 7) else 0

                                if not self.correct_robot(y, x, rotate_orientation=rotate_orientation):
                                    return False
                            
                            check_y_counter += 1
                            check_yaw_counter = 0
                            continue

                        elif (state == self.robot_state.MODE_PICKUP):
                            self.update_polygon_params(1)
                        
                self.printout_success("Completed!")
                return True
            
            self.rate.sleep()     
        exit(0)


    def steer_dock(self, state, dock_name) -> bool:
        self.set_state(DockState.STEER_DOCK, "Running!")

        if (state == self.robot_state.MODE_CHARGE):
            self.turn_off_front_safety(True)
            if not self.steer_to_charger(self.cfg.max_x_pid_steer,
                                       self.cfg.min_x_pid_steer):
                return False
        else:
            self.turn_off_back_safety(True)
            if (state == self.robot_state.MODE_PICKUP):
                if not self.steer_dock_with_pickup(state, dock_name,
                                                self.cfg.max_x_pid_steer, self.cfg.min_x_pid_steer):
                    return False
            else:
                if not self.steer_dock_with_dropoff(state, dock_name,
                                                self.cfg.max_x_pid_steer, self.cfg.min_x_pid_steer):
                    return False

        self.printout_success("Completed!")
        return True
    

    def lastmile_dock(self, state, dock_name) -> bool:
        self.set_state(DockState.LAST_MILE, "Running!")
        
        if (state == self.robot_state.MODE_PICKUP):
            if not self.lastmile_with_pickup(state, dock_name):
                return False
        elif (state == self.robot_state.MODE_DROPOFF):
            if not self.lastmile_with_dropoff():
                return False
        elif (state == self.robot_state.MODE_CHARGE):
            if not self.move_with_odom(0.02, 0.03, 0.14):
                return False
            
        self.printout_success("Completed!")
        return True


    def cmd_slider(self, state, timeout=30.0) -> bool:
        if (state == self.robot_state.MODE_PICKUP):
            self.set_state(DockState.SLIDER_GO_OUT, "Running!")
            cmd_slider = 1
            sensor_order = 1
            sensor_check = 0
            self.enable_apriltag_detector(False)
        elif (state == self.robot_state.MODE_DROPOFF):
            self.set_state(DockState.SLIDER_GO_IN, "Running!")
            cmd_slider = 2
            sensor_order = 0
            sensor_check = 1
        elif (state == self.robot_state.MODE_CHARGE):
            return True
        
        if not self.check_slider_state(cmd_slider, sensor_order, sensor_check, timeout):
            return False
        
        self.brake(False)
        
        self.printout_success("Completed!")
        return True


    def go_out_dock(self, state):
        if (state == self.robot_state.MODE_PICKUP):
            s = 0.8
        elif (state == self.robot_state.MODE_DROPOFF):
            if (self.dock_name == 7 or self.dock_name == 8):
                s = 1.0
            else:
                s = 0.6
        elif (state == self.robot_state.MODE_CHARGE):
            return True

        self.set_state(DockState.GO_OUT_DOCK, f"Move robot {s}m go out dock!")

        if not self.move_with_odom(self.cfg.min_linear_vel,self.cfg.max_linear_vel, s):
            return False
        if (self.dock_name == 7):
            self.turn_off_back_safety(False)
            if (not self.rotate_with_odom(self.cfg.min_angular_vel, self.cfg.max_angular_vel, -92*math.pi/180)
                or not self.move_with_odom(self.cfg.min_linear_vel, self.cfg.max_linear_vel, -0.5)
                or not self.rotate_with_odom(self.cfg.min_angular_vel, self.cfg.max_angular_vel, 92*math.pi/180)):
                return False
        
        self.printout_success("Completed!")
        return True 


if __name__ == "__main__":
    config = AutodockConfig()

    node = AutoDockStateMachine(
        config,
        run_server=True,
        load_rosparam=True,
        fake_clock=False)
    rospy.spin()
