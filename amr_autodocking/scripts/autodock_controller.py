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

from amr_autodocking.autodock_server import AutodockConfig, AutoDockServer, AutodockConst
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
        

    def start(self, mode, dock_name, tag_ids, angle_to_dock, correction_angle, rotate_type, distance_go_out) -> bool:
        rospy.loginfo(f"AutodockController: Start autodock! Will attempt with {self.cfg.retry_count} retry!")
        
        if self.cfg.debug_mode:
            print(f"Autodock params:")
            print(f"* dock_name: {dock_name}")
            print(f"* mode: {mode}")
            print(f"* tag_ids: {list(tag_ids)}")
            print(f"* angle_to_dock: {angle_to_dock}")
            print(f"* correction_angle: {correction_angle}")
            print(f"* rotate_type: {rotate_type}")
            print(f"* distance_go_out: {distance_go_out}")

        if not self.rotate_to_dock(angle_to_dock):
            return False

        # Reset some needed values when start docking
        self.reset()

        first_frame = ""
        if (mode == self.mode.MODE_CHARGE):
            first_frame = self.cfg.charger_frame
            self.enable_line_detector("front", True)
            self.turn_off_front_safety(True)
            self.turn_off_ultrasonic_safety(True)
        else:
            self.enable_line_detector('back', True)
            
            if (mode == self.mode.MODE_PICKUP):
                self.update_line_extraction_params()
                self.enable_apriltag_detector(True)
                
                # (TODO) - Sometimes tag is delayed in few miliseconds,
                #        - so waiting for tag with counter.
                wait_for_util_tag_appear = 0
                while True:
                    try:
                        tag_detections = rospy.wait_for_message("/back_camera/tag_detections",
                                                              AprilTagDetectionArray, timeout=1.0)
                        tag_frame = ""
                        if tag_detections is not None:
                            tags = tag_detections.detections
                            if len(list(tag_ids)) != 0:
                                for tag in tags:
                                    if tag.id[0] in list(tag_ids):
                                        tag_frame = f"tag_frame_{tag.id[0]}"
                                self.tag_frame = tag_frame
                            else:
                                rospy.logerr("No tag_ids recieved!")
                                return False

                        if tag_frame != "":
                            if (self.check_dock_frame(self.cfg.first_frame, tag_frame)):
                                first_frame = self.cfg.first_frame
                            else:
                                first_frame = tag_frame
                        else:
                            if wait_for_util_tag_appear == 5:
                                if self.get_tf(self.cfg.first_frame) is None:
                                    self.enable_apriltag_detector(False)
                                    self.enable_line_detector("back", False)
                                    return False
                                else:
                                    first_frame = self.cfg.first_frame
                                    break
                            wait_for_util_tag_appear += 1
                            rospy.sleep(0.5)
                            continue
                        break
                    except Exception as e:
                        print(e)
                        return False
            else:
                first_frame = self.cfg.parallel_frame
                self.update_line_extraction_params(AutodockConst.DROPOFF)
                self.update_polygon_params(AutodockConst.DROPOFF)

        last_frame = self.cfg.last_frame

        while(True):
            if(
                self.pre_dock(mode, first_frame, correction_angle, rotate_type) and
                self.steer_dock(mode, first_frame, correction_angle, rotate_type) and
                self.lastmile_dock(mode, last_frame, correction_angle, rotate_type) and
                self.cmd_slider(mode) and
                self.go_out_dock(mode, dock_name, distance_go_out)
            ):
                if (mode == self.mode.MODE_CHARGE):
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
            
            if first_frame == self.cfg.parallel_frame:
                if self.retry(first_frame):
                    first_frame = self.cfg.parallel_frame
                else: break
            else:
                if (self.retry(self.tag_frame)):
                    first_frame = self.tag_frame
                elif (self.retry(self.cfg.first_frame)):
                    first_frame = self.cfg.first_frame
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
                
    
    def steer_dock_with_pickup(self, mode, dock_frame, correction_angle, rotate_type, max_x, min_x):
        counter = 0     # How many times for retry_with_high_current

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
                    not self.pre_dock(mode, dock_frame, correction_angle, rotate_type)):
                    return False

                counter += 1
            
            else:
                if self.check_dock_frame(self.cfg.first_frame, self.tag_frame):
                    dock_frame = self.cfg.first_frame
                else:
                    dock_frame = self.tag_frame

                dock_tf = self.get_tf(dock_frame, transform_timeout=0.3)
                
                if dock_tf is None:
                    return True

                dock_pose = utils.get_2d_pose(dock_tf)
                
                x, y, yaw = dock_pose

                sign = 1 if x > 0 else -1

                if dock_frame == self.cfg.first_frame:
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
                
                w = utils.clamp(w, -0.15, 0.15)
                
                self.publish_velocity(v, w)
            self.rate.sleep()
    
    
    def steer_dock_with_dropoff(self, mode, dock_frame, correction_angle, rotate_type, max_x, min_x):
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
                self.update_line_extraction_params(AutodockConst.DROPOFF)

                if (not self.retry_if_high_current(0.4, counter, 5) and
                    not self.pre_dock(mode, dock_frame, correction_angle, rotate_type)):
                    return False
        
                counter += 1
            else:
                # Check whether back laser in dock (Depend on distance from center of laser to both side of the dock)
                if (self.left_range < 0.3 and self.right_range < 0.3):
                    if not flag:
                        self.update_line_extraction_params(AutodockConst.PICKUP)
                        start_time = rospy.Time.now()
                        flag = True
                        rospy.loginfo("AutodockController: BackLaser is in dropoff dock!")
                    
                    if (rospy.Time.now() - start_time).to_sec() > 4.5:
                        if not self.high_motor_drop_current:
                            return True
                        
                elif (flag): 
                    rospy.logwarn("AutodockController: BackLaser is out dropoff dock, it's wrong. Please check!")
                    flag = False    # Reset flag for calculate total time

                dock_tf = self.get_tf(dock_frame)

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
                w = utils.clamp(w, -0.3, 0.3)
                
                self.publish_velocity(v, w)
            self.rate.sleep()

    def steer_to_charger(self, mode, dock_frame, correction_angle, rotate_type, max_vel_x, min_vel_x):
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
                    not self.pre_dock(mode, dock_frame, correction_angle, rotate_type)):
                    rospy.logerr("AutodockController: [Steer to charger]: Can't execute after recovery!")
                    return False
                continue
            
            else:
                dock_tf = self.get_tf(dock_frame)
                if (dock_tf is None):
                    rospy.logerr("AutodockController: Can not detect dock frame: %s", dock_frame)
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


    def lastmile_with_pickup(self, mode, last_frame, correction_angle, rotate_type) -> bool:
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
                self.update_line_extraction_params()
                flag = False
                flag_1 = False

                if not self.retry_if_high_current(0.4, counter, 5):
                    return False
                
                if self.check_dock_frame(self.cfg.first_frame, self.tag_frame):
                    dock_frame = self.cfg.first_frame
                else:
                    dock_frame = self.tag_frame

                if (not self.pre_dock(mode, dock_frame, correction_angle, rotate_type) and
                    not self.steer_dock(mode, dock_frame, correction_angle, rotate_type)):
                    return False
                
                counter += 1         

            else:
                if (not flag_1):
                    self.update_line_extraction_params(AutodockConst.PICKUP)
                    flag_1 = True

                dock_tf = self.get_tf(last_frame)

                if (dock_tf is None):
                    if count_lost_dock <= 5:
                        rospy.logwarn(f"AutodockController: Can not get {last_frame}!, will retry find dock: {count_lost_dock} retry")
                        count_lost_dock += 1
                        self.publish_velocity(-self.cfg.min_x_pid_lastmile,0)
                        continue
                    else:
                        rospy.logerr(f"AutodockController: Maximum retry find dock frame: {last_frame}")
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
                w = utils.clamp(w, -0.1, 0.1)
                
                self.publish_velocity(v, w)
            self.rate.sleep()
        return True
    
    
    #============> MAIN RUN <============#

    def pre_dock(self, mode, dock_frame, correction_angle, rotate_type) -> bool:
        self.set_state(DockState.PREDOCK, "Running!")
        
        pose_list = []
        check_y_counter = 0
        check_yaw_counter = 0

        while not rospy.is_shutdown():
            if self.check_cancel():
                return False
            
            elif self.do_pause():
                pass
            
            else:
                if (mode == self.mode.MODE_CHARGE):
                    dock_tf = self.get_tf(dock_frame)
                    if dock_tf is None:
                        rospy.logerr("AutodockController: Can not detect dock frame: %s", dock_frame)
                    
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
                        if not self.rotate_with_odom(yaw):
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
                    if mode != self.mode.MODE_DROPOFF:
                        if self.check_dock_frame(self.cfg.first_frame, self.tag_frame):
                            dock_frame = self.cfg.first_frame
                        else:
                            dock_frame = self.tag_frame

                    dock_tf = self.get_tf(dock_frame)
                    
                    if dock_tf is None:
                        rospy.logerr(f"AutodockController: Can not detect dock name: {dock_frame}!")
                        return False        
        
                    dock_pose = utils.get_2d_pose(dock_tf)

                    if len(pose_list) < self.cfg.predock_tf_samples:
                        pose_list.append(dock_pose)
                        continue

                    avg_pose = utils.avg_2d_poses(pose_list)
                    pose_list = []

                    x, y, yaw = avg_pose
                    yaw = utils.clamp(yaw, -0.26, 0.26)

                    # Check yaw
                    # if not check_yaw:
                    if check_yaw_counter < 2:
                        if (abs(yaw) > self.cfg.yaw_predock_tolerance):
                            if not self.rotate_with_odom(yaw): return False
                            # check_yaw = True
                            check_yaw_counter += 1
                            continue

                    # Check y
                    if check_y_counter <= 2:
                        if abs(y) > self.cfg.max_parallel_offset:
                            if (mode == self.mode.MODE_PICKUP):
                                if (correction_angle == 0):
                                    if (not self.auto_correction(x, y, 0.25, self.cfg.back_laser_offset) and
                                        not self.correct_robot(y, x)):
                                        return False
                                else:
                                    if not self.correct_robot(y, x, correcttion_angle=correction_angle):
                                        return False
                            
                                check_y_counter += 1
                                check_yaw_counter = 0
                                self.update_polygon_params(AutodockConst.PICKUP)
                                continue

                            elif (mode == self.mode.MODE_DROPOFF):
                                if not self.correct_robot(y, x,
                                                          correcttion_angle=correction_angle,
                                                          rotate_type=rotate_type):
                                    return False
                            
                            check_y_counter += 1
                            check_yaw_counter = 0
                            continue

                        elif (mode == self.mode.MODE_PICKUP):
                            self.update_polygon_params(AutodockConst.PICKUP)
                        
                self.printout_success("Completed!")
                return True
            
            self.rate.sleep()     
        exit(0)


    def steer_dock(self, mode, dock_frame, correction_angle, rotate_type) -> bool:
        self.set_state(DockState.STEER_DOCK, "Running!")

        if (mode == self.mode.MODE_CHARGE):
            self.turn_off_front_safety(True)
            if not self.steer_to_charger(mode, dock_frame, correction_angle, rotate_type,
                                         self.cfg.max_x_pid_steer, self.cfg.min_x_pid_steer):
                return False
        else:
            self.turn_off_back_safety(True)
            if (mode == self.mode.MODE_PICKUP):
                if not self.steer_dock_with_pickup(mode, dock_frame, correction_angle, rotate_type,
                                                   self.cfg.max_x_pid_steer, self.cfg.min_x_pid_steer):
                    return False
            else:
                if not self.steer_dock_with_dropoff(mode, dock_frame, correction_angle, rotate_type,
                                                self.cfg.max_x_pid_steer, self.cfg.min_x_pid_steer):
                    return False

        self.printout_success("Completed!")
        return True
    

    def lastmile_dock(self, mode, dock_frame, correction_angle, rotate_type) -> bool:
        self.set_state(DockState.LAST_MILE, "Running!")
        
        if (mode == self.mode.MODE_PICKUP):
            if not self.lastmile_with_pickup(mode, dock_frame, correction_angle, rotate_type):
                return False
        elif (mode == self.mode.MODE_DROPOFF):
            if not self.lastmile_with_dropoff():
                return False
        elif (mode == self.mode.MODE_CHARGE):
            if not self.move_with_odom(0.04, 0.055, 0.155):
                return False
            
        self.printout_success("Completed!")
        return True


    def cmd_slider(self, state, timeout=30.0) -> bool:
        if (state == self.mode.MODE_PICKUP):
            self.set_state(DockState.SLIDER_GO_OUT, "Running!")
            cmd_slider = 1
            sensor_order = 1
            sensor_check = 0
            self.enable_apriltag_detector(False)
        elif (state == self.mode.MODE_DROPOFF):
            self.set_state(DockState.SLIDER_GO_IN, "Running!")
            cmd_slider = 2
            sensor_order = 0
            sensor_check = 1
        elif (state == self.mode.MODE_CHARGE):
            return True
        
        if not self.check_slider_state(cmd_slider, sensor_order, sensor_check, timeout):
            return False
        
        self.brake(False)
        
        self.printout_success("Completed!")
        return True


    def go_out_dock(self, mode, dock_name, distance_go_out):
        if (mode == self.mode.MODE_CHARGE):
            return True

        self.set_state(DockState.GO_OUT_DOCK, f"Move robot {distance_go_out}m go out dock!")

        if not self.move_with_odom(self.cfg.min_linear_vel,self.cfg.max_linear_vel, distance_go_out):
            return False
        
        # Custom dock
        if not self.custom_rotation_after_undock(dock_name):
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
