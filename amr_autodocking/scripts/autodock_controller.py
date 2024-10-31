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
        self.resetVariable()

        if (self.dock_name == 57):
            self.enableLineDetector("front", True)
            state = self.robot_state.MODE_CHARGE
        else:
            self.enableLineDetector('back', True)
            
            if (self.dock_name == 1 or self.dock_name == 2 or
                self.dock_name == 5 or self.dock_name == 6):
                state = self.robot_state.MODE_PICKUP
                self.updateLineExtractionParams(0)
                self.enableApriltag(True)

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
                        self.checkLaserFrame(laser_tf_name=self.cfg.first_frame, tag_tf_name=tag_name)):
                        self.first_name = self.cfg.first_frame
                    else:
                        self.first_name = tag_name
                elif tag_name == "" and self.get_tf(self.cfg.first_frame) is None:
                    self.enableApriltag(False)
                    self.enableLineDetector(False)
                    return False
                else:
                    self.first_name = self.cfg.first_frame if self.get_tf(self.cfg.first_frame) is not None else tag_name
            else:
                state = self.robot_state.MODE_DROPOFF
                self.first_name = self.cfg.parallel_frame
                self.updateLineExtractionParams(2)
                self.updatePolygonParams(2)

        last_frame = self.cfg.last_frame

        while(True):
            if(
                self.preDock(state, self.first_name) and
                self.steerDock(state, self.first_name) and
                self.lastmileDock(state, last_frame) and
                self.cmdSliderMotor(state) and
                self.goOutDock(state)
            ):
                if (state == self.robot_state.MODE_CHARGE):
                    self.enableLineDetector("front", False)
                    self.setSpeed()
                else:
                    self.setSpeed()
                    self.turnOffBackLaserSafety(False)
                    self.enableLineDetector('back', False)
                self.setState(DockState.IDLE, "Autodock completed!")
                return True

            # If Dock failed
            self.setSpeed()
            self.printOutError("Error!")
            self.setState(DockState.ERROR, "Autodock failed!")

            if (self.dock_state == DockState.SLIDER_GO_IN or
                self.dock_state == DockState.SLIDER_GO_OUT or
                self.dock_state == DockState.GO_OUT_DOCK):
                break    

            # check again if it failed because of canceled
            if self.checkCancel():
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

        self.setSpeed()
        self.turnOffBackLaserSafety(False)
        self.turnOffFrontLaserSafety(False)
        self.turnOffUltrasonicSafety(False)
        self.enableLineDetector('back', False)
        self.enableApriltag(False)
        return False
    

    def checkSliderState(self, slider_cmd: int, sensor_order:int,
                         sensor_check: int, timeout: float) -> bool:
        """
        `sensor_order = 1`: Max position sensor
        `sensor_order = 0`: Original position sensor
        `timeout`: Max time for slider motor reachs original position or max position
        """
        start_time = rospy.Time.now()
        check_pause = False
        timeout_checksensor = 5.0

        self.sliderCommand(slider_cmd)
        while True:
            if self.checkCancel():
                return False
            
            elif self.doPause():
                if not check_pause:
                    timeout_used = (rospy.Time.now() - start_time).to_sec() 
                    timeout -= timeout_used
                    timeout_checksensor -= timeout_used
                    check_pause = True
            else:
                if check_pause:
                    self.sliderCommand(slider_cmd)
                    start_time = rospy.Time.now()
                    check_pause = False

                elif ((rospy.Time.now() - start_time).to_sec() >= timeout):
                    rospy.logerr("AutodockController: Slider motor error: Exceed timeout 15s")
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
                
    
    def steerDockWithPickup(self, state, dock_name, max_x, min_x):
        """
        Move straight robot to goal using PID controller
        `max_x`: Max linear velocity
        `min_x`: Min linear velocity
        `tf_name`: Dock tf name
        """
        counter = 0     # How many times for retry_with_high_current
        flag = False

        while not rospy.is_shutdown():
            if self.checkCancel():
                return False
            
            elif self.doPause():
                pass
            
            elif (self.high_motor_drop_current or
                self.high_motor_pickup_current):
                self.resetHighCurrent()
                self.setSpeed()

                if (not self.retryWithHighMotorCurrent(0.4, counter, 5) and
                    not self.preDock(state, dock_name, check_dock_frame=True)):
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
                
                self.setSpeed(v, w)
            self.rate.sleep()
    
    
    def steerDockWithDropOff(self, state, dock_name, max_x, min_x):
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
            if self.checkCancel():
                return False
            
            elif self.doPause():
                pass

            elif (self.high_motor_drop_current or
                  self.high_motor_pickup_current):
                if not check_high_current:
                    check_timer = rospy.Time.now()
                    check_high_current = True
                if ((rospy.Time.now() - check_timer).to_sec() <= 3.0):
                    self.resetHighCurrent()
                    continue
                self.resetHighCurrent()
                flag = False
                self.setSpeed()
                self.updateLineExtractionParams(2)

                if (not self.retryWithHighMotorCurrent(0.4, counter, 5) and
                    not self.preDock(state, dock_name)):
                    return False
        
                counter += 1
            else:
                # Check whether back laser in dock (Depend on distance from center of laser to both side of the dock)
                if (self.left_range < 0.3 and self.right_range < 0.3):
                    if not flag:
                        self.updateLineExtractionParams(1)
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
                
                self.setSpeed(v, w)
            self.rate.sleep()

    def steerToCharger(self, max_vel_x, min_vel_x):
        _pid = PID(self.cfg.k_p_steer, self.cfg.k_i_steer, self.cfg.k_d_steer)
        prev_time = rospy.Time.now()

        while not rospy.is_shutdown():
            if self.checkCancel():
                return False
            
            elif self.doPause():
                pass
            
            elif (self.high_motor_drop_current or
                self.high_motor_pickup_current):
                self.resetHighCurrent()
                self.setSpeed()

                if (not self.moveWithOdom(self.cfg.min_linear_vel, self.cfg.max_linear_vel, -0.5) and
                    not self.preDock(self.robot_state.MODE_CHARGE, 57)):
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
                    #     return (self.moveWithOdom(0.02, 0.035, dis_move + self.cfg.stop_trans_diff))
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
                
                self.setSpeed(v, w)
                prev_time = time_now
            self.rate.sleep()
    

    def lastmileDockWithDropOff(self) -> bool:
        while not rospy.is_shutdown():
            if self.checkCancel():
                return False
            
            elif self.doPause():
                pass
            
            elif (self.high_motor_drop_current or
                self.high_motor_pickup_current):
                self.resetHighCurrent()
                self.setSpeed()
                self.turnOnBrake(True)
                return True

            else:
                self.setSpeed(-self.cfg.max_x_lastmile)
            
            self.rate.sleep()


    def lastmileWithPickUpOrder(self, state, dock_name) -> bool:
        flag = False
        flag_1 = False
        counter = 0     # How many times for retry_with_high_current
        count_lost_dock = 0
        allow_count = True
        start_time = 0.0

        while not rospy.is_shutdown():
            if self.checkCancel():
                return False
            
            elif self.doPause():
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
                        self.resetHighCurrent()
                        self.turnOnBrake(True)
                        break
                    elif self.cart_sensor_state == (1,0):
                        w = -0.02
                    elif self.cart_sensor_state == (0,1):
                        w = 0.02
                    self.setSpeed(v, w)
                    self.rate.sleep()
                    continue

                self.resetHighCurrent()
                self.setSpeed()
                self.updateLineExtractionParams(0)
                flag = False
                flag_1 = False

                if (not self.retryWithHighMotorCurrent(0.4, counter, 5) and
                    not self.preDock(state, self.first_name, check_dock_frame=True) and
                    not self.steerDock(state, self.first_name)):
                    return False
                
                counter += 1         

            else:
                if (not flag_1):
                    self.updateLineExtractionParams(1)
                    flag_1 = True

                dock_tf = self.get_tf(dock_name)

                if (dock_tf is None):
                    if count_lost_dock <= 5:
                        rospy.logwarn(f"AutodockController: Can not get {dock_name}!, will retry find dock: {count_lost_dock} retry")
                        count_lost_dock += 1
                        self.setSpeed(-self.cfg.min_x_pid_lastmile,0)
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
                        self.turnOnBrake(True)
                        break
                    elif self.cart_sensor_state == (1,0):
                        w = -0.02
                    elif self.cart_sensor_state == (0,1):
                        w = 0.02
                
                # Constrain angle
                if abs(w) > 0.15:
                    w = 0.0
                
                self.setSpeed(v, w)
            self.rate.sleep()
        return True
    
    
    #============> MAIN RUN <============#

    def preDock(self, state, dock_name, check_dock_frame=False) -> bool:
        self.setState(DockState.PREDOCK, "Running!")
        
        pose_list = []
        check_yaw = False
        check_y_counter = 0
        check_yaw_counter = 0

        if state == self.robot_state.MODE_CHARGE:
            self.turnOffFrontLaserSafety(True)
            self.turnOffUltrasonicSafety(True)

        while not rospy.is_shutdown():
            if self.checkCancel():
                return False
            
            elif self.doPause():
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
                        if not self.rotateWithOdom(self.cfg.min_angular_vel, self.cfg.max_angular_vel, yaw):
                            return False
                        # check_yaw = True
                        check_yaw_counter += 1
                        continue
                    
                    if ((check_y_counter < 3) and
                        (abs(y) > self.cfg.max_parallel_offset)):
                            if (not self.checkDistanceAndCorrection(x, y, 0.2, self.cfg.front_laser_offset) and
                                not self.correct_robot(y,x)):
                                return False
                            check_y_counter += 1
                            check_yaw_counter = 0
                            self.setState(DockState.PREDOCK, "")
                            continue
                else:
                    if ((check_y_counter == 1 or check_dock_frame)
                        and dock_name != self.cfg.parallel_frame):
                        laser_tf = self.get_tf(self.cfg.first_frame)
                        tag_tf = self.get_tf(self.tag_frame)
                        if (laser_tf is not None and tag_tf is not None):
                            if self.checkLaserFrame(laser_tf=laser_tf, tag_tf=tag_tf):
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
                            if not self.rotateWithOdom(self.cfg.min_angular_vel, self.cfg.max_angular_vel, yaw): return False
                            # check_yaw = True
                            check_yaw_counter += 1
                            continue

                    # Check y
                    if check_y_counter <= 2:
                        if abs(y) > self.cfg.max_parallel_offset:
                            if (state == self.robot_state.MODE_PICKUP):
                                if (not self.checkDistanceAndCorrection(x, y, 0.25, self.cfg.back_laser_offset) and
                                    not self.correct_robot(y, x)):
                                    return False
                            
                                check_y_counter += 1
                                check_yaw_counter = 0
                                self.updatePolygonParams(1)
                                continue

                            elif (state == self.robot_state.MODE_DROPOFF):
                                rotate_orientation = 2 if (self.dock_name == 7) else 0

                                if not self.correct_robot(y, x, rotate_orientation=rotate_orientation):
                                    return False
                            
                            check_y_counter += 1
                            check_yaw_counter = 0
                            continue

                        elif (state == self.robot_state.MODE_PICKUP):
                            self.updatePolygonParams(1)
                        
                self.printOutSuccess("Completed!")
                return True
            
            self.rate.sleep()     
        exit(0)


    def steerDock(self, state, dock_name) -> bool:
        self.setState(DockState.STEER_DOCK, "Running!")

        if (state == self.robot_state.MODE_CHARGE):
            self.turnOffFrontLaserSafety(True)
            if not self.steerToCharger(self.cfg.max_x_pid_steer,
                                       self.cfg.min_x_pid_steer):
                return False
        else:
            self.turnOffBackLaserSafety(True)
            if (state == self.robot_state.MODE_PICKUP):
                if not self.steerDockWithPickup(state, dock_name,
                                                self.cfg.max_x_pid_steer, self.cfg.min_x_pid_steer):
                    return False
            else:
                if not self.steerDockWithDropOff(state, dock_name,
                                                self.cfg.max_x_pid_steer, self.cfg.min_x_pid_steer):
                    return False

        self.printOutSuccess("Completed!")
        return True
    

    def lastmileDock(self, state, dock_name) -> bool:
        self.setState(DockState.LAST_MILE, "Running!")
        
        if (state == self.robot_state.MODE_PICKUP):
            if not self.lastmileWithPickUpOrder(state, dock_name):
                return False
        elif (state == self.robot_state.MODE_DROPOFF):
            if not self.lastmileDockWithDropOff():
                return False
        elif (state == self.robot_state.MODE_CHARGE):
            if not self.moveWithOdom(0.02, 0.03, 0.14):
                return False
            
        self.printOutSuccess("Completed!")
        return True


    def cmdSliderMotor(self, state, timeout=25.0) -> bool:
        if (state == self.robot_state.MODE_PICKUP):
            self.setState(DockState.SLIDER_GO_OUT, "Running!")
            cmd_slider = 1
            sensor_order = 1
            sensor_check = 0
            self.enableApriltag(False)
        elif (state == self.robot_state.MODE_DROPOFF):
            self.setState(DockState.SLIDER_GO_IN, "Running!")
            cmd_slider = 2
            sensor_order = 0
            sensor_check = 1
        elif (state == self.robot_state.MODE_CHARGE):
            return True
        
        if not self.checkSliderState(cmd_slider, sensor_order, sensor_check, timeout):
            return False
        
        self.turnOnBrake(False)
        
        self.printOutSuccess("Completed!")
        return True


    def goOutDock(self, state):
        if (state == self.robot_state.MODE_PICKUP):
            s = 0.8
        elif (state == self.robot_state.MODE_DROPOFF):
            if (self.dock_name == 7 or self.dock_name == 8):
                s = 1.0
            else:
                s = 0.6
        elif (state == self.robot_state.MODE_CHARGE):
            return True

        self.setState(DockState.GO_OUT_DOCK, f"Move robot {s}m go out dock!")

        if not self.moveWithOdom(self.cfg.min_linear_vel,self.cfg.max_linear_vel, s):
            return False
        
        self.printOutSuccess("Completed!")
        return True 


if __name__ == "__main__":
    config = AutodockConfig()

    node = AutoDockStateMachine(
        config,
        run_server=True,
        load_rosparam=True,
        fake_clock=False)
    rospy.spin()
