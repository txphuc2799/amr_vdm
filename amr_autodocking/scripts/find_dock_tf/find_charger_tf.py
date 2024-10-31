#!/usr/bin/env python3

import math
import rospy
import tf2_ros
import tf_conversions
import geometry_msgs.msg

from geometry_msgs.msg import Pose2D

from laser_line_extraction.msg import LineSegmentList

class FindChargerTF():

    def __init__(self):

        # Parameters:
        self.vector_pattern_length = 0.15
        self.pattern_angle_bc = 100*math.pi/180
        self.pattern_angle_ab = 220*math.pi/180
        self.pattern_angle_cd = 220*math.pi/180
        self.pattern_distance_bc = (0.15 * 2 * math.cos(40*math.pi/180))
        self.distance_tolerance = 0.1
        self.pattern_length_tolerance = 0.1
        self.detect_angle_tolerance = 0.15

        self.laser_frame_id = "front_laser_link"
        self.map_frame_id = "map"

        # Subscribes:
        self.sub_line_segments = rospy.Subscriber("/front_line_segments", LineSegmentList,
                                                  self.lineSegmentCB)
        # Publishers:
        self.pub_dock_pose = rospy.Publisher("dock_pose", Pose2D, queue_size=5)
        

    def calculateVectorLength(self, start_point, end_point):
        length_vector = math.sqrt(pow(start_point[0] - end_point[0],2) + pow(start_point[1] - end_point[1],2))
        return length_vector
    

    def checkAngleBetweenTwoVector(self, angle_1, angle_2, angle_pattern_12, angle_tolerance):
        if (angle_1 * angle_2) > 0:
            angle = abs(angle_1 - angle_2)
        else:
            angle = 2*math.pi - abs(angle_1 - angle_2)
            if abs(angle - (2*math.pi - angle_pattern_12)) <= angle_tolerance:
                return True
            else:
                return False
            
        if (abs(angle_pattern_12 - angle) <= angle_tolerance):
            return True
        else:
            return False
        
    
    def checkDistanceBetweenTwoVector(self, start_point, end_point, pattern_distance, distance_tolerance):
        distance = self.calculateVectorLength(start_point, end_point)
        return True if (abs(distance - pattern_distance) <= distance_tolerance) else False
    

    def checkDistance(self, start_point, end_point, distance_tolerance):
        distance = self.calculateVectorLength(start_point, end_point)
        return True if (distance <= distance_tolerance) else False
            

    def transformTFDock(self, x, y, theta, child_frame_id, frame_id):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        q = tf_conversions.transformations.quaternion_from_euler(0,0,theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransform(t)
    

    def lineSegmentCB(self, msg: LineSegmentList):
        
        # Send a virtual frame for comparing between virtual frame & acutal frame
        self.transformTFDock(4.955, -2.465, math.pi, "virtual_charger_frame", self.map_frame_id)

        line_segments = msg.line_segments
        line_num = len(line_segments)

        find_bc = False
        find_a = False
        find_d = False

        # Check number of line
        # print(f"Number of line = {line_num}")

        if line_num < 2:
            # rospy.logwarn("There isn't enough line in the laser field!")
            return
        
        # Find vector b, c
        for i in range(line_num-1):
            for j in range(i+1, line_num):
                if self.checkAngleBetweenTwoVector(line_segments[i].angle,
                                                   line_segments[j].angle,
                                                   math.pi - self.pattern_angle_bc,
                                                   self.detect_angle_tolerance):
                    if (self.checkDistanceBetweenTwoVector(line_segments[i].start,
                                                           line_segments[j].end,
                                                           self.pattern_distance_bc,
                                                           self.distance_tolerance)):
                        vector_b = line_segments[i]
                        vector_c = line_segments[j]
                        find_bc = True

                    elif (self.checkDistanceBetweenTwoVector(line_segments[j].start,
                                                             line_segments[i].end,
                                                             self.pattern_distance_bc,
                                                             self.distance_tolerance)):
                        vector_b = line_segments[j]
                        vector_c = line_segments[i]
                        find_bc = True

        # Find vector a, d
        if find_bc:
            for i in range(line_num):
                if (self.checkAngleBetweenTwoVector(line_segments[i].angle,
                                                    vector_b.angle,
                                                    self.pattern_angle_ab - math.pi,
                                                    self.detect_angle_tolerance)): 
                    if (self.checkDistance(line_segments[i].end,
                                           vector_b.start,
                                           self.distance_tolerance)):
                        if (self.checkDistanceBetweenTwoVector(line_segments[i].start,
                                                               line_segments[i].end,
                                                               self.vector_pattern_length,
                                                               self.pattern_length_tolerance)):
                            vector_a = line_segments[i]
                            find_a = True
                
                if (self.checkAngleBetweenTwoVector(line_segments[i].angle,
                                                    vector_c.angle,
                                                    self.pattern_angle_cd - math.pi,
                                                    self.detect_angle_tolerance)):
                    if (self.checkDistance(vector_c.end,
                                           line_segments[i].start,
                                           self.distance_tolerance)):
                        if (self.checkDistanceBetweenTwoVector(line_segments[i].start,
                                                               line_segments[i].end,
                                                               self.vector_pattern_length,
                                                               self.pattern_length_tolerance)):
                            vector_d = line_segments[i]
                            find_d = True

        if (find_bc and find_a and find_d):

            x = (vector_a.end[0] + vector_d.start[0])/2
            y = (vector_b.start[1] + vector_c.end[1])/2
            theta = math.pi + (vector_a.angle + vector_d.angle)/2
            
            dock_pose = Pose2D()
            dock_pose.x = x
            dock_pose.y = y
            dock_pose.theta = theta
            self.pub_dock_pose.publish(dock_pose)

            self.transformTFDock(x, y, theta, "charger_frame", self.laser_frame_id)


if __name__== '__main__':
    rospy.init_node("find_charger_tf")
    try:
        find_charger_tf = FindChargerTF()
        rospy.loginfo("FindChargerTF is running!")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass