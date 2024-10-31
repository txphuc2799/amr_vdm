#!/usr/bin/env python3
import rospy
import tf2_ros
import yaml

from std_msgs.msg import Int16, Bool

safety_zone_file = "/home/amr/catkin_ws/src/amr_vdm/amr_driver/config/zone_types/safety_zone.yaml"

class SafetyZoneType():

    def __init__(self):

        # Get params from server
        self.pub_frequency = rospy.get_param("/pub_frequency", 10)

        self.rate = rospy.Rate(self.pub_frequency)

        # Frames
        self.map_frame = "map"
        self.robot_base_frame = "base_footprint"

        # Listen to Transfromation
        self.__tfBuffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.__tf_listener = tf2_ros.TransformListener(self.__tfBuffer)

        # Publishers:
        self._pub_safety_zone_type = rospy.Publisher("safety_zone_type", Int16, queue_size=5)

        # Subscribers:
        rospy.Subscriber("state_runonce_nav", Bool, self.runOnceStateCb)

        # Variables:
        self.is_run_once = False
        self.safety_zone_type = 1
        self.goal_name = 0
        self.safety_zone = self.yamlPose("safety_zone")
        
    
    def get2DPose(self):
        """
        Take 2D robot base frame respective map frame
        """
        ref_frame = self.map_frame
        target_frame = self.robot_base_frame
        try: 
            trans = self.__tfBuffer.lookup_transform(
                ref_frame, target_frame,
                rospy.Time(0), timeout=rospy.Duration(0.1))
            
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            rz = trans.transform.rotation.z
            rw = trans.transform.rotation.w

            return x, y, rz, rw
        
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logwarn(f"Failed lookup: {target_frame}, from {ref_frame}")
            return None

    
    def runOnceStateCb(self, msg:Bool):
        self.is_run_once = msg.data

    def pubSafetyZoneType(self, type):
        """
        `type = 0`: Big safety zone type
        `type = 1`: Small safety zone type
        """
        self._pub_safety_zone_type.publish(type)
    
    def yamlPose(self, position_name: str):
        """
        Get position from yaml file
        """
        assert (type(position_name) == str), "position_name is not str type"

        with open(safety_zone_file, 'r') as file:
            data = yaml.safe_load(file)

            position = data[f'{position_name}']['pose']
            float_position_list = [[float(value) if isinstance(value, (int, float, str)) \
                                  else value for value in sublist] for sublist in position]
                
            return float_position_list 
    
    def inZone(self, x, y, x1, x2, y1, y2):
        """
        Check whether AMR position in zone?
        `x`, `y`: respect with amr pose
        """
        if x1 > x2:
            if y1 > y2:
                if (x >= x2 and x <= x1 and
                    y >= y2 and y <= y1):
                    return True
            else:
                if (x >= x2 and x <= x1 and
                    y >= y1 and y <= y2):
                    return True
        else:
            if y1 > y2:
                if (x >= x1 and x <= x2 and
                    y >= y2 and y <= y1):
                    return True
            else:
                if (x >= x1 and x <= x2 and
                    y >= y1 and y <= y2):
                    return True

    def run(self):
        while not rospy.is_shutdown():
            if not self.is_run_once:
                self.rate.sleep()
                continue
            try:
                amr_tf = self.get2DPose()
                x, y,_,_ = amr_tf

                if self.inZone(x, y, self.safety_zone[0][0], self.safety_zone[1][0],
                                     self.safety_zone[0][1], self.safety_zone[1][1]):
                    if self.safety_zone_type:
                        self.pubSafetyZoneType(1)
                        self.safety_zone_type = 0

                elif self.inZone(x, y, self.safety_zone[2][0], self.safety_zone[3][0],
                                       self.safety_zone[2][1], self.safety_zone[3][1]):
                    if self.safety_zone_type:
                        self.pubSafetyZoneType(1)
                        self.safety_zone_type = 0

                elif self.inZone(x, y, self.safety_zone[4][0], self.safety_zone[5][0],
                                       self.safety_zone[4][1], self.safety_zone[5][1]):
                    if self.safety_zone_type:
                        self.pubSafetyZoneType(1)
                        self.safety_zone_type = 0

                elif not self.safety_zone_type:
                    self.pubSafetyZoneType(0)
                    self.safety_zone_type = 1
            except:
                pass

            self.rate.sleep()


if __name__== '__main__':
    rospy.init_node("safety_zone_type")
    try:
        safety_zone_type = SafetyZoneType()
        rospy.loginfo("SafetyZoneType node is running!")
        safety_zone_type.run()

    except rospy.ROSInterruptException:
        pass
        