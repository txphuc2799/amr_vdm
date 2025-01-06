#!/usr/bin/env python3
import rospy

from std_msgs.msg import Bool, Int16, Empty
from amr_msgs.msg import SafetyZone
from amr_driver.mcprotocol.type1e import Type1E

class Parameter():
    PLC_port = 8002
    obstacle_detecting_bit = "M4"
    safety_zone_bit = "M28"
    initialpose_bit = "M406"
    brake_bit = "M161"
    runonce_bit = "M8"
    start_bit = "M405"
    error_mode_bit = ["M70", "M71", "M72"]
    go_out_silder_bit = "M151"
    go_in_silder_bit = "M150"
    load_finished_bit = "M9"
    server_cancel_bit = "M40"
    reset_amr_bit = "M413"

class PCWritePLC(Type1E):

    def __init__(self):
        
        super().__init__()
        rospy.init_node("PC_write_PLC")

        self.params = Parameter()
        self.init_params()

        # Parameter of PLC brigde:
        self.PLC_IP_address = rospy.get_param("~IP_addres_PLC",'192.168.0.250')

        # Connect to PLC
        self.connect_PLC(self.PLC_IP_address, self.params.PLC_port)
        if self._is_connected:
            rospy.loginfo("PC_controller(WRITE): is connected to PLC success")
        else:
            rospy.logerr("PC_controller(WRITE): can't connect to PLC")

        # Only Subcriber:
        rospy.Subscriber("status_protected_field",Bool,self.protected_field_callback)
        rospy.Subscriber("cmd_brake",Bool, self.brake_callback)
        rospy.Subscriber("state_runonce_nav", Bool, self.runonce_callback)
        rospy.Subscriber("is_intialpose", Bool, self.is_initialpose_callback)
        rospy.Subscriber("safety_zone_type", SafetyZone, self.safety_zone_callback)
        rospy.Subscriber("error_mode", Int16, self.mode_error_callback)
        rospy.Subscriber("cmd_slider", Int16, self.cmd_slider_callback)
        rospy.Subscriber("/back_camera/camera_startup_finished", Bool, self.finished_callback)
        rospy.Subscriber("RESET_AMR", Empty, self.reset_callback)

        # Avariables:
        self.status_protected_field = False
        self.brake_cmd = False
        self.slider_cmd = 0
        self.mode_error = 0


    def init_params(self):
        print(f"NODE: PCWritePLC")
        print("PARAMETERS")

        param_names = [attr for attr in dir(self.params) if not callable(getattr(self.params, attr)) and not attr.startswith("__")]

        # get private rosparam, if none use default
        for param_name in param_names:
            param_val = rospy.get_param("~" + param_name, getattr(self.params, param_name))
            print(f"* /{param_name}: {param_val}")
            setattr(self.params, param_name, param_val)


    def finished_callback(self, msg: Bool):
        if msg.data:
            rospy.loginfo("PLCWrite: AMR is ready for running!")
            self.batchwrite_bitunits(self.params.load_finished_bit, [1])


    def cmd_slider_callback(self, msg: Int16):
        self.slider_cmd = msg.data
        
        if self.slider_cmd == 1:
            self.batchwrite_bitunits(self.params.go_out_silder_bit, [1])   # M151 - Slider go out
        elif self.slider_cmd == 2:
            self.batchwrite_bitunits(self.params.go_in_silder_bit, [1])    # M150 - Slider go in


    def protected_field_callback(self, msg: Bool):
        self.status_protected_field = msg.data

        if self.status_protected_field:
            self.batchwrite_bitunits(self.params.obstacle_detecting_bit, [1])  # M4
            rospy.logwarn("PLCWrite: Detect obtacles in protected filed!")
        else:
            self.batchwrite_bitunits(self.params.obstacle_detecting_bit, [0])


    def safety_zone_callback(self, msg: SafetyZone):
        if msg.zone == SafetyZone.SMALL_ZONE:
            self.batchwrite_bitunits(self.params.safety_zone_bit, [1,1])       # M28: Front - M29: Back
            rospy.loginfo("PLCWrite: Switched back and front safety zone to small zone!")
        else:
            self.batchwrite_bitunits(self.params.safety_zone_bit, [0,0])
            rospy.loginfo("PLCWrite: Switched back and front safety zone to default zone!")


    def is_initialpose_callback(self, msg: Bool):
        if msg.data:
            self.batchwrite_bitunits(self.params.initialpose_bit, [0])          # M406

    def brake_callback(self, msg: Bool):
        self.brake_cmd = msg.data

        if self.brake_cmd:
            self.batchwrite_bitunits(self.params.brake_bit, [0,0])   # Brake off [M1-M2]
            rospy.loginfo("PLCWrite: Brake is ON.")
        else:
            self.batchwrite_bitunits(self.params.brake_bit, [1,1])   # Brake on
            rospy.loginfo("PLCWrite: Brake is OFF.")


    def runonce_callback(self, msg: Bool):
        if msg.data:
            self.batchwrite_bitunits(self.params.runonce_bit, [1])    # M8
        else:
            self.batchwrite_bitunits(self.params.runonce_bit, [0])


    def mode_error_callback(self, msg):
        self.batchwrite_wordunits("D200", [msg.data])

    def reset_callback(self, msg: Empty):
        self.batchwrite_bitunits(self.params.reset_amr_bit, [0])

        
if __name__== '__main__':
    try:
        PC_bridge_PLC = PCWritePLC()
        rospy.loginfo("PCWritePLC node is running!")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass