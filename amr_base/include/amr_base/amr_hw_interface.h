#ifndef AMR_HW_INTERFACE_H
#define AMR_HW_INTERFACE_H

#include <amr_base/socket_tcp_client_fx3u.h>
#include <amr_base/pid.h>

//ROS
#include <ros/ros.h>
#include <urdf/model.h>
#include <amr_msgs/Encoders.h>
#include <amr_msgs/EncodersStamped.h>
#include <amr_msgs/WheelsCmdStamped.h>
#include <amr_msgs/AngularVelocitiesStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

//ROS Controls
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>


namespace amr_base
{
    const unsigned int NUM_JOINTS = 3;

    struct JointState
    {
        float angular_position_;
        float angular_velocity_;
    };

    struct Mode
    {
        std::string WHEEL;
        std::string JOINT;
        std::string SLIDER;
    };

    /// \brief Hardware interface for a robot
    class AmrHWInterface : public hardware_interface::RobotHW
    {
    public:
        /**
         * \brief Constructor
         * \param nh - Node handle for topics.
         * \param urdf_model - optional pointer to a parsed robot model
         */   
        AmrHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);

        /** \brief Destructor */
        virtual ~AmrHWInterface() {}

        /** \brief The init function is called to initialize the RobotHW from a
         *         non-realtime thread.
         *
         * Initialising a custom robot is done by registering joint handles
         * (\ref hardware_interface::ResourceManager::registerHandle) to hardware
         * interfaces that group similar joints and registering those individual
         * hardware interfaces with the class that represents the custom robot
         * (derived from this \ref hardware_interface::RobotHW)
         *
         * \note Registering of joint handles and interfaces can either be done in the
         * constructor or this \ref init method.
         *
         * \param root_nh A NodeHandle in the root of the caller namespace.
         *
         * \param robot_hw_nh A NodeHandle in the namespace from which the RobotHW
         * should read its configuration.
         *
         * \returns True if initialization was successful
         */
        virtual bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh);

        /** \brief Read data from the robot hardware.
         *
         * The read method is part of the control loop cycle (\ref read, update, \ref write) 
         * and is used to populate the robot state from the robot's hardware resources
         * (joints, sensors, actuators). This method should be called before 
         * controller_manager::ControllerManager::update() and \ref write.
         * 
         * \note The name \ref read refers to reading state from the hardware.
         * This complements \ref write, which refers to writing commands to the hardware.
         *
         * Querying WallTime inside \ref read is not realtime safe. The parameters
         * \p time and \p period make it possible to inject time from a realtime source.
         *
         * \param time The current time
         * \param period The time passed since the last call to \ref read
         */
        virtual void read(const ros::Time& time, const ros::Duration& period) override;

        /** \brief Write commands to the robot hardware.
         * 
         * The write method is part of the control loop cycle (\ref read, update, \ref write) 
         * and is used to send out commands to the robot's hardware 
         * resources (joints, actuators). This method should be called after 
         * \ref read and controller_manager::ControllerManager::update.
         * 
         * \note The name \ref write refers to writing commands to the hardware.
         * This complements \ref read, which refers to reading state from the hardware.
         *
         * Querying WallTime inside \ref write is not realtime safe. The parameters
         * \p time and \p period make it possible to inject time from a realtime source.
         *
         * \param time The current time
         * \param period The time passed since the last call to \ref write
         */

        virtual void write(const ros::Time& time, const ros::Duration& period);

        /** \brief Check if socket between PC and PLC is success.
         * 
         * This function blocks until the socket betwwen PC and PLC is success or reaches timeout
         *
         * \param timeout Minimum time to wait for conect socket
         */
        bool isSocketPcAndPlcOK(const ros::Duration &timeout=ros::Duration(1));

        /** \brief Helper for debugging a joint's state */
        virtual void printState();
        std::string printStateHelper();

        /** \brief Helper for debugging a joint's command */
        std::string printCommandHelper();

    protected:

        /** \brief Get the URDF XML from the parameter server */
        virtual void loadURDF(const ros::NodeHandle& nh, std::string param_name);

        /** \brief Callback to receive the encoder ticks from Arduino_Mega */
        // void encoderTicks_Callback(const amr_msgs::EncodersStamped::ConstPtr& msg_encoders);

        /** \brief Callback to receive the measured joint states from Arduino_Mega */
        // void measuredJointStates_Callback(const sensor_msgs::JointState::ConstPtr& msg_joint_states);

        /** \brief Callback to receive the signal from Emergency Stop button*/
        //void emergencyStopCallback(const std_msgs::Empty::ConstPtr& msg_EmergencyStop);             

        /** \brief Convert number of encoder ticks to angle in radians */
        double ticksToAngle(const int &ticks) const;

        /** \brief Normalize angle in the range of [0, 360) */
        double normalizeAngle(double &angle) const;

        // Reset all motor when start first launch
        bool reset_motor();

        // Publish motor encoder for debugging
        void pub_encoder_ticks(int32_t _encoder_ticks_[]);

        // Publish Encoder target
        void _pub_encoder_target(int32_t encoder_target);

        // Publish Encoder error
        void _pub_encoder_error(uint16_t encoder_error);

        // Publish joint states read from PLC (eg: velocity)
        void pub_measured_jointstate(std::vector<std::string> name, double pos[NUM_JOINTS], double vel[NUM_JOINTS]);

        // Send vel_cmd for PLC
        void send_vel_cmd_for_PLC(const std::vector<double>& speed, bool sign);

        // Stop amr function callback
        void stop_amr_cb(const std_msgs::Bool::ConstPtr& msg_stop_amr);

        // Hand control amr function callback
        void hand_control_cb(const std_msgs::Bool::ConstPtr& msg_hand_control);

        // Subscribe cmd_brake topic
        void cmd_brake_cb(const std_msgs::Bool::ConstPtr& msg_cmd_brake);

        // The following functions are currently unused
        // AMR directly calculates normalized angles from encoder ticks
        // The joint_commands from ros_control are mapped to percentage values for the motor driver
        // The following comments are incorrect
        /** \brief AMR reports travel distance in metres, need radians for ros_control RobotHW */
        double linearToAngular(const double &distance) const;
        /** \brief RobotHW provides velocity command in rad/s, AMR needs m/s. */
        double angularToLinear(const double &angle) const;

        // Set slider motor to original position if not at original position
        bool set_origin_slider_motor(const ros::Duration &timeout=ros::Duration(1));

        // Short name of this class
        std::string name_;

        // Startup and shutdown of the internal node inside a roscpp program
        ros::NodeHandle nh_;

        // Hardware interfaces
        // hardware_interface::JointStateInterface gives read access to all joint values 
        // without conflicting with other controllers.
        hardware_interface::JointStateInterface joint_state_interface_;
        // hardware_interface::VelocityJointInterface inherits from 
        // hardware_interface::JointCommandInterface and is used for reading and writing
        // joint velocities. Because this interface reserves the joints for write access,
        // conflicts with other controllers writing to the same joints might occure.
        // To only read joint velocities, avoid conflicts using 
        // hardware_interface::JointStateInterface.
        hardware_interface::VelocityJointInterface velocity_joint_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;              

        // Configuration
        std::vector<std::string> joint_names_;
        std::size_t num_joints_;
        urdf::Model *urdf_model_;

        double wheel_radius_;
        double wheel_diameter_;
        double max_velocity_;
        double min_position_;
        double max_position_;

        int32_t left_motor_vel;
        int32_t right_motor_vel;

        uint8_t origin_sensor_state;
        int origin_pos_bit_number;
        int pos_motor_origin_vel;
        int pos_motor_vel;

        std::vector<int32_t> motor_vel;

        std::vector<double> velocity;

        uint8_t dir_l;
        uint8_t dir_r;

        // Hardware related parameters to hold values from the parameter server
        // The parameters are defined in amr_base/config/base.yaml
        int MR_encoder_resolution_;
        int MS_encoder_resolution_;
        
        // For left-right motor
        double gear_ratio_;

        // For slider motor
        double gear_ratio_pinion, pinion_gear_radius, rack_pinion_gear_efficiency;

        // Parameters for the gain trim model
        double gain_;
        double trim_;
        // Assume the same motor constant for both motors
        double motor_constant_;
        //double pwm_limit_;

        // Enable/Disable debug output
        // Setting only possible during first start of the hw interface
        // to avoid reading permanently form the parameter server
        bool debug_; 

        // Accleration motor
        int acc_lim_motor;
        int acc_lim_motor_break;
        int current_lim_motor;
        int current_lim_motor_break;
        int slider_motor_origin_spd;
        int slider_motor_spd;
        std::vector<std::string>  operating_mode;

        // Structure Register
        std::string bit_name;
        std::string word_name;
        std::vector<double> reg_data_motor;
        int reg_data_motor_length;
        std::vector<double> reg_spd_cmd_motor;
        int reg_spd_cmd_length;
        std::vector<double> bit_reset_motor;
        std::vector<double> bit_break_motor;
        std::vector<double> bit_slider_origin;

        // PLC's IP Adress
        std::string ip_address;
        // PLC's IP Port
        int ip_port;

        // Data member array to store the controller commands which are sent to the 
        // robot's resources (joints, actuators)
        // The diff_drive_controller uses the hardware_interface::VelocityJointInterface
        // It provides semantic meaning to the wheel joints describing that 
        // they require velocity commands.
        double joint_commands_[NUM_JOINTS];
        double joint_positions_[NUM_JOINTS];
        double joint_velocities_[NUM_JOINTS];
        double joint_efforts_[NUM_JOINTS];

        // Data member arrays to store the state of the robot's resources (joints, sensors)
        // These values are filled in the read() method and are registered to the 
        // joint_state_interface_ of type hardware_interface::JointStateInterface.     

        // Declare publishers for angular wheel joint velocities
        ros::Publisher pub_wheel_cmd_velocities_;

        // Publish Encoder_ticks
        ros::Publisher pub_encoder_ticks_;

        // Publish Encoder target
        ros::Publisher pub_encoder_target;

        // Publish error between Encoder ticks and encoder target
        // ros::Publisher pub_encoder_error;

        // Publish Measured_Joint
        ros::Publisher pub_measured_joint_states;

        // Declare publisher to reset the wheel encoders
        // used during first launch of hardware interface to avoid large difference in encoder ticks from a previous run
        ros::Publisher pub_reset_encoders_;

        // Declare subscriber for the signal Emergency stop 
        // This subscriber receives signal from button Emergency Stop in the std_msgs::Empty message
        //ros::Subscriber sub_emergency_stop_;
        // Subscribers
        ros::Subscriber sub_stop_amr;

        ros::Subscriber sub_hand_control_amr;

        ros::Subscriber sub_cmd_brake;

        // Array to store the received encoder tick values from the \ref sub_encoder_ticks_ subscriber
        int32_t encoder_ticks_[NUM_JOINTS];
        int32_t encoder_ticks_prev[NUM_JOINTS];
        JointState measured_joint_states_[NUM_JOINTS];

        // Socket
        SkTcpClientFx3u sktcpclient_fx3u_;

        bool stop_amr;
        bool hand_control;
        bool cmd_brake;

        uint8_t accel_bytes[2];
        uint8_t accel_break_bytes[2];
        uint8_t current_bytes[2];
        uint8_t current_break_bytes[2];
        bool breaker_state[NUM_JOINTS];
        
        Mode MOTOR_MODE;

        PID pids_[NUM_JOINTS]; 
    };  // class AmrHWInterface
        
}// namespace

#endif // AMR_HW_INTERFACE_H