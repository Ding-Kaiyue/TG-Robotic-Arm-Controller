#ifndef __ROBOT_KEY_CONTROLLER_H__
#define __ROBOT_KEY_CONTROLLER_H__

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.h>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <robot_interfaces/msg/robot_control_msg.hpp>
#include <robot_interfaces/msg/move_c_action.hpp>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <urdf/model.h>
#include <chrono>
#include <thread>
#include <sqlite3.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <sstream>

#define BACKTOSTART 0
#define DISABLE 1
#define JOINTCONTROL 2
#define CARTESIAN 3
#define MOVEJ 4
#define MOVEL 5
#define MOVEC 6
#define TEACH 7
#define TEACHREPEAT 8
#define SAVESTATE 9
#define LOADSTATE 10
#define BACKTOINITIAL 11

#define VELOCITY_MODE 0x04
#define POSITION_MODE 0x05
#define EFFORT_MODE 0x02

#define JOINT1_POSITIVE 41
#define JOINT1_NEGATIVE 42
#define JOINT2_POSITIVE 43
#define JOINT2_NEGATIVE 44
#define JOINT3_POSITIVE 45
#define JOINT3_NEGATIVE 46
#define JOINT4_POSITIVE 47
#define JOINT4_NEGATIVE 48
#define JOINT5_POSITIVE 49
#define JOINT5_NEGATIVE 50
#define JOINT6_POSITIVE 51
#define JOINT6_NEGATIVE 52
#define GRIPPER_OPEN 53
#define GRIPPER_CLOSE 54

#define FORWARD 55
#define BACKWARD 56
#define LEFT 57
#define RIGHT 58
#define UP 59
#define DOWN 60
#define ROLL_POSITIVE 61
#define ROLL_NEGATIVE 62
#define PITCH_POSITIVE 63
#define PITCH_NEGATIVE 64
#define YAW_POSITIVE 65
#define YAW_NEGATIVE 66


#define PI_ 3.141592654f
#define JOINT_ROTATE_POSITIVE_SPEED 0.04*3.141592654f
#define JOINT_ROTATE_NEGATIVE_SPEED -0.04*3.141592654f

typedef enum {
    CURRENT_MODE = 0x02,
    EFFORT_POSITION_MODE = 0x03,
    SPEED_MODE = 0x04,
    POSITION_ABS_MODE = 0x05,
    POSITION_INC_MODE = 0x06
} Motor_Control_Mode_e;

typedef enum {
    MOTOR_ENABLE = 0x01,
    MOTOR_DISABLE = 0x00
} Motor_Enable_Mode_e;

class KeyboardController : public rclcpp :: Node {
    public:
        explicit KeyboardController(std::string node_name);
        ~KeyboardController(){}
    private:
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub;

        /* Parameter Set */
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
        double speed_scaling_{0.5f};
        
        /* JOINTCTRL mode */
        std::map<std::string, std::pair<double, double>> joint_limits_;
        rclcpp::Time last_jointctrl_msg_time;
        rclcpp::TimerBase::SharedPtr jointctrl_action_timer_;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr jointctrl_action_sub_;

        /* CARTESIAN mode */
        rclcpp::Time last_cartesian_msg_time;
        rclcpp::TimerBase::SharedPtr cartesian_action_timer_;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr cartesian_action_sub_;


        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr movej_action_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr movel_action_sub_;
        rclcpp::Subscription<robot_interfaces::msg::MoveCAction>::SharedPtr movec_action_sub_; 
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr teach_action_sub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr teach_repeat_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr save_state_sub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr load_state_sub_;

        rclcpp::Publisher<robot_interfaces::msg::RobotControlMsg>::SharedPtr motor_msg_pub;
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr gripper_msg_pub;
        rclcpp::Publisher<robot_interfaces::msg::RobotControlMsg>::SharedPtr joint_states_pub_;
        geometry_msgs::msg::Pose current_pose_;
        
        /* Teach and Teach Repeat*/
        sensor_msgs::msg::JointState::SharedPtr latest_joint_states_;
        std::unique_ptr<rosbag2_cpp::Writer> teach_bag_writer_;
        std::unique_ptr<rosbag2_cpp::Reader> play_bag_reader_;
        
        uint8_t current_gripper_states_[3] = {0, 20, 0};

        // std::vector<double> joint_group_positions;
        robot_interfaces::msg::RobotControlMsg rbt_ctrl_msg;
        
        uint32_t current_count_;
        bool recording_;

        void working_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg);
        

        /* JOINTCONTROL mode */
        void jointctrl_motor_mode(const uint8_t joint_index, const float speed);
        void load_joint_limits();
        void jointctrl_action_callback(const std_msgs::msg::UInt8::SharedPtr msg);
        void jointctrl_timer_callback();
        /* CARTESIAN mode */
        void move_arm_in_direction(const geometry_msgs::msg::Pose& current_pose, int direction);
        void move_arm_in_orientation(const geometry_msgs::msg::Pose& current_pose, int direction);
        void adjust_gripper(int action);
        void cartesian_action_callback(const std_msgs::msg::UInt8::SharedPtr msg);
        void cartesian_timer_callback();

        void movej_action_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
        void movel_action_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
        void movec_action_callback(robot_interfaces::msg::MoveCAction::SharedPtr msg);
        void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

        /* TEACH and TEACHREPEAT mode */
        void teach_motor_mode();
        void teach_action_callback(const std_msgs::msg::String::SharedPtr msg);
        void teach_repeat_callback(const std_msgs::msg::String::SharedPtr msg);

        /* SAVESTATE and LOADSTATE mode */
        void save_current_state_to_db(const std::string& state_name);
        void load_state_from_db(const std::string& state_name);
        void save_state_callback(const std_msgs::msg::String::SharedPtr msg);
        void load_state_callback(const std_msgs::msg::String::SharedPtr msg);

};

#endif
