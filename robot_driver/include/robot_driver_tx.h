#ifndef __ROBOT_DIRVER_TX_H__
#define __ROBOT_DRIVER_TX_H__

#include <rclcpp/rclcpp.hpp>
#include "can_msgs/msg/frame.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "robot_interfaces/msg/qt_pub.hpp"
#include "robot_interfaces/msg/robot_control_msg.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "ros2_socketcan/socket_can_id.hpp"
#include "ros2_socketcan/socket_can_sender.hpp"
#include <iostream>
#include <cstdint>
#include <cstring>
#include <thread>
#include <chrono>

using namespace drivers::socketcan;

enum DataFdbClass_e {
    DATA1 = 0x01,
    DATA2,
    DATA3,
    DATA4
};
enum ExecuteResult_e {
    SUCCESS = 0x01,
    FAILED = 0xFF,
};
union FloatUintConverter_u {
    float f;
    uint32_t u;
};

// 发送接口类型
typedef enum {
    MOTOR_CTRL_TX = 0x000,
    FDB_REQ_TX = 0x200,
    FUNC_CTRL_TX = 0x400,
    PARM_RW_TX = 0x600,
} CommandID_Tx_e;

typedef enum {
    PARAM_R_TX = 0x01,
    PARAM_W_TX = 0x02,
} Operation_Method_e;

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

typedef enum {
    ONCE = 0x00,
    CYCLE = 0x01
} Motor_Feedback_Method_e;

class tx_package_t {
    public:
        uint8_t tx_data[8] = {0};

        std::shared_ptr<SocketCanSender> sender = std::make_shared<SocketCanSender>("can0", false);
        std::unique_ptr<CanId> canid;

        // Parameter Read
        tx_package_t(uint8_t motor_id, uint32_t interfaces_type, uint8_t operation_method, uint16_t params_addr);
        // Motor Control Commands
        tx_package_t(uint8_t motor_id, uint32_t interfaces_type, uint8_t control_enable_flag, uint8_t control_mode, float control_value);
        // Motor Feedback Request
        tx_package_t(uint8_t motor_id, uint32_t interfaces_type, uint8_t feedback_data, uint8_t feedback_method);
        // Function Operation interfaces
        tx_package_t(uint8_t motor_id, uint32_t interfaces_type, uint8_t func_operation_code);
        // Parameter Write
        tx_package_t(uint8_t motor_id, uint32_t interfaces_type, uint8_t operation_method, uint16_t param_addr, uint8_t param_type, int32_t param_data);
        // Parameter Write
        tx_package_t(uint8_t motor_id, uint32_t interfaces_type, uint8_t operation_method, uint16_t param_addr, uint8_t param_type, float_t param_data);
        // Gripper Control
        tx_package_t(uint8_t gripper_id, uint8_t gripper_position, uint8_t gripper_velocity, uint8_t gripper_effort);
        ~tx_package_t() = default;
};

class SocketCanSenderNode : public rclcpp :: Node 
{
    public:
        SocketCanSenderNode(const std::string& node_name);
        ~SocketCanSenderNode();

        void motor_data1_message_request(uint8_t feedback_method);
        void motor_data2_message_request(uint8_t feedback_method);
        void motor_data3_message_request(uint8_t feedback_method);
        void motor_data4_message_request(uint8_t feedback_method);

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
        rclcpp::Subscription<robot_interfaces::msg::QtPub>::SharedPtr subscriber_motor_states_;
        rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscriber_gripper_states_;
        rclcpp::Subscription<robot_interfaces::msg::RobotControlMsg>::SharedPtr subscriber_motor_control_msg_;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscriber_motor_zero_position_set_;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::thread data_request_thread_;
        
        // Motor Zero Position Set
        void motor_zero_position_set_callback(const std_msgs::msg::UInt8::SharedPtr msg);
        // rviz control
        void joint_pos_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
        // QT control
        void motor_states_request_callback(const robot_interfaces::msg::QtPub::SharedPtr msg);
        // Keyboard Control
        void motor_control_msg_callback(const robot_interfaces::msg::RobotControlMsg::SharedPtr msg);
        // Gripper Control
        void gripper_control_msg_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);

        void data_request_loop();
};

#endif