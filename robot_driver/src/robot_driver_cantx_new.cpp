#include "robot_driver_tx.h"
#include "robot_working_mode.h"

using namespace drivers::socketcan;
using std::placeholders::_1;

tx_package_t::tx_package_t(uint8_t motor_id, uint32_t interfaces_type, uint8_t operation_method, uint16_t params_addr) {
    this->canid = std::make_unique<CanId>(motor_id + interfaces_type, 0, FrameType::DATA, ExtendedFrame);
    tx_data[0] = operation_method;
    tx_data[1] = params_addr >> 8;
    tx_data[2] = params_addr & 0xFF;
    sender->send(tx_data, sizeof(tx_data), *this->canid, std::chrono::seconds(1));
}

tx_package_t::tx_package_t(uint8_t motor_id, uint32_t interfaces_type, uint8_t control_enable_flag, uint8_t control_mode, float control_value) {
    this->canid = std::make_unique<CanId>(motor_id + interfaces_type, 0, FrameType::DATA, ExtendedFrame);
    FloatUintConverter_u converter;
    converter.f = static_cast<float>(control_value) * 180.0f / 3.141592654f;
    uint32_t control_value_uint = converter.u;
    tx_data[0] = control_enable_flag;
    tx_data[1] = control_mode;
    tx_data[2] = 0;
    tx_data[3] = 0;
    tx_data[4] = control_value_uint >> 24;
    tx_data[5] = control_value_uint >> 16;
    tx_data[6] = control_value_uint >> 8;
    tx_data[7] = control_value_uint & 0xFF;
    sender->send(tx_data, sizeof(tx_data), *this->canid, std::chrono::seconds(1));
}

tx_package_t::tx_package_t(uint8_t motor_id, uint32_t interfaces_type, uint8_t feedback_data, uint8_t feedback_method) {
    this->canid = std::make_unique<CanId>(motor_id + interfaces_type, 0, FrameType::DATA, ExtendedFrame);
    tx_data[0] = feedback_data;
    tx_data[1] = feedback_method;
    sender->send(tx_data, sizeof(tx_data), *this->canid, std::chrono::seconds(1));
}

tx_package_t::tx_package_t(uint8_t motor_id, uint32_t interfaces_type, uint8_t func_operation_code) {
    this->canid = std::make_unique<CanId>(motor_id + interfaces_type, 0, FrameType::DATA, ExtendedFrame);
    tx_data[0] = func_operation_code;
    sender->send(tx_data, sizeof(tx_data), *this->canid, std::chrono::seconds(1));
}

tx_package_t:: tx_package_t(uint8_t motor_id, uint32_t interfaces_type, uint8_t operation_method, uint16_t param_addr, uint8_t param_type, int32_t param_data) {
    this->canid = std::make_unique<CanId>(motor_id + interfaces_type, 0, FrameType::DATA, ExtendedFrame);
    tx_data[0] = operation_method;
    tx_data[1] = param_addr >> 8;
    tx_data[2] = param_addr & 0xFF;
    tx_data[3] = param_type;
    tx_data[4] = param_data >> 24;
    tx_data[5] = param_data >> 16;
    tx_data[6] = param_data >> 8;
    tx_data[7] = param_data & 0xFF;
    sender->send(tx_data, sizeof(tx_data), *this->canid, std::chrono::seconds(1));
}

tx_package_t::tx_package_t(uint8_t motor_id, uint32_t interfaces_type, uint8_t operation_method, uint16_t param_addr, uint8_t param_type, float_t param_data) {
    this->canid = std::make_unique<CanId>(motor_id + interfaces_type, 0, FrameType::DATA, ExtendedFrame);
    tx_data[0] = operation_method;
    tx_data[1] = param_addr >> 8;
    tx_data[2] = param_addr & 0xFF;
    tx_data[3] = param_type;

    FloatUintConverter_u converter;
    converter.f = param_data;
    uint32_t param_data_uint = converter.u;
    tx_data[4] = param_data_uint >> 24;
    tx_data[5] = param_data_uint >> 16;
    tx_data[6] = param_data_uint >> 8;
    tx_data[7] = param_data_uint & 0xFF;
    sender->send(tx_data, sizeof(tx_data), *this->canid, std::chrono::seconds(1));
}

tx_package_t::tx_package_t(uint8_t gripper_id, uint8_t gripper_position, uint8_t gripper_velocity, uint8_t gripper_effort) {
    this->canid = std::make_unique<CanId>(gripper_id, 0, FrameType::DATA, StandardFrame);
    tx_data[0] = 0x03;
    tx_data[1] = gripper_position;
    tx_data[2] = gripper_velocity;
    tx_data[3] = gripper_effort;
    sender->send(tx_data, sizeof(tx_data), *this->canid, std::chrono::seconds(1));
}

void SocketCanSenderNode::motor_data1_message_request(uint8_t feedback_method) {
    for (uint8_t i = 0; i < 6; i++) {
        tx_package_t data1_msg_request(i+1, static_cast<uint32_t>(FDB_REQ_TX), 
                                            static_cast<uint8_t>(DATA1), 
                                            static_cast<uint8_t>(feedback_method));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
void SocketCanSenderNode::motor_data2_message_request(uint8_t feedback_method) {
    for (uint8_t i = 0; i < 6; i++) {
        tx_package_t data2_msg_request(i+1, static_cast<uint32_t>(FDB_REQ_TX), 
                                            static_cast<uint8_t>(DATA2), 
                                            static_cast<uint8_t>(feedback_method));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
void SocketCanSenderNode::motor_data3_message_request(uint8_t feedback_method) {
    for (uint8_t i = 0; i < 6; i++) {
        tx_package_t data3_msg_request(i+1, static_cast<uint32_t>(FDB_REQ_TX), 
                                            static_cast<uint8_t>(DATA3), 
                                            static_cast<uint8_t>(feedback_method));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
void SocketCanSenderNode::motor_data4_message_request(uint8_t feedback_method) {
    for (uint8_t i = 0; i < 6; i++) {
        tx_package_t data4_msg_request(i+1, static_cast<uint32_t>(FDB_REQ_TX), 
                                            static_cast<uint8_t>(DATA4), 
                                            static_cast<uint8_t>(feedback_method));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void SocketCanSenderNode::motor_zero_position_set_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
    // motor disable
    tx_package_t motor_disable(static_cast<uint8_t>(msg->data), MOTOR_CTRL_TX, MOTOR_DISABLE, POSITION_ABS_MODE, 0.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // Set the current position to motor zero position
    tx_package_t motor_zero_position_set(static_cast<uint8_t>(msg->data), FUNC_CTRL_TX, 0x04);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // Save the parameter to flash
    tx_package_t motor_save_to_flash(static_cast<uint8_t>(msg->data), FUNC_CTRL_TX, 0x02);
    RCLCPP_INFO(this->get_logger(), "==========================================");
}

// rviz control
void SocketCanSenderNode::joint_pos_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i = 0; i < msg->position.size(); i++) {
        tx_package_t motor_control_pos(static_cast<uint8_t>(i+1), MOTOR_CTRL_TX, MOTOR_ENABLE, POSITION_ABS_MODE, msg->position[i]);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// QT control
void SocketCanSenderNode::motor_states_request_callback(const robot_interfaces::msg::QtPub::SharedPtr msg) {
    if (msg->working_mode != 0x06) {
        for (size_t i = 0; i < msg->joint_group_positions.size(); i++) {
            tx_package_t motor_control_pos(static_cast<uint8_t>(i+1), MOTOR_CTRL_TX, MOTOR_ENABLE, POSITION_ABS_MODE, msg->joint_group_positions[i]);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    } else {
        for (size_t i = 0; i < msg->joint_group_positions.size(); i++) {
            tx_package_t motor_control_speed(static_cast<uint8_t>(i+1), MOTOR_CTRL_TX, MOTOR_ENABLE, SPEED_MODE, 0.0f);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

// Keyboard Control
void SocketCanSenderNode::motor_control_msg_callback(const robot_interfaces::msg::RobotControlMsg::SharedPtr msg) {
    for (size_t i = 0; i < msg->motor_mode.size(); i++) {
        tx_package_t motor_control(static_cast<uint8_t>(i+1), MOTOR_CTRL_TX, msg->motor_enable_flag[i], msg->motor_mode[i], msg->motor_msg[i]);
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); 
        // RCLCPP_INFO(this->get_logger(), "Position[%lf] has been sent to motor[%ld]", msg->motor_msg[i], i+1);
    }
}

// Gripper Control
void SocketCanSenderNode::gripper_control_msg_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    tx_package_t gripper_control(0x3F, msg->data[0], msg->data[1], msg->data[2]);
    RCLCPP_INFO(this->get_logger(), "Gripper msg sent------------------");
}

void SocketCanSenderNode::data_request_loop() {
    while (rclcpp::ok()) {
        motor_data1_message_request(ONCE);
        motor_data3_message_request(ONCE);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

SocketCanSenderNode::SocketCanSenderNode(const std::string& node_name) : Node(node_name) {
    // Rviz Controller
    subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("motor_cmd", 10, std::bind(&SocketCanSenderNode::joint_pos_callback, this, std::placeholders::_1));
    // Qt Controller
    subscriber_motor_states_ = this->create_subscription<robot_interfaces::msg::QtPub>("motor_states_req", 10, std::bind(&SocketCanSenderNode::motor_states_request_callback, this, std::placeholders::_1));
    subscriber_gripper_states_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>("gripper_control_msg", 10, std::bind(&SocketCanSenderNode::gripper_control_msg_callback, this, std::placeholders::_1));
    // Keyboards Controller
    subscriber_motor_control_msg_ = this->create_subscription<robot_interfaces::msg::RobotControlMsg>("motor_control_msg", 10, std::bind(&SocketCanSenderNode::motor_control_msg_callback, this, std::placeholders::_1));
    data_request_thread_ = std::thread(&SocketCanSenderNode::data_request_loop, this);
    data_request_thread_.detach();
    subscriber_motor_zero_position_set_ = this->create_subscription<std_msgs::msg::UInt8>("motor_zero_position_set", 10, std::bind(&SocketCanSenderNode::motor_zero_position_set_callback, this, std::placeholders::_1));
}

SocketCanSenderNode::~SocketCanSenderNode()
{
    if (data_request_thread_.joinable()) {
        data_request_thread_.join();
    }
}
