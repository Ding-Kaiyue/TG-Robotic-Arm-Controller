#ifndef __ROBOT_DRIVER_RX_H__
#define __ROBOT_DRIVER_RX_H__

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <map>
#include <memory>
#include <chrono>
#include <array>
#include "can_msgs/msg/frame.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "robot_interfaces/msg/motor_fdb.hpp"
#include "ros2_socketcan/socket_can_id.hpp"
#include "ros2_socketcan/socket_can_receiver.hpp"

using namespace drivers::socketcan;

#define PI_ 3.141592654f
#define MOTOR_NUM 6

typedef struct
{
    uint8_t enable_flag = 0;
    uint8_t control_mode = 0;
    uint8_t limit_flag = 0;
    uint16_t temperature = 0.0f;
    float position = 0.0f;
    float velocity = 0.0f;
    float current = 0.0f;
    float vbus = 0.0f;
    uint32_t error_code = 0;
} Motor_Status_t;

typedef enum
{
    MOTOR_CTRL_RX = 0x100,
    FDB_REQ_RX = 0x300,
    FUN_CTRL_RX = 0x500,
    PAM_RW_RX = 0x700,
} CommandID_Rx_e;

class rx_package_t
{
public:
    uint8_t Motor_ID;
    uint8_t rx_data[8] = {0};
    std::shared_ptr<Motor_Status_t> motor_status;
    rclcpp::Logger logger;

    enum DataFdbClass_e
    {
        DATA1 = 0x01,
        DATA2,
        DATA3,
        DATA4
    };
    union FloatUintConverter_u
    {
        float f;
        uint32_t u;
    };

    rx_package_t(CanId can_id, uint8_t *data, std::shared_ptr<Motor_Status_t> motor_status_, rclcpp::Logger logger_);
};

class SocketCanReceiverNode : public rclcpp :: Node
{
public:
    SocketCanReceiverNode(const std::string &node_name);

    std::string filter_str = "0x00000000~0xFFFFFFFF";
    SocketCanReceiver::CanFilterList filters;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    std::map<uint8_t, std::shared_ptr<Motor_Status_t>> motor_status_map;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<SocketCanReceiver> receiver_;

    union FloatUintConverter
    {
        float f;
        uint32_t u;
    };

    void timer_callback();
};

#endif