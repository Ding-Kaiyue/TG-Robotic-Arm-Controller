#include "robot_driver_rx.h"

using namespace drivers::socketcan;
using std::placeholders::_1;

void SocketCanReceiverNode::timer_callback() {
    auto message = sensor_msgs::msg::JointState();
    message.header.stamp = this->get_clock()->now();

    for (const auto &[motor_id, status] : motor_status_map) {
        message.name.push_back("joint" + std::to_string(motor_id));
        message.position.push_back(status->position * PI_ / 180.0);
        message.velocity.push_back(status->velocity);
        message.effort.push_back(status->current);
    }
    publisher_->publish(message);
}

rx_package_t::rx_package_t(CanId can_id, uint8_t *data, std::shared_ptr<Motor_Status_t> motor_status_, rclcpp::Logger logger_) : motor_status(motor_status_), logger(logger_) {
    this->motor_status = motor_status_;

    if (sizeof(data) == 8) {
        std::copy(data, data + 8, rx_data);
        Motor_ID = can_id.identifier() & 0x000000FF;

        switch (can_id.identifier() & 0xFFFFFF00) {
            // 常用控制接口
            case MOTOR_CTRL_RX: {
                break;
            }
            // 请求反馈接口
            case FDB_REQ_RX: {
                if (rx_data[0] == DATA1) { // data1
                    motor_status->enable_flag = rx_data[1];
                    motor_status->control_mode = rx_data[2];
                    uint32_t position_uint = (rx_data[4] << 24) | (rx_data[5] << 16) | (rx_data[6] << 8) | (rx_data[7] & 0xFF);
                    FloatUintConverter_u converter;
                    converter.u = position_uint;
                    motor_status->position = converter.f;
                }
                else if (rx_data[0] == DATA2) { // data2
                    motor_status->limit_flag = rx_data[1];
                    motor_status->temperature = (rx_data[2] << 8) | (rx_data[3] & 0xFF);
                    uint32_t velocity_uint = (rx_data[4] << 24) | (rx_data[5] << 16) | (rx_data[6] << 8) | (rx_data[7] & 0xFF);
                    FloatUintConverter_u converter;
                    converter.u = velocity_uint;
                    motor_status->velocity = converter.f;
                }
                else if (rx_data[0] == DATA3) { // data3
                    motor_status->vbus = (rx_data[2] << 8) | (rx_data[3] & 0xFF);
                    uint32_t current_uint = (rx_data[4] << 24) | (rx_data[5] << 16) | (rx_data[6] << 8) | (rx_data[7] & 0xFF);
                    FloatUintConverter_u converter;
                    converter.u = current_uint;
                    motor_status->current = converter.f;
                } else if (rx_data[0] == DATA4) { // data4
                    motor_status->error_code = (rx_data[4] << 24) | (rx_data[5] << 16) | (rx_data[6] << 8) | (rx_data[7] & 0xFF);
                }
                break;
            }
            // 函数操作接口
            case FUN_CTRL_RX: {
                if (rx_data[1] == 1) {
                    RCLCPP_INFO(logger, "Operation %u done successfully!", static_cast<unsigned>(rx_data[0]));
                }
                break;
            }
            // 参数读写接口
            case PAM_RW_RX: {
                uint16_t param_addr = static_cast<uint16_t>((rx_data[1] << 8) | (rx_data[2] & 0xFF));
                if (rx_data[0] == 0x01) { // Parameter read
                    if (rx_data[3] == 0x01) { // The parameter is int
                        int32_t param_value = (rx_data[4] << 24) | (rx_data[5] << 16) | (rx_data[6] << 8) | (rx_data[7] & 0xFF);
                        RCLCPP_INFO(logger, "Read parameter %u (int), value: %d", param_addr, param_value);
                    }
                    else if (rx_data[3] == 0x02) { // The parameter is float
                        FloatUintConverter_u converter;
                        converter.u = (rx_data[4] << 24) | (rx_data[5] << 16) | (rx_data[6] << 8) | (rx_data[7] && 0xFF);
                        RCLCPP_INFO(logger, "Read parameter %u (float), value: %f", param_addr, converter.f);
                    }
                }
                else if (rx_data[0] == 0x02) { // Parameter write
                    if (rx_data[3] == 0x01) { // The parameter is int
                        int32_t param_value = (rx_data[4] << 24) | (rx_data[5] << 16) | (rx_data[6] << 8) | (rx_data[7] & 0xFF);
                        RCLCPP_INFO(logger, "Write parameter %u (int), value: %d", param_addr, param_value);
                    }
                    else if (rx_data[3] == 0x02) { // The parameter is float
                        FloatUintConverter_u converter;
                        converter.u = (rx_data[4] << 24) | (rx_data[5] << 16) | (rx_data[6] << 8) | (rx_data[7] && 0xFF);
                        RCLCPP_INFO(logger, "Write parameter %u (float), value: %f", param_addr, converter.f);
                    }
                }
                break;
            }
            default: {
                break;
            }
        }
    }
}

SocketCanReceiverNode::SocketCanReceiverNode(const std::string &node_name) : Node(node_name), filter_str("0x00000000~0xFFFFFFFF"), filters(SocketCanReceiver::CanFilterList::ParseFilters(filter_str)) {
    receiver_ = std::make_unique<SocketCanReceiver>("can0", false);
    // publish the joint attitude from the real motors
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&SocketCanReceiverNode::timer_callback, this));
    filters = SocketCanReceiver::CanFilterList::ParseFilters(filter_str);
    std::shared_ptr<Motor_Status_t> motor_status = std::make_shared<Motor_Status_t>();

    // Set CAN Filters
    try {
        receiver_->SetCanFilters(filters);
    } catch (const std::runtime_error & e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set filters: %s", e.what());
    }

    // CAN Message Receive
    std::thread([this]() {
        std::array<uint8_t, 8> data;
        while (rclcpp::ok()) {
            try {
                CanId can_id = receiver_->receive(data, std::chrono::seconds(1));
                uint8_t motor_id = can_id.identifier() & 0x000000FF;

                if (motor_status_map.find(motor_id) == motor_status_map.end()) {
                    motor_status_map[motor_id] = std::make_shared<Motor_Status_t>();
                }

                rx_package_t can_rx(can_id, data.data(), motor_status_map[motor_id], this->get_logger());
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Error receiving CAN data: %s", e.what());
            }
        }
    }).detach();
}
