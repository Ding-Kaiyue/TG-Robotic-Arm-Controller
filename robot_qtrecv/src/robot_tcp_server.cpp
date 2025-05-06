#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/quaternion.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "robot_interfaces/msg/qt_recv.hpp"
#include "robot_interfaces/msg/arm_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;

class TcpServer : public rclcpp::Node
{
    public:
        TcpServer(const std::string& node_name) : Node(node_name)
        {
            publisher_ = this->create_publisher<robot_interfaces::msg::QtRecv>("qt_cmd", 10);
            publisher_gripper_states_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("gripper_control_msg", 10);
            subscriber_states_ = this->create_subscription<robot_interfaces::msg::ArmState>("arm_states", 10, std::bind(&TcpServer::arm_states_callback, this, _1));
            subscriber_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&TcpServer::joint_states_callback, this, _1));

            server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
            if (server_fd_ < 0) {
                RCLCPP_ERROR(this->get_logger(), "Socket creation failed!");
                return;
            }
            // 套接字选项
            int opt = 1;        
            if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
                RCLCPP_ERROR(this->get_logger(), "Setsockopt failed!");
                return;
            }
            // 绑定套接字
            struct sockaddr_in address;
            address.sin_family = AF_INET;
            address.sin_addr.s_addr = INADDR_ANY;
            address.sin_port = htons(8899);         // 端口号
            if (bind(server_fd_, (struct sockaddr *)&address, sizeof(address)) < 0) {
                RCLCPP_ERROR(this->get_logger(), "Bind failed!");
                return;
            }

            // 监听套接字
            if (listen(server_fd_, 3) < 0) {
                RCLCPP_ERROR(this->get_logger(), "Listen failed!");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "TCP Server is listening on port 8899!");

            // 启动一个线程处理连接
            accept_thread_ = std::thread(&TcpServer::acceptLoop, this);
        }

        ~TcpServer() {
            if (accept_thread_.joinable()) {
                accept_thread_.join();
            }
            close(server_fd_);
        }

    private:
        int server_fd_;
        std::thread accept_thread_;
        int receivedValue[50];
        robot_interfaces::msg::QtRecv qt_cmd;
        std_msgs::msg::UInt8MultiArray gripper_states;
        int client_fd_;
        geometry_msgs::msg::Pose end_effector_pose;     // 末端执行器的当前姿态
        tf2::Quaternion end_effector_quat;    // 末端执行器的当前四元数

        rclcpp::Publisher<robot_interfaces::msg::QtRecv>::SharedPtr publisher_;         // joint_states
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_gripper_states_;     // gripper states
        rclcpp::Subscription<robot_interfaces::msg::ArmState>::SharedPtr subscriber_states_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_joint_states_;

        void acceptLoop() {
            while (rclcpp::ok()) {
                struct sockaddr_in address;
                socklen_t addrlen = sizeof(address);
                int new_socket = accept(server_fd_, (struct sockaddr *)&address, &addrlen);
                if (new_socket < 0) {
                    RCLCPP_ERROR(this->get_logger(), "Accept failed!");
                    continue;
                }

                RCLCPP_INFO(this->get_logger(), "Connection accepted from %s", inet_ntoa(address.sin_addr));

                // 处理新连接的请求
                std::thread client_thread(&TcpServer::handleConnection, this, new_socket);
                client_thread.detach(); // 分离线程，使其独立运行
            }
        }

        void handleConnection(int client_fd) {
            client_fd_ = client_fd;
            const int bufferSize = 50; 
            unsigned char buffer[bufferSize] = {0};

            while (true) {
                int valread = read(client_fd, buffer, bufferSize);
                // RCLCPP_INFO(this->get_logger(), "valread: %d", valread);
                if (valread > 0) {     
                    RCLCPP_INFO(this->get_logger(), "received data len: %d", valread);               
                    for (int i = 0; i < valread/4; ++i) {
                        receivedValue[i] = *(int*)(buffer + i * sizeof(int));
                        RCLCPP_INFO(this->get_logger(), "Received int: %d", receivedValue[i]);
                    }
                    RCLCPP_INFO(this->get_logger(), "--------------------------");
                    handleCommand(receivedValue);
                } else if (valread == 0) {
                    // 客户端关闭连接
                    RCLCPP_INFO(this->get_logger(), "Client disconnected");
                    qt_cmd.working_mode = 0x06;     // 急停
                    publisher_->publish(qt_cmd);
                    break;
                } else {
                    // 读取错误
                    RCLCPP_ERROR(this->get_logger(), "Read error: %s", strerror(errno));
                    qt_cmd.working_mode = 0x06;     // 急停
                    publisher_->publish(qt_cmd);
                    break;
                }
            }

            close(client_fd);
        }

        void handleCommand(const int *data) {
            switch (data[0]) {
                case 0x00: {    // Functions
                    qt_cmd.working_mode = data[3];
                    publisher_->publish(qt_cmd);
                    RCLCPP_INFO(this->get_logger(), "Mode: %d", qt_cmd.working_mode);
                    break;
                }
                case 0x01: {    // z
                    qt_cmd.working_mode = 0x08;
                    qt_cmd.arm_pose_goal.position = end_effector_pose.position;
                    qt_cmd.arm_pose_goal.orientation = end_effector_pose.orientation;
                    // qt_cmd.gripper_goal.data = {data[4], data[5], data[6]};
                    if (data[1] == 0x01) {           // Position                  
                        if (data[2] == 0x01) {           // +
                            qt_cmd.arm_pose_goal.position.z = end_effector_pose.position.z + 0.01 * data[3];
                        } else if (data[2] == 0x00) {    // -
                            qt_cmd.arm_pose_goal.position.z = end_effector_pose.position.z - 0.01 * data[3];
                        }
                        RCLCPP_INFO(this->get_logger(), "New Position: %lf, %lf, %lf", 
                                                            qt_cmd.arm_pose_goal.position.x, 
                                                            qt_cmd.arm_pose_goal.position.y, 
                                                            qt_cmd.arm_pose_goal.position.z);
                    } 
                    else if (data[1] == 0x02) {    // Rotation
                        // 绕z轴旋转的旋转四元数
                        tf2::Quaternion rotation_quat;
                        if (data[2] == 0x01) {           // +
                            RCLCPP_INFO(this->get_logger(), "zrot+%d", data[3]);
                            rotation_quat.setRPY(0, 0, data[3] * 5 * M_PI / 180.0);
                            // 旋转后的新四元数
                            tf2::Quaternion new_quat = rotation_quat * end_effector_quat;
                            new_quat.normalize();
                            // 新四元数转换回geometry_msgs::msg::Quaternion
                            qt_cmd.arm_pose_goal.orientation = tf2::toMsg(new_quat);
                        } else if (data[2] == 0x00) {    // -
                            RCLCPP_INFO(this->get_logger(), "zrot-%d", data[3]);
                            rotation_quat.setRPY(0, 0, -data[3] * 5 * M_PI / 180.0); 
                            // 旋转后的新四元数
                            tf2::Quaternion new_quat = rotation_quat * end_effector_quat;
                            new_quat.normalize();
                            // 新四元数转换回geometry_msgs::msg::Quaternion
                            qt_cmd.arm_pose_goal.orientation = tf2::toMsg(new_quat);
                        }
                    }
                    publisher_->publish(qt_cmd);
                    break;
                }
                case 0x02: {    // x
                    qt_cmd.working_mode = 0x08;
                    qt_cmd.arm_pose_goal.position = end_effector_pose.position;
                    qt_cmd.arm_pose_goal.orientation = end_effector_pose.orientation;
                    // qt_cmd.gripper_goal.data = {data[4], data[5], data[6]};
                    if (data[1] == 0x01) {           // Position
                        if (data[2] == 0x01) {           // +
                            RCLCPP_INFO(this->get_logger(), "xpos+%d", data[3]);
                            qt_cmd.arm_pose_goal.position.x = end_effector_pose.position.x + 0.01 * data[3];                                    
                        } else if (data[2] == 0x00) {    // -
                            RCLCPP_INFO(this->get_logger(), "xpos-%d", data[3]);
                            qt_cmd.arm_pose_goal.position.x = end_effector_pose.position.x - 0.01 * data[3];
                        }
                    } else if (data[1] == 0x02) {    // Rotation
                        // 绕x轴旋转的旋转四元数
                        tf2::Quaternion rotation_quat;
                        if (data[2] == 0x01) {           // +
                            RCLCPP_INFO(this->get_logger(), "xrot+%d", data[3]);                                    
                            rotation_quat.setRPY(data[3] * 5 * M_PI / 180.0, 0, 0);
                            // 旋转后的新四元数
                            tf2::Quaternion new_quat = rotation_quat * end_effector_quat;
                            new_quat.normalize();
                            // 新四元数转换回geometry_msgs::msg::Quaternion
                            qt_cmd.arm_pose_goal.orientation = tf2::toMsg(new_quat);
                        } else if (data[2] == 0x00) {    // -
                            RCLCPP_INFO(this->get_logger(), "xrot-%d", data[3]);
                            rotation_quat.setRPY(-data[3] * 5 * M_PI / 180.0, 0, 0);
                            // 旋转后的新四元数
                            tf2::Quaternion new_quat = rotation_quat * end_effector_quat;
                            new_quat.normalize();
                            // 新四元数转换回geometry_msgs::msg::Quaternion
                            qt_cmd.arm_pose_goal.orientation = tf2::toMsg(new_quat);
                        }
                    }
                    publisher_->publish(qt_cmd);
                    break;
                }
                case 0x03: {    // y
                    qt_cmd.working_mode = 0x08;
                    qt_cmd.arm_pose_goal.position = end_effector_pose.position;
                    qt_cmd.arm_pose_goal.orientation = end_effector_pose.orientation;
                    // qt_cmd.gripper_goal.data = {data[4], data[5], data[6]};
                    if (data[1] == 0x01) {           // Position
                        if (data[2] == 0x01) {           // +
                            RCLCPP_INFO(this->get_logger(), "ypos+%d", data[3]);
                            qt_cmd.arm_pose_goal.position.y = end_effector_pose.position.y + 0.01 * data[3];
                        } else if (data[2] == 0x00) {    // -
                            RCLCPP_INFO(this->get_logger(), "ypos-%d", data[3]);
                            qt_cmd.arm_pose_goal.position.y = end_effector_pose.position.y - 0.01 * data[3];
                        }
                    } else if (data[1] == 0x02) {    // Rotation
                        // 绕y轴旋转的旋转四元数
                        tf2::Quaternion rotation_quat;
                        if (data[2] == 0x01) {           // +
                            RCLCPP_INFO(this->get_logger(), "yrot+%d", data[3]);
                            rotation_quat.setRPY(0, data[3] * 5 * M_PI / 180.0, 0);
                            // 旋转后的新四元数
                            tf2::Quaternion new_quat = rotation_quat * end_effector_quat;
                            new_quat.normalize();
                            // 新四元数转换回geometry_msgs::msg::Quaternion
                            qt_cmd.arm_pose_goal.orientation = tf2::toMsg(new_quat);
                        } else if (data[2] == 0x00) {    // -
                            RCLCPP_INFO(this->get_logger(), "yrot-%d", data[3]);
                            rotation_quat.setRPY(0, -data[3] * 5 * M_PI / 180.0, 0);
                            // 旋转后的新四元数
                            tf2::Quaternion new_quat = rotation_quat * end_effector_quat;
                            new_quat.normalize();
                            // 新四元数转换回geometry_msgs::msg::Quaternion
                            qt_cmd.arm_pose_goal.orientation = tf2::toMsg(new_quat);
                        }
                    }
                    publisher_->publish(qt_cmd);
                    break;
                }
                case 1145: {
                    // RCLCPP_INFO(this->get_logger(), "%d, %d, %d", data[1], data[2], data[3]);
                    // RCLCPP_INFO(this->get_logger(), "%d, %d, %d", data[4], data[5], data[6]);
                    // RCLCPP_INFO(this->get_logger(), "%d, %d, %d", data[7], data[8], data[9]);

                    qt_cmd.joint_angles_goal.data = {data[1], data[2], data[3], data[4], data[5], data[6]};
                    // qt_cmd.gripper_goal.data = {data[7], data[8], data[9]};
                    qt_cmd.working_mode = 0x09;
                    qt_cmd.arm_pose_goal = end_effector_pose;
                    publisher_->publish(qt_cmd);       
                    RCLCPP_INFO(this->get_logger(), "qt_cmd topic has been published");
                    break;
                }
                case 1245: {
                    gripper_states.data.clear();
                    gripper_states.data.push_back(data[1]);
                    gripper_states.data.push_back(data[2]);
                    gripper_states.data.push_back(data[3]);
                    publisher_gripper_states_->publish(gripper_states);
                    break;
                }
                default:
                    RCLCPP_ERROR(this->get_logger(),"error receivedID");
                    break;
            }
        }

        void arm_states_callback(const robot_interfaces::msg::ArmState::SharedPtr msg)
        {
            //tf2::fromMsg(msg->end_effector_quat, end_effector_quat);
            tf2::fromMsg(msg->end_effector_pose.orientation, end_effector_quat);
            end_effector_pose = msg->end_effector_pose;
        }
        
        void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            if (client_fd_ <= 0) {
                // RCLCPP_INFO(this->get_logger(), "No client connected to send joint_states");
                return;
            }
            std::vector<double> positions = msg->position;
            std::vector<double> velocities = msg->velocity;
            std::vector<double> efforts = msg->effort;

            int positionSize = positions.size() * sizeof(double);
            int effortSize = efforts.size() * sizeof(double);
            int velocitySize = velocities.size() * sizeof(double);
            int dataSize = positionSize + effortSize + velocitySize;

            std::vector<unsigned char> buffer(dataSize);
            memcpy(buffer.data(), positions.data(), positionSize);
            memcpy(buffer.data() + positionSize, efforts.data(), effortSize);
            memcpy(buffer.data() + positionSize + effortSize, velocities.data(), velocitySize);
            
            send(client_fd_, buffer.data(), dataSize, 0);
            // RCLCPP_INFO(this->get_logger(), "Sent joint positions and efforts to client");
        }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TcpServer>("tcp_server");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
