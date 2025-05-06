#include <memory>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/buffer_core.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <robot_interfaces/msg/qt_recv.hpp>
#include <robot_interfaces/msg/arm_state.hpp>

/* 
 * @brief 这是一个超级无敌宇宙大爆炸没用的注释
 * @details 这是一个超级无敌宇宙大爆炸没用的类
*/
class RobotStateGet : public rclcpp :: Node
{
    public :
        RobotStateGet(const std::string& node_name) : Node(node_name), tf_buffer_(), tf_listener_(tf_buffer_) {
            timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RobotStateGet::timer_callback, this));
            publisher_arm_pose_ = this->create_publisher<robot_interfaces::msg::ArmState>("arm_states", 10);
            publisher_gripper_pose_= this->create_publisher<robot_interfaces::msg::ArmState>("gripper_poses", 10);
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        }
    private :
        tf2::BufferCore tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<robot_interfaces::msg::ArmState>::SharedPtr publisher_arm_pose_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::Publisher<robot_interfaces::msg::ArmState>::SharedPtr publisher_gripper_pose_;

        void timer_callback()
        {
            geometry_msgs::msg::TransformStamped transform;
            try {
                transform = tf_buffer_.lookupTransform("world", "Link6", tf2::TimePointZero);

                robot_interfaces::msg::ArmState arm_pose;
                arm_pose.end_effector_pose.position.x = transform.transform.translation.x;
                arm_pose.end_effector_pose.position.y = transform.transform.translation.y;
                arm_pose.end_effector_pose.position.z = transform.transform.translation.z;
                arm_pose.end_effector_pose.orientation = transform.transform.rotation;

                // RCLCPP_INFO(this->get_logger(), "Get the arm position: %lf, %lf, %lf. Get the arm orientation: %lf, %lf, %lf, %lf", 
                //                                     arm_pose.end_effector_pose.position.x, 
                //                                     arm_pose.end_effector_pose.position.y,
                //                                     arm_pose.end_effector_pose.position.z,
                //                                     arm_pose.end_effector_pose.orientation.w,
                //                                     arm_pose.end_effector_pose.orientation.x,
                //                                     arm_pose.end_effector_pose.orientation.y,
                //                                     arm_pose.end_effector_pose.orientation.z);
                publisher_arm_pose_->publish(arm_pose);

                // 计算爪子末端位姿
                tf2::Vector3 offset(0, 0, 0.175); // 位移向量 (17.5cm)
                tf2::Quaternion orientation;
                tf2::fromMsg(transform.transform.rotation, orientation);
                tf2::Transform tf_transform(orientation);
                tf2::Vector3 gripper_position = tf_transform * offset + tf2::Vector3(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);

                robot_interfaces::msg::ArmState gripper_pose;
                gripper_pose.end_effector_pose.position.x = gripper_position.x();
                gripper_pose.end_effector_pose.position.y = gripper_position.y();
                gripper_pose.end_effector_pose.position.z = gripper_position.z();
                gripper_pose.end_effector_pose.orientation = transform.transform.rotation;

                // RCLCPP_INFO(this->get_logger(), "Get the gripper position: %lf, %lf, %lf. Get the gripper orientation: %lf, %lf, %lf, %lf", 
                //                                     gripper_pose.end_effector_pose.position.x, 
                //                                     gripper_pose.end_effector_pose.position.y,
                //                                     gripper_pose.end_effector_pose.position.z,
                //                                     gripper_pose.end_effector_pose.orientation.w,
                //                                     gripper_pose.end_effector_pose.orientation.x,
                //                                     gripper_pose.end_effector_pose.orientation.y,
                //                                     gripper_pose.end_effector_pose.orientation.z);
                publisher_gripper_pose_->publish(gripper_pose);

                double x = 0.0317196, y = -0.020996, z = 0.750743;
                double rx = -28.0715, ry = -23.8247, rz = -156.766;

                // 将欧拉角转换为四元数
                tf2::Quaternion q;
                q.setRPY(rx * M_PI / 180.0, ry * M_PI / 180.0, rz * M_PI / 180.0);
                // 创建从夹爪到相机的变换
                tf2::Transform world_to_camera_transform(q, tf2::Vector3(x, y, z));
                // 获取夹爪在世界坐标系中的变换
                // tf2::Transform world_to_gripper_transform;
                // world_to_gripper_transform.setOrigin(tf2::Vector3(
                //     gripper_pose.end_effector_pose.position.x,
                //     gripper_pose.end_effector_pose.position.y,
                //     gripper_pose.end_effector_pose.position.z));
                // world_to_gripper_transform.setRotation(tf2::Quaternion(
                //     gripper_pose.end_effector_pose.orientation.x,
                //     gripper_pose.end_effector_pose.orientation.y,
                //     gripper_pose.end_effector_pose.orientation.z,
                //     gripper_pose.end_effector_pose.orientation.w));
                // // 计算相机在世界坐标系中的变换
                // tf2::Transform world_to_camera_transform = world_to_gripper_transform * gripper_to_camera_transform;

                // 将变换发布到 /tf
                geometry_msgs::msg::TransformStamped camera_transform;
                camera_transform.header.stamp = this->get_clock()->now();
                camera_transform.header.frame_id = "base_link";
                camera_transform.child_frame_id = "camera";
                camera_transform.transform = tf2::toMsg(world_to_camera_transform);

                tf_broadcaster_->sendTransform(camera_transform);
            } catch (tf2::TransformException &ex) {
                // RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
            }
        }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotStateGet>("robot_state_get");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
