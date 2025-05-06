#ifndef __ROBOT_CONTROL_H__
#define __ROBOT_CONTROL_H__

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;

class RobotControl : public rclcpp::Node {
    public:
        using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
        using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
        explicit RobotControl(std::string name);
        ~RobotControl(){}

        void timer_callback();

    private:
        sensor_msgs::msg::JointState joint_msg;

        // bool follow_ = false;
        rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pos_publisher;
        rclcpp::TimerBase::SharedPtr State_Timer;

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
        void execute_move(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
        void handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
};      
#endif  /* __ROBOT_CONTROL_H__ */