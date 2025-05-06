#include "robot_key_controller.h"

KeyboardController::KeyboardController(std::string node_name) : Node(node_name) {
    this->declare_parameter<std::string>("robot_model_name", "arm620");

    load_joint_limits();
    this->declare_parameter<double>("speed_scaling", 0.1);
    double init_speed_scaling;
    this->get_parameter("speed_scaling", init_speed_scaling);

    arm = std::make_shared<moveit::planning_interface::MoveGroupInterface>(rclcpp::Node::SharedPtr(this), std::string("arm"));
    mode_sub = this->create_subscription<std_msgs::msg::UInt8>("robotic_arm_control_mode", 10, std::bind(&KeyboardController::working_mode_callback, this, std::placeholders::_1));
    motor_msg_pub = this->create_publisher<robot_interfaces::msg::RobotControlMsg>("motor_control_msg", 10);
    gripper_msg_pub = this->create_publisher<std_msgs::msg::UInt8MultiArray>("gripper_control_msg", 10);
    
    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&KeyboardController::joint_states_callback, this, std::placeholders::_1));
    // 设置机械臂运动允许的误差
    arm->setGoalJointTolerance(0.0007);
    // 设置机械臂运动参数
    arm->setMaxVelocityScalingFactor(init_speed_scaling);  // 初始值
    arm->setMaxAccelerationScalingFactor(init_speed_scaling);
    arm->setPoseReferenceFrame("base_link");
    arm->allowReplanning(true);
    arm->setGoalPositionTolerance(0.0005);
    arm->setGoalOrientationTolerance(0.0007);

    current_count_ = 0;
    recording_ = false;
    teach_bag_writer_ = nullptr;
    play_bag_reader_ = nullptr;

    // 注册参数修改回调
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> & parameters) -> rcl_interfaces::msg::SetParametersResult {
            for (const auto & param : parameters) {
                if (param.get_name() == "speed_scaling") {
                    speed_scaling_ = param.as_double();
                    arm->setMaxVelocityScalingFactor(speed_scaling_);
                    arm->setMaxAccelerationScalingFactor(speed_scaling_);
                    RCLCPP_INFO(this->get_logger(), "Speed scaling set to: %f", speed_scaling_);
                }
            }
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            return result;
        }
    );
}

void KeyboardController::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg->effort.size() < 6 || msg->position.size() < 6) {
        RCLCPP_WARN(this->get_logger(), "JointStates data in complete! Expected 6, got effort: %zu, position: %zu", msg->effort.size(), msg->position.size());
        return;
    }
    latest_joint_states_ = msg;

    if (recording_ && teach_bag_writer_) {
        rclcpp::Serialization<sensor_msgs::msg::JointState> serializer;
        auto serialized_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        serialized_msg->topic_name = "joint_states";

        rclcpp::SerializedMessage serialized_ros_message;
        serializer.serialize_message(msg.get(), &serialized_ros_message);

        serialized_msg->serialized_data = std::make_shared<rcutils_uint8_array_t>(serialized_ros_message.get_rcl_serialized_message());
        serialized_msg->time_stamp = msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;

        teach_bag_writer_->write(serialized_msg);

        RCLCPP_INFO(this->get_logger(), "JointState recorded...");
    }
}

void KeyboardController::working_mode_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
    switch (msg->data) {
        case BACKTOSTART: {         // 回到初始位置
            rbt_ctrl_msg.motor_enable_flag.resize(6, true);
            rbt_ctrl_msg.motor_mode.resize(6, POSITION_MODE);
            rbt_ctrl_msg.motor_msg.resize(6, 0.0f);
            motor_msg_pub->publish(rbt_ctrl_msg);
            break;
        }
        case DISABLE: {             
            rbt_ctrl_msg.motor_enable_flag.resize(6, false);
            rbt_ctrl_msg.motor_mode.resize(6, VELOCITY_MODE);
            rbt_ctrl_msg.motor_msg.resize(6, 0.0f);
            motor_msg_pub->publish(rbt_ctrl_msg);
            break;
        }
        case JOINTCONTROL: {         // 关节空间速度控制，可以通过长按键盘直接地给定机械臂六个关节运动的速度
            jointctrl_action_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&KeyboardController::jointctrl_timer_callback, this));
            jointctrl_action_sub_ = this->create_subscription<std_msgs::msg::UInt8>("jointctrl_action", 10, std::bind(&KeyboardController::jointctrl_action_callback, this, std::placeholders::_1));
            break;
        }
        case CARTESIAN: {            // 笛卡尔空间控制，可以通过键盘给定机械臂末端的期望位置与姿态的运动速度
            cartesian_action_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&KeyboardController::cartesian_timer_callback, this));
            cartesian_action_sub_ = this->create_subscription<std_msgs::msg::UInt8>("cartesian_action", 10, std::bind(&KeyboardController::cartesian_action_callback, this, std::placeholders::_1));
            break;
        }
        case MOVEJ: {
            movej_action_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("movej_action", 10, std::bind(&KeyboardController::movej_action_callback, this, std::placeholders::_1));
            break;
        }
        case MOVEL: {
            movel_action_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("movel_action", 10, std::bind(&KeyboardController::movel_action_callback, this, std::placeholders::_1));
            break;
        }
        case MOVEC: {
            movec_action_sub_ = this->create_subscription<robot_interfaces::msg::MoveCAction>("movec_action", 10, std::bind(&KeyboardController::movec_action_callback, this, std::placeholders::_1));
            break;
        }
        case TEACH: {            
            RCLCPP_INFO(this->get_logger(), "Robot arm is at TEACH mode!");
            teach_motor_mode();
            teach_action_sub_ = this->create_subscription<std_msgs::msg::String>("teach_action", 10, std::bind(&KeyboardController::teach_action_callback, this, std::placeholders::_1));
            break;
        }
        case TEACHREPEAT: {
            teach_repeat_sub_ = this->create_subscription<std_msgs::msg::String>("teachrepeat_action", 10, std::bind(&KeyboardController::teach_repeat_callback, this, std::placeholders::_1));
            break;
        }
        case SAVESTATE: {
            RCLCPP_INFO(this->get_logger(), "Saving current robotic arm state!");
            save_state_sub_ = this->create_subscription<std_msgs::msg::String>("savestate_action", 10, std::bind(&KeyboardController::save_state_callback, this, std::placeholders::_1));
            break;
        }
        case LOADSTATE: {
            RCLCPP_INFO(this->get_logger(), "Loading saved robotic arm state!");
            load_state_sub_ = this->create_subscription<std_msgs::msg::String>("loadstate_action", 10, std::bind(&KeyboardController::load_state_callback, this, std::placeholders::_1));
            break;
        }
        case BACKTOINITIAL: {
            rbt_ctrl_msg.motor_enable_flag.resize(6, true);
            rbt_ctrl_msg.motor_mode.resize(6, POSITION_MODE);
            rbt_ctrl_msg.motor_msg.resize(6);
            rbt_ctrl_msg.motor_msg[0] = 0.0;
            rbt_ctrl_msg.motor_msg[1] = 0.43633f;
            rbt_ctrl_msg.motor_msg[2] = 0.43633f;
            rbt_ctrl_msg.motor_msg[3] = 0.0f;
            rbt_ctrl_msg.motor_msg[4] = 0.0f;
            rbt_ctrl_msg.motor_msg[5] = 0.0f;
            motor_msg_pub->publish(rbt_ctrl_msg);
            break;
        }
        default:
            break;
    }
}

void KeyboardController::jointctrl_timer_callback() {
    if (jointctrl_action_sub_ && (this->now() - last_jointctrl_msg_time).seconds() > 0.05) {    // 50ms超时时间
        rbt_ctrl_msg.motor_enable_flag.resize(6, true);
        rbt_ctrl_msg.motor_mode.resize(6, VELOCITY_MODE);
        rbt_ctrl_msg.motor_msg.resize(6, 0.0f);
        motor_msg_pub->publish(rbt_ctrl_msg);
    }
}

void KeyboardController::load_joint_limits() {
    std::string robot_model_name;
    this->get_parameter("robot_model_name", robot_model_name);

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("robot_description");

    std::string urdf_file_path = package_share_directory + "/urdf" + robot_model_name + ".urdf";
    RCLCPP_INFO(this->get_logger(), "Load URDF file: %s", urdf_file_path.c_str());

    std::ifstream urdf_file(urdf_file_path);
    if (!urdf_file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open URDF file: %s", urdf_file_path.c_str());
        return;
    }

    std::stringstream urdf_string_stream;
    urdf_string_stream << urdf_file.rdbuf();
    std::string urdf_content = urdf_string_stream.str();
    urdf_file.close();

    urdf::Model model;
    if (!model.initString(urdf_content)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF file.");
        return;
    }

    for (const auto& joint_pair : model.joints_) {
        const auto& joint = joint_pair.second;
        if (joint->type == urdf::Joint::REVOLUTE) {
            if (joint->limits) {
                joint_limits_[joint->name] = {joint->limits->lower, joint->limits->upper};
                RCLCPP_INFO(this->get_logger(), "Loaded limit for joint [%s]: [%f, %f]", 
                            joint->name.c_str(), joint->limits->lower, joint->limits->upper);
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "Successfully loaded joint limits from file: %s", urdf_file_path.c_str());
}


void KeyboardController::jointctrl_motor_mode(const uint8_t joint_index, const float speed) {
    rbt_ctrl_msg.motor_enable_flag.resize(6, true);
    rbt_ctrl_msg.motor_mode.resize(6, VELOCITY_MODE);
    rbt_ctrl_msg.motor_msg.resize(6);

    for (size_t i = 0; i < rbt_ctrl_msg.motor_msg.size(); i++) {
        float command_speed = (i == joint_index) ? speed : 0.0f;

        if (latest_joint_states_ && i < latest_joint_states_->position.size()) {
            std::string joint_name = latest_joint_states_->name[i];
            double position = latest_joint_states_->position[i];
            auto it = joint_limits_.find(joint_name);
            if (it != joint_limits_.end()) {
                // 3 degrees of tolerance
                double lower_limit = it->second.first + 0.05236f;
                double upper_limit = it->second.second - 0.05236f;

                if (position <= lower_limit && command_speed < 0.0f) {   
                    command_speed = 0.0f;
                } else if (position >= upper_limit && command_speed > 0.0f) {
                    command_speed = 0.0f;
                }
            }
        }
        rbt_ctrl_msg.motor_msg[i] = command_speed;
    }

    motor_msg_pub->publish(rbt_ctrl_msg);
}


void KeyboardController::jointctrl_action_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
    last_jointctrl_msg_time = this->now();
    switch (msg->data) {
        case JOINT1_POSITIVE:
        case JOINT1_NEGATIVE:
        case JOINT2_POSITIVE: 
        case JOINT2_NEGATIVE:
        case JOINT3_POSITIVE: 
        case JOINT3_NEGATIVE:
        case JOINT4_POSITIVE:
        case JOINT4_NEGATIVE:
        case JOINT5_POSITIVE:
        case JOINT5_NEGATIVE:
        case JOINT6_POSITIVE:
        case JOINT6_NEGATIVE:
            jointctrl_motor_mode(msg->data - JOINT1_POSITIVE, (msg->data & 1) ? JOINT_ROTATE_POSITIVE_SPEED : JOINT_ROTATE_NEGATIVE_SPEED);
            break;
        case GRIPPER_OPEN:
        case GRIPPER_CLOSE:
            adjust_gripper(msg->data);
            break;
        default: {
            break;
        }
    }
}


void KeyboardController::cartesian_timer_callback() {
    if (cartesian_action_sub_ && (this->now() - last_jointctrl_msg_time).seconds() > 0.5) {
        rbt_ctrl_msg.motor_enable_flag.resize(6, true);
        rbt_ctrl_msg.motor_mode.resize(6, VELOCITY_MODE);
        rbt_ctrl_msg.motor_msg.resize(6, 0.0f);
        motor_msg_pub->publish(rbt_ctrl_msg);
    }
}


void KeyboardController::move_arm_in_direction(const geometry_msgs::msg::Pose& current_pose, int direction) {
    geometry_msgs::msg::Pose arm_goal_pose = current_pose;
    double delta = (direction == FORWARD || direction == RIGHT || direction == UP) ? 0.01 : -0.01;

    arm->setMaxAccelerationScalingFactor(speed_scaling_);
    arm->setMaxVelocityScalingFactor(speed_scaling_);

    switch (direction) {
        case FORWARD: 
        case BACKWARD: 
            arm_goal_pose.position.x += delta;
            break;
        case LEFT:
        case RIGHT: 
            arm_goal_pose.position.y += delta;
            break;
        case UP:
        case DOWN: 
            arm_goal_pose.position.z += delta;
            break;
        default:
            break;
    }

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(arm_goal_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = arm->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    if (fraction > 0.9) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        arm->execute(plan);
    } else {
        RCLCPP_ERROR(this->get_logger(), "MoveIt failed to plan the Cartesian motion in direction %d", direction);
    }
}


void KeyboardController::move_arm_in_orientation(const geometry_msgs::msg::Pose& current_pose, int direction) {
    geometry_msgs::msg::Pose arm_goal_pose = current_pose;
    double angle = (direction == ROLL_POSITIVE || direction == PITCH_POSITIVE || direction == YAW_POSITIVE) ? 0.017453f : -0.017453f;

    arm->setMaxAccelerationScalingFactor(speed_scaling_);
    arm->setMaxVelocityScalingFactor(speed_scaling_);

    tf2::Quaternion q;
    switch (direction) {
        case ROLL_POSITIVE:
        case ROLL_NEGATIVE:
            q.setRPY(angle, 0.0, 0.0);
            break;
        case PITCH_POSITIVE:
        case PITCH_NEGATIVE:
            q.setRPY(0.0, angle, 0.0);
            break;
        case YAW_POSITIVE:
        case YAW_NEGATIVE:
            q.setRPY(0.0, 0.0, angle);
            break;
        default:
            break;
    }

    tf2::Quaternion q_current;
    tf2::fromMsg(current_pose.orientation, q_current);
    tf2::Quaternion q_goal = q * q_current;
    q_goal.normalize();
    arm_goal_pose.orientation = tf2::toMsg(q_goal);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(arm_goal_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = arm->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    if (fraction > 0.9) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        arm->execute(plan);
    } else {
        RCLCPP_ERROR(this->get_logger(), "MoveIt failed to plan the Cartesian motion in orientation %d", direction);
    }
}


void KeyboardController::adjust_gripper(int action) {
    std_msgs::msg::UInt8MultiArray gripper_states;
    int delta = (action == GRIPPER_OPEN) ? -1 : 1;

    int new_position_value = current_gripper_states_[0] + delta;
    current_gripper_states_[0] = std::max(0, std::min(new_position_value, 255));
    current_gripper_states_[1] = 20;
    current_gripper_states_[2] = 0;

    gripper_states.data.clear();
    for (int state : current_gripper_states_) {
        gripper_states.data.push_back(state);
    }
    gripper_msg_pub->publish(gripper_states);
}


void KeyboardController::cartesian_action_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
    geometry_msgs::msg::Pose current_pose = arm->getCurrentPose().pose;
    geometry_msgs::msg::Pose arm_goal_pose;

    switch (msg->data) {
        case FORWARD:
        case BACKWARD:
        case LEFT:
        case RIGHT:
        case UP:
        case DOWN:
            move_arm_in_direction(current_pose, msg->data);
            break;
        case ROLL_POSITIVE:
        case ROLL_NEGATIVE: 
        case PITCH_POSITIVE:
        case PITCH_NEGATIVE:
        case YAW_POSITIVE:
        case YAW_NEGATIVE:
            move_arm_in_orientation(current_pose, msg->data);
            break;
        case GRIPPER_OPEN:
        case GRIPPER_CLOSE:
            adjust_gripper(msg->data);
            break;
        default:
            break;
    }
}


void KeyboardController::movej_action_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    std::vector<double> joint_group_positions;
    
    arm->setMaxAccelerationScalingFactor(speed_scaling_);
    arm->setMaxVelocityScalingFactor(speed_scaling_);

    if (msg->data.size() % 6 != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error input joint angle values!");
    } else {
        joint_group_positions.resize(6);
        for (size_t i = 0; i < msg->data.size(); i += 6) {
            for (size_t j = 0; j < 6; j++) {
                joint_group_positions[j] = msg->data[i + j];
            }
            arm->setJointValueTarget(joint_group_positions);
            moveit_msgs::msg::MoveItErrorCodes move_result = arm->move();
            if (move_result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                RCLCPP_ERROR(this->get_logger(), "MoveIt error: %d", move_result.val);
            } else {
                RCLCPP_INFO(this->get_logger(), "Joint angle movements group[%zu] are completed.", i / 6 + 1);
            }
        }
    }
}


void KeyboardController::movel_action_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() % 6 != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error input pose values!");
        return;
    } 
    std::vector<geometry_msgs::msg::Pose> waypoints;

    arm->setMaxAccelerationScalingFactor(speed_scaling_);
    arm->setMaxVelocityScalingFactor(speed_scaling_);
    
    for (size_t i = 0; i < msg->data.size(); i += 6) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = msg->data[i];
        pose.position.y = msg->data[i + 1];
        pose.position.z = msg->data[i + 2];

        double roll = msg->data[i + 3];
        double pitch = msg->data[i + 4];
        double yaw = msg->data[i + 5];

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        pose.orientation = tf2::toMsg(q);

        waypoints.push_back(pose);
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = arm->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    if (fraction > 0.9) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        arm->execute(plan);
    } else {
        RCLCPP_ERROR(this->get_logger(), "MoveIt failed to plan the trajectory through waypoints!");
    }
}


void KeyboardController::movec_action_callback(const robot_interfaces::msg::MoveCAction::SharedPtr msg) {
    auto pose_array = msg->pose_array;
    auto must_pass_through_middle = msg->must_pass_through_middle;
    
    if (pose_array.data.size() != 12) {
        RCLCPP_ERROR(this->get_logger(), "Error input pose values!");
        return;
    }

    geometry_msgs::msg::Pose arm_goal_pose, arm_medium_pose;
    arm->setMaxAccelerationScalingFactor(speed_scaling_);
    arm->setMaxVelocityScalingFactor(speed_scaling_);

    // 中间点
    arm_medium_pose.position.x = pose_array.data[0];
    arm_medium_pose.position.y = pose_array.data[1];
    arm_medium_pose.position.z = pose_array.data[2];
    double roll_medium = pose_array.data[3];
    double pitch_medium = pose_array.data[4];
    double yaw_medium = pose_array.data[5];
    tf2::Quaternion quat_tf_medium;
    quat_tf_medium.setRPY(roll_medium, pitch_medium, yaw_medium);
    arm_medium_pose.orientation = tf2::toMsg(quat_tf_medium);

    // 目标点
    arm_goal_pose.position.x = pose_array.data[6];
    arm_goal_pose.position.y = pose_array.data[7];
    arm_goal_pose.position.z = pose_array.data[8];
    double roll_goal = pose_array.data[9];
    double pitch_goal = pose_array.data[10];
    double yaw_goal = pose_array.data[11];
    tf2::Quaternion quat_tf_goal;
    quat_tf_goal.setRPY(roll_goal, pitch_goal, yaw_goal);
    arm_goal_pose.orientation = tf2::toMsg(quat_tf_goal);

    // 获取当前末端位姿
    geometry_msgs::msg::Pose current_pose = arm->getCurrentPose().pose; 

    // 插值准备
    tf2::Quaternion start_q;
    tf2::fromMsg(current_pose.orientation, start_q);
    tf2::Quaternion goal_q;
    tf2::fromMsg(arm_goal_pose.orientation, goal_q);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    const int num_points = 30;

    if (must_pass_through_middle.data) {
        // 必经点插值: 辅助点位置: current->medium之间40%的位置
        geometry_msgs::msg::Point assist_point;
        assist_point.x = current_pose.position.x + 0.4 * (arm_medium_pose.position.x - current_pose.position.x);
        assist_point.y = current_pose.position.y + 0.4 * (arm_medium_pose.position.y - current_pose.position.y);
        assist_point.z = current_pose.position.z + 0.4 * (arm_medium_pose.position.z - current_pose.position.z);
    
        // 第一段: current -> assist_point -> arm_medium_pose
        for (int i = 0; i < num_points / 2; ++i) {
            double t = static_cast<double>(i) / (num_points / 2);
            geometry_msgs::msg::Pose pose;
            pose.position.x = pow(1 - t, 2) * current_pose.position.x + 2 * (1 - t) * t * assist_point.x + pow(t, 2) * arm_medium_pose.position.x;
            pose.position.y = pow(1 - t, 2) * current_pose.position.y + 2 * (1 - t) * t * assist_point.y + pow(t, 2) * arm_medium_pose.position.y;
            pose.position.z = pow(1 - t, 2) * current_pose.position.z + 2 * (1 - t) * t * assist_point.z + pow(t, 2) * arm_medium_pose.position.z;

            tf2::Quaternion interpolated_q = start_q.slerp(start_q, t);
            pose.orientation = tf2::toMsg(interpolated_q);

            waypoints.push_back(pose);
        }

        // 第二段: arm_medium_pose -> arm_goal_pose
        for (int i = 1; i <= num_points / 2; ++i) {     // i从1开始，避免重复添加arm_medium_pose
            double t = static_cast<double>(i) / (num_points / 2);
            geometry_msgs::msg::Pose pose;
            pose.position.x = (1 - t) * arm_medium_pose.position.x + t * arm_goal_pose.position.x;
            pose.position.y = (1 - t) * arm_medium_pose.position.y + t * arm_goal_pose.position.y;
            pose.position.z = (1 - t) * arm_medium_pose.position.z + t * arm_goal_pose.position.z;

            tf2::Quaternion mid_q;
            tf2::fromMsg(arm_medium_pose.orientation, mid_q);
            tf2::Quaternion interpolated_q = mid_q.slerp(goal_q, t);
            pose.orientation = tf2::toMsg(interpolated_q);

            waypoints.push_back(pose);
        }
    } else {    // 非必经点插值
        for (int i = 0; i < num_points; ++i) {
            double t = static_cast<double>(i) / num_points;
            geometry_msgs::msg::Pose pose;
            pose.position.x = pow(1 - t, 2) * current_pose.position.x + 2 * (1 - t) * t * arm_medium_pose.position.x + pow(t, 2) * arm_goal_pose.position.x;
            pose.position.y = pow(1 - t, 2) * current_pose.position.y + 2 * (1 - t) * t * arm_medium_pose.position.y + pow(t, 2) * arm_goal_pose.position.y;
            pose.position.z = pow(1 - t, 2) * current_pose.position.z + 2 * (1 - t) * t * arm_medium_pose.position.z + pow(t, 2) * arm_goal_pose.position.z;
        
            tf2::Quaternion interpolated_q = start_q.slerp(goal_q, t);
            pose.orientation = tf2::toMsg(interpolated_q);

            waypoints.push_back(pose);
        }
    }

    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = arm->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    if (fraction > 0.9) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        arm->execute(plan);
    } else {
        RCLCPP_ERROR(this->get_logger(), "MoveIt failed to plan the trajectory with required conditions");
    }
}

void KeyboardController::teach_motor_mode() {
    rbt_ctrl_msg.motor_enable_flag.resize(6);
    rbt_ctrl_msg.motor_mode.resize(6);
    rbt_ctrl_msg.motor_msg.resize(6);

    for (size_t i = 0; i < rbt_ctrl_msg.motor_mode.size(); i++) {
        rbt_ctrl_msg.motor_enable_flag[i] = false;
        rbt_ctrl_msg.motor_mode[i] = CURRENT_MODE;
        rbt_ctrl_msg.motor_msg[i] = latest_joint_states_->effort[i];
    }
    motor_msg_pub->publish(rbt_ctrl_msg);
}


void KeyboardController::teach_action_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (latest_joint_states_->position.empty()) {
        RCLCPP_WARN(this->get_logger(), "No joint states received yet!");
        return;
    }

    if (msg->data == "stop") {
        if (recording_) {
            recording_ = false;
            teach_bag_writer_.reset();  // 关闭并释放writer资源
            RCLCPP_INFO(this->get_logger(), "Stopped recording joint states to bag.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Recording was not active!");
        }
        return;
    }

    if (!teach_bag_writer_) {
        char* workspace_path = getenv("COLCON_PREFIX_PATH");
        if (!workspace_path) {
            RCLCPP_ERROR(this->get_logger(), "COLCON_PREFIX_PATH environment variable not set! Cannot determine workspace location.");
            return;
        }
    
        std::filesystem::path ws_path(workspace_path);
        ws_path = ws_path.parent_path();

        std::string config_dir = ws_path.string() + "/config";
        if (!std::filesystem::exists(config_dir)) {
            std::filesystem::create_directories(config_dir);
        }

        std::string bag_file_path = config_dir + "/" + msg->data;
        teach_bag_writer_ = std::make_unique<rosbag2_cpp::Writer>();
        teach_bag_writer_->open(
            rosbag2_cpp::StorageOptions{bag_file_path, "sqlite3"},
            rosbag2_cpp::ConverterOptions{"", ""});
        

        rosbag2_storage::TopicMetadata topic_metadata;
        topic_metadata.name = "joint_states";
        topic_metadata.type = "sensor_msgs/msg/JointState";
        topic_metadata.serialization_format = "cdr";
        teach_bag_writer_->create_topic(topic_metadata);
    }

    recording_ = true;
    RCLCPP_INFO(this->get_logger(), "Started recording joint states to bag...");
}


void KeyboardController::teach_repeat_callback(const std_msgs::msg::String::SharedPtr msg) {
    char* workspace_path = getenv("COLCON_PREFIX_PATH");
    if (!workspace_path) {
        RCLCPP_ERROR(this->get_logger(), "COLCON_PREFIX_PATH not set! Cannot determine workspace location.");
        return;
    }
    
    std::filesystem::path ws_path(workspace_path);
    ws_path = ws_path.parent_path(); 

    std::string full_path = ws_path.string() + "/config/" + msg->data;

    if (!std::filesystem::exists(full_path)) {
        RCLCPP_ERROR(this->get_logger(), "Rosbag file does not exist: %s", full_path.c_str());
        return;
    }

    if (msg->data == "stop") {

    }
    
    if (!play_bag_reader_) {
        play_bag_reader_ = std::make_unique<rosbag2_cpp::Reader>();
        play_bag_reader_->open(
            rosbag2_cpp::StorageOptions{full_path, "sqlite3"}, 
            rosbag2_cpp::ConverterOptions{"", ""});

        RCLCPP_INFO(this->get_logger(), "Playing trajectory from rosbag: %s", full_path.c_str());
    }

    sensor_msgs::msg::JointState start_joint_state;
    bool first_joint_state_received = false;

    while (play_bag_reader_->has_next()) {
        auto serialized_msg = play_bag_reader_->read_next();
        
        if (serialized_msg->topic_name != "joint_states") {
            continue;   // Jump out the uncorrect topic
        }

        // 反序列化消息
        rclcpp::SerializedMessage serialized_ros_msg(*serialized_msg->serialized_data);
        sensor_msgs::msg::JointState joint_state_msg;
        rclcpp::Serialization<sensor_msgs::msg::JointState> serializer;
        serializer.deserialize_message(&serialized_ros_msg, &joint_state_msg);

        // Take the first joint_state point as the started point
        if (!first_joint_state_received) {
            start_joint_state = joint_state_msg;
            first_joint_state_received = true;

            rbt_ctrl_msg.motor_enable_flag.resize(joint_state_msg.position.size(), MOTOR_ENABLE);
            rbt_ctrl_msg.motor_mode.resize(joint_state_msg.position.size(), POSITION_ABS_MODE); 
            rbt_ctrl_msg.motor_msg.resize(joint_state_msg.position.size());
            rbt_ctrl_msg.motor_msg = start_joint_state.position;
            motor_msg_pub->publish(rbt_ctrl_msg);
            rclcpp::sleep_for(std::chrono::milliseconds(1000));
        }

        rbt_ctrl_msg.motor_enable_flag.resize(joint_state_msg.position.size(), MOTOR_ENABLE);
        rbt_ctrl_msg.motor_mode.resize(joint_state_msg.position.size(), POSITION_ABS_MODE); 
        rbt_ctrl_msg.motor_msg.resize(joint_state_msg.position.size());

        rbt_ctrl_msg.motor_msg = joint_state_msg.position;
        motor_msg_pub->publish(rbt_ctrl_msg);
        RCLCPP_INFO(this->get_logger(), "Publishing joint state to motors: %lf, %lf, %lf, %lf, %lf, %lf", 
                                                                        joint_state_msg.position[0], joint_state_msg.position[1],
                                                                        joint_state_msg.position[2], joint_state_msg.position[3],
                                                                        joint_state_msg.position[4], joint_state_msg.position[5]);
        rclcpp::sleep_for(std::chrono::milliseconds(10));
    }

    play_bag_reader_->close();
    play_bag_reader_.reset();
}


void KeyboardController::save_current_state_to_db(const std::string& state_name) {
    if (!latest_joint_states_ || latest_joint_states_->position.empty()) {
        RCLCPP_WARN(this->get_logger(), "No joint states received yet!");
        return;
    }
    char* workspace_path = getenv("COLCON_PREFIX_PATH");
    if (!workspace_path) {
        RCLCPP_ERROR(this->get_logger(), "COLCON_PREFIX_PATH environment variable not set!");
        return;
    }
    std::filesystem::path ws_path(workspace_path);
    ws_path = ws_path.parent_path();
    std::string config_dir = ws_path.string() + "/config";

    if (!std::filesystem::exists(config_dir)) {
        std::filesystem::create_directory(config_dir);
    }

    std::string db_path = config_dir + "/robotic_arm_saved_states.db3";

    sqlite3* db;
    if (sqlite3_open(db_path.c_str(), &db)) {
        RCLCPP_ERROR(this->get_logger(), "Can't open database: %s", sqlite3_errmsg(db));
        return;
    }

    // 创建表（如果不存在）
    const char* create_table_sql = 
        "CREATE TABLE IF NOT EXISTS saved_states ("
        "name TEXT PRIMARY KEY, "
        "joints TEXT"
        ");";
    char* errmsg;
    if (sqlite3_exec(db, create_table_sql, nullptr, nullptr, &errmsg) != SQLITE_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create table: %s", errmsg);
        sqlite3_free(errmsg);
        sqlite3_close(db);
        return;
    }

    // 把关节角度数组转成字符串
    std::ostringstream joints_stream;
    for (size_t i = 0; i < latest_joint_states_->position.size(); ++i) {
        joints_stream << latest_joint_states_->position[i];
        if (i != latest_joint_states_->position.size() - 1) {
            joints_stream << ",";
        }
    }
    std::string joints_str = joints_stream.str();

    // 插入或替换
    std::string insert_sql = "INSERT OR REPLACE INTO saved_states (name, joints) VALUES (?, ?);";
    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db, insert_sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to prepare insert statement: %s", sqlite3_errmsg(db));
        sqlite3_close(db);
        return;
    }

    sqlite3_bind_text(stmt, 1, state_name.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, joints_str.c_str(), -1, SQLITE_TRANSIENT);

    if (sqlite3_step(stmt) != SQLITE_DONE) {
        RCLCPP_ERROR(this->get_logger(), "Failed to insert data: %s", sqlite3_errmsg(db));
    } else {
        RCLCPP_INFO(this->get_logger(), "Saved current joint state as '%s' into database!", state_name.c_str());
    }

    sqlite3_finalize(stmt);
    sqlite3_close(db);
}


void KeyboardController::save_state_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Received empty state name, ignoring save request.");
        return;
    }
    save_current_state_to_db(msg->data);
}


void KeyboardController::load_state_from_db(const std::string& state_name) {
    char* workspace_path = getenv("COLCON_PREFIX_PATH");
    if (!workspace_path) {
        RCLCPP_ERROR(this->get_logger(), "COLCON_PREFIX_PATH environment variable not set! Cannot determine workspace location.");
        return;
    }

    std::filesystem::path ws_path(workspace_path);
    ws_path = ws_path.parent_path();
    std::string config_dir = ws_path.string() + "/config";
    std::string db_path = config_dir + "/robotic_arm_saved_states.db3";

    sqlite3* db;
    if (sqlite3_open(db_path.c_str(), &db)) {
        RCLCPP_ERROR(this->get_logger(), "Can't open database: %s", sqlite3_errmsg(db));
        return;
    }

    std::string query_sql = "SELECT joints FROM saved_states WHERE name = ?;";
    sqlite3_stmt* stmt;
    if (sqlite3_prepare_v2(db, query_sql.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to prepare select statement: %s", sqlite3_errmsg(db));
        sqlite3_close(db);
        return;
    }

    sqlite3_bind_text(stmt, 1, state_name.c_str(), -1, SQLITE_TRANSIENT);

    int rc = sqlite3_step(stmt);
    if (rc == SQLITE_ROW) {
        const unsigned char* joints_text = sqlite3_column_text(stmt, 0);
        if (joints_text) {
            std::string joints_str(reinterpret_cast<const char*>(joints_text));
            std::vector<double> joint_positions;
            std::stringstream ss(joints_str);
            std::string token;
            while (std::getline(ss, token, ',')) {
                joint_positions.push_back(std::stod(token));
            }

            if (joint_positions.size() != 6) {
                RCLCPP_WARN(this->get_logger(), "Loaded joint positions size mismatch! Expected 6, got %zu", joint_positions.size());
            } else {
                // 成功加载，可以赋值到你的 current_joint_positions_
                rbt_ctrl_msg.motor_enable_flag.resize(6, true);
                rbt_ctrl_msg.motor_mode.resize(6, POSITION_ABS_MODE);
                rbt_ctrl_msg.motor_msg.resize(6);

                rbt_ctrl_msg.motor_msg = joint_positions;
                motor_msg_pub->publish(rbt_ctrl_msg);
                RCLCPP_INFO(this->get_logger(), "Successfully loaded state '%s'!", state_name.c_str());
            }
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "State '%s' not found in database!", state_name.c_str());
    }

    sqlite3_finalize(stmt);
    sqlite3_close(db);
}


void KeyboardController::load_state_callback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty state name, ignoring load request.");
        return;
    }
    load_state_from_db(msg->data);
}


int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardController>("key_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

