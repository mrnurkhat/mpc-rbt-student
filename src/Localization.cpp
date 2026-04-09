#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "Localization.hpp"

LocalizationNode::LocalizationNode() : 
    rclcpp::Node("localization_node"),
    last_time_(this->get_clock()->now()), 
    x_(0.0), y_(0.0), theta_(0.0)     
{
    wheel_radius = robot_config::WHEEL_RADIUS;
    wheel_base = 2 * robot_config::HALF_DISTANCE_BETWEEN_WHEELS;

    odometry_.header.frame_id = "map";
    odometry_.child_frame_id = "base_link";
    odometry_.pose.pose.position.x = 0.0;
    odometry_.pose.pose.position.y = 0.0;
    odometry_.pose.pose.position.z = 0.0;
    odometry_.pose.pose.orientation.x = 0.0;
    odometry_.pose.pose.orientation.y = 0.0;
    odometry_.pose.pose.orientation.z = 0.0;
    odometry_.pose.pose.orientation.w = 1.0;

    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, 
        std::bind(&LocalizationNode::jointCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Localization node started successfully.");
}
void LocalizationNode::jointCallback(const sensor_msgs::msg::JointState & msg) {
    double left_wheel_vel = 0.0;
    double right_wheel_vel = 0.0;
    bool left_found = false;
    bool right_found = false;


    for (size_t i = 0; i < msg.name.size(); i++) {
        if (msg.name[i] == "wheel_left_joint") {
            left_wheel_vel = msg.velocity[i];
            left_found = true;
        } else if (msg.name[i] == "wheel_right_joint") {
            right_wheel_vel = msg.velocity[i];
            right_found = true;
        }
    }

    if (left_found && right_found) {
        auto current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        if (dt > 0.0 && dt < 1.0) {
            updateOdometry(left_wheel_vel, right_wheel_vel, dt);
            publishOdometry();
            publishTransform();
        }
    }
}

void LocalizationNode::updateOdometry(double left_wheel_vel, double right_wheel_vel, double dt) {
    double linear = (wheel_radius / 2.0) * (right_wheel_vel + left_wheel_vel);
    double angular = (wheel_radius / wheel_base) * (right_wheel_vel - left_wheel_vel);

    if (std::isnan(linear) || std::isnan(angular)) return;

    theta_ += angular * dt;
    theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

    x_ += linear * std::cos(theta_) * dt;
    y_ += linear * std::sin(theta_) * dt;

    // Если всё еще числа — записываем
    if (!std::isnan(x_) && !std::isnan(y_) && !std::isnan(theta_)) {
        odometry_.pose.pose.position.x = x_;
        odometry_.pose.pose.position.y = y_;   

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        q.normalize(); // На всякий случай нормализуем принудительно
        odometry_.pose.pose.orientation = tf2::toMsg(q);

        odometry_.twist.twist.linear.x = linear;
        odometry_.twist.twist.angular.z = angular;
    }
}

void LocalizationNode::publishOdometry() {
    odometry_.header.stamp = this->get_clock()->now();
    odometry_publisher_->publish(odometry_);
}

void LocalizationNode::publishTransform() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = odometry_.header.frame_id; // "map"
    t.child_frame_id = odometry_.child_frame_id;   // "base_link"

    t.transform.translation.x = odometry_.pose.pose.position.x;
    t.transform.translation.y = odometry_.pose.pose.position.y;
    t.transform.translation.z = 0.0;
    t.transform.rotation = odometry_.pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);
}
