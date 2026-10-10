#include "carrierbot_controller/simple_controller.hpp"

using std::placeholders::_1;

SimpleController::SimpleController(const std::string & name)
    : Node(name)
{
    declare_parameter("wheel_radius", 0.033);
    declare_parameter("wheel_separation", 0.17);

    wheel_radius_ = get_parameter("wheel_radius").as_double();
    wheel_separation_ = get_parameter("wheel_separation").as_double();

    RCLCPP_INFO_STREAM(get_logger(), "Using wheel radius " << wheel_radius_);
    RCLCPP_INFO_STREAM(get_logger(), "Using wheel separation " << wheel_separation_);

    wheel_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/simple_velocity_controller/commands", 10);
    vel_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>("/cmd_vel_wheels", 10,
        std::bind(&SimpleController::velCallback, this, _1));

    RCLCPP_INFO_STREAM(get_logger(), "SimpleController started - waiting for /cmd_vel_wheels [left_velocity, right_velocity]");
}

void SimpleController::velCallback(const std::shared_ptr<const std_msgs::msg::Float64MultiArray> msg)
{
    if (msg->data.size() < 2) {
        RCLCPP_WARN_STREAM(get_logger(), "Invalid message size. Expected [left_velocity, right_velocity]");
        return;
    }

    std_msgs::msg::Float64MultiArray wheel_speed_msg;
    wheel_speed_msg.data.push_back(msg->data[0]);  // left wheel velocity
    wheel_speed_msg.data.push_back(msg->data[1]);  // right wheel velocity

    wheel_cmd_pub_->publish(wheel_speed_msg);
    RCLCPP_DEBUG_STREAM(get_logger(), "Sending wheel velocities: [" << msg->data[0] << ", " << msg->data[1] << "]");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleController>("simple_controller");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}