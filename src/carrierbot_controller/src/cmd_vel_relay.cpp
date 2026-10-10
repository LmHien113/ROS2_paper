#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class CmdVelRelay : public rclcpp::Node
{
public:
    CmdVelRelay() : Node("cmd_vel_relay")
    {
        // Subscribe to cmd_vel_wheels
        cmd_vel_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/cmd_vel_wheels",
            10,
            std::bind(&CmdVelRelay::cmdVelCallback, this, std::placeholders::_1)
        );

        // Publish to simple_velocity_controller/commands
        cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/simple_velocity_controller/commands",
            10
        );

        RCLCPP_INFO(this->get_logger(), "CMD Velocity Relay started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to: /cmd_vel_wheels");
        RCLCPP_INFO(this->get_logger(), "Publishing to: /simple_velocity_controller/commands");
    }

private:
    void cmdVelCallback(const std::shared_ptr<const std_msgs::msg::Float64MultiArray> msg)
    {
        if (msg->data.size() < 2)
        {
            RCLCPP_WARN(this->get_logger(), "Invalid message size. Expected [left, right]");
            return;
        }

        // Create command message
        auto cmd_msg = std_msgs::msg::Float64MultiArray();
        cmd_msg.data.push_back(msg->data[1]);  // left wheel velocity
        cmd_msg.data.push_back(msg->data[0]);  // right wheel velocity

        cmd_pub_->publish(cmd_msg);
        RCLCPP_INFO_STREAM(this->get_logger(), 
            "Relay: Left: " << msg->data[0] << " m/s, Right: " << msg->data[1] << " m/s");
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdVelRelay>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
