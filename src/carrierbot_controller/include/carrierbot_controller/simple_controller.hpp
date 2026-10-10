#ifndef SIMPLE_CONTROLLER_HPP
#define SIMPLE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class SimpleController : public rclcpp::Node
{
public:
    SimpleController(const std::string & name);

private:
    void velCallback(const std::shared_ptr<const std_msgs::msg::Float64MultiArray> msg);

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;

    double wheel_radius_;
    double wheel_separation_;

};

#endif
