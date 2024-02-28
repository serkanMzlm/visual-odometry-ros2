#ifndef __CONTROL_NODE_HPP__
#define __CONTROL_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "control_type.hpp"

using joyMsg = sensor_msgs::msg::Joy;
using twistMsg = geometry_msgs::msg::Twist;

class ControlNode: public rclcpp::Node{
private:
    twistMsg pub_data;
    rclcpp::Publisher<twistMsg>::SharedPtr pub;
    rclcpp::Subscription<joyMsg>::SharedPtr sub;

public:
    ControlNode();
    void joyCallback(const joyMsg::SharedPtr msg); 
};
#endif