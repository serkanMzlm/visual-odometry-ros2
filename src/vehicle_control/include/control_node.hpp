#ifndef __CONTROL_NODE_HPP__
#define __CONTROL_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"

#include "control_type.hpp"

using joyMsg = sensor_msgs::msg::Joy;
using int32Msg = std_msgs::msg::Int32;
using twistMsg = geometry_msgs::msg::Twist;

class ControlNode: public rclcpp::Node{
private:
    twistMsg pub_data;
    rclcpp::TimerBase::SharedPtr keyboard_timer; 
    rclcpp::Publisher<twistMsg>::SharedPtr pub;
    rclcpp::Subscription<joyMsg>::SharedPtr sub;
    rclcpp::Subscription<int32Msg>::SharedPtr keyboard_sub;

    std::chrono::steady_clock::time_point last_msg_timestamp_;
    std::chrono::steady_clock::duration elapsed_time ;

public:
    ControlNode();
    void joyCallback(const joyMsg::SharedPtr msg); 
    void keyboardCallback(const int32Msg::SharedPtr msg);
    void keyboardControl();
};
#endif