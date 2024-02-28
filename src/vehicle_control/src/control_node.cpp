#include "control_node.hpp"

#define OFFSET(X) ((X < 0.1) && (X > -0.1) ? 0.0: X)

ControlNode::ControlNode(): Node("control_node"){
    pub = this->create_publisher<twistMsg>("cmd_vel", 10);
    sub = this->create_subscription<joyMsg>("joy", 10, 
                    std::bind(&ControlNode::joyCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Created control node started.");  
}

void ControlNode::joyCallback(const joyMsg::SharedPtr msg){
    pub_data.linear.x = OFFSET(msg->axes[1]) * 1.5;
    pub_data.angular.z = OFFSET(msg->axes[0]) * 1.0;
    pub->publish(pub_data);
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}