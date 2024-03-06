#include "control_node.hpp"

#define LINEAR_X  3.0f
#define ANGULAR_Z 1.0f

#define OFFSET(X) ((X < 0.1) && (X > -0.1) ? 0.0: X)

ControlNode::ControlNode(): Node("control_node"){
    pub = this->create_publisher<twistMsg>("cmd_vel", 10);
    sub = this->create_subscription<joyMsg>("joy", 10, 
                    std::bind(&ControlNode::joyCallback, this, std::placeholders::_1));

    keyboard_sub = this->create_subscription<int32Msg>("/keypress", 10,
                                    std::bind(&ControlNode::keyboardCallback, this, std::placeholders::_1));
    keyboard_timer = this->create_wall_timer(std::chrono::milliseconds(100),
                                    std::bind(&ControlNode::keyboardControl, this)); 
    RCLCPP_INFO(this->get_logger(), "Created control node started.");  
}

void ControlNode::joyCallback(const joyMsg::SharedPtr msg){
    pub_data.linear.x = OFFSET(msg->axes[1]) * 1.5;
    pub_data.angular.z = OFFSET(msg->axes[0]) * 1.0;
    pub->publish(pub_data);
}

void ControlNode::keyboardCallback(const int32Msg::SharedPtr msg){
    bool is_ready = true;
    pub_data.linear.x = 0.0;
    pub_data.angular.z = 0.0;

    switch(msg->data){
      case KEYCODE_W:
        RCLCPP_DEBUG(this->get_logger(), "UP");
        pub_data.linear.x = LINEAR_X;
        break;
      case KEYCODE_A:
        RCLCPP_DEBUG(this->get_logger(), "LEFT");
        pub_data.angular.z = ANGULAR_Z;
        break;
      case KEYCODE_S:
        RCLCPP_DEBUG(this->get_logger(), "STOP");
        break;
      case KEYCODE_D:
        RCLCPP_DEBUG(this->get_logger(), "RIGHT");
        pub_data.angular.z = -ANGULAR_Z;
        break;
      case KEYCODE_X:
        RCLCPP_DEBUG(this->get_logger(), "DOWN");
        pub_data.linear.x = -LINEAR_X;
        break;
      default:
        RCLCPP_DEBUG(this->get_logger(), "None: %x", msg->data);
        is_ready = false;
        break;
    }

    if(is_ready){
        last_msg_timestamp_ = std::chrono::steady_clock::now();
        pub->publish(pub_data);
    }
}

void ControlNode::keyboardControl(){
    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    elapsed_time = current_time - last_msg_timestamp_;

    if (elapsed_time.count() > 100000000) {
        pub->publish(pub_data);
    }
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}