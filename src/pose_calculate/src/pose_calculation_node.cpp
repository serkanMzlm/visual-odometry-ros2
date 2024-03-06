#include "pose_calculation_node.hpp"
using namespace std::placeholders;
PoseCalculationNode::PoseCalculationNode():Node("optical_flow_node"){
    setup();
    initData();
}

void PoseCalculationNode::cameraCallback(const imageMsg::SharedPtr msg){
    int height = msg->height;
    int width = msg->width;
    int channels = 0;

    if(msg->encoding == sensor_msgs::image_encodings::RGB8){
        channels = 3;
    }else if(msg->encoding == sensor_msgs::image_encodings::MONO8){
        channels = 1;
    }else{
        RCLCPP_ERROR(this->get_logger(), "Unsupported Image Encoding");
    }

    const uint8_t *img_ptr = msg->data.data();
    std::vector<uint8_t> pixel_values;
    pixel_values.reserve(height * width);

    for(int y = 0; y < height; y++){
        for(int x = 0; x < width; x++){
            int idx = (y * width + x) * channels;
            pixel_values.push_back(img_ptr[idx]);
        }
    }

    for(int i = 0; i < cam_info.size; i++){
        image[i] = pixel_values[i];
    }

    float flow_x_ang = 0.0f;
    float flow_y_ang = 0.0f;

    int dt_us = 0.01;
    uint32_t frame_time_us = (frame_time - cam_info.first_frame_time) * 1e6;
    int quality = optical_flow->calcFlow((uint8_t *)image, frame_time_us, 
                                                dt_us, flow_x_ang, flow_y_ang);
    
    if(enable_display){
        cv::Mat image(height, width, CV_8UC(channels), const_cast<uint8_t*>(img_ptr));
        cv::imshow("Image", image);
        cv::waitKey(1); 
    }

    twistMsg pos_msg;

    sum_x += (abs(flow_x_ang) > OFFSET ? flow_x_ang : 0.0f);
    sum_y += (abs(flow_y_ang) > OFFSET ? flow_y_ang : 0.0f);

    pos_msg.linear.x = flow_x_ang;
    pos_msg.linear.y = flow_y_ang;

    pos_msg.angular.x = sum_x;
    pos_msg.angular.y = sum_y;

    pos_pub->publish(pos_msg);
}

void PoseCalculationNode::cameraInfoCallback(const cameraInfoMsg::SharedPtr msg){
    if(cam_info.first_frame_time_flag == 0){
        cam_info.first_frame_time = msg->header.stamp.sec;
        cam_info.first_frame_time_flag=1;
        RCLCPP_INFO(this->get_logger(), "First Frame Time");
    }

    frame_time = msg->header.stamp.sec;
    // RCLCPP_INFO(this->get_logger(), "Frame Time: %ld", cam_info.first_frame_time);
}

void PoseCalculationNode::setup(){
    image_sub = this->create_subscription<imageMsg>("/camera", 10, 
                        std::bind(&PoseCalculationNode::cameraCallback, this, _1));
    camera_info_sub = this->create_subscription<cameraInfoMsg>("/camera_info", 10,
                        std::bind(&PoseCalculationNode::cameraInfoCallback, this, _1));
    pos_pub = this->create_publisher<twistMsg>("/pose", 10);

}

void PoseCalculationNode::initData(){
    cam_info.width = 720;
    cam_info.height = 720;
    cam_info.depth = 0;
    cam_info.format = "R8G8B8";
    cam_info.hfov = 0.088; 
    cam_info.focal_length = (cam_info.width / 2) / tan(cam_info.hfov / 2);
    cam_info.output_rate = 20;
    
    optical_flow = new OpticalFlowOpenCV(cam_info.focal_length,
                                         cam_info.focal_length,
                                         cam_info.output_rate);

    cam_info.size = cam_info.width * cam_info.height;
    
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseCalculationNode>());
    rclcpp::shutdown();
    return 0;
}