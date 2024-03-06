#ifndef __POSE_CALCULATION_NODE_HPP__
#define __POSE_CALCULATION_NODE_HPP__

#include <vector>

#include "cv_bridge/cv_bridge.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "pose_calculation_node_type.hpp"
#include "flow_opencv.hpp"


using cameraInfoMsg = sensor_msgs::msg::CameraInfo;
using imageMsg = sensor_msgs::msg::Image;
using twistMsg = geometry_msgs::msg::Twist;

class PoseCalculationNode: public rclcpp::Node{
public:
    PoseCalculationNode();

    void setup();
    void initData();
    void cameraCallback(const imageMsg::SharedPtr msg);
    void cameraInfoCallback(const cameraInfoMsg::SharedPtr msg);
private:
    bool enable_display = false;
    uint8_t  image[IMAGE_SIZE];
    uint32_t frame_time = 0;

    float sum_x = 0.0f;
    float sum_y = 0.0f;

    OpticalFlowOpenCV* optical_flow;
    cv_bridge::CvImagePtr cv_ptr;
    cameraInformations cam_info;

    rclcpp::Subscription<imageMsg>::SharedPtr image_sub; 
    rclcpp::Subscription<cameraInfoMsg>::SharedPtr camera_info_sub;
    rclcpp::Publisher<twistMsg>::SharedPtr pos_pub;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif