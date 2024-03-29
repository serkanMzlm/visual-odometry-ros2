#ifndef __POSE_CALCULATION_NODE_TYPE_HPP__
#define __POSE_CALCULATION_NODE_TYPE_HPP__

#include <string>

#define OFFSET 0.01f
#define MIN_VELOCITY -50.0f
#define MAX_VELOCITY 50.0f

#define  IMAGE_SIZE 518400 // 720 * 720
// #define  IMAGE_SIZE 4096   // 64 * 64

typedef struct {
    unsigned int width;
    unsigned int height;
    unsigned int depth;
    std::string format;
    double hfov;
    double focal_length;
    int output_rate;
    int first_frame_time;
    int first_frame_time_flag = 0;
    int size;
}cameraInformations;

#endif