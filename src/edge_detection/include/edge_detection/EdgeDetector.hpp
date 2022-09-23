#pragma once
#include <ros/ros.h>
#include <opencv4/opencv2/opencv.hpp>
#include <iostream>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <cv_bridge/cv_bridge.h>
#include <edge_srv/edgeDetector.h>
using namespace cv;

namespace edge_detection
{
class EdgeDetector
{
    // Your class declaration goes here

public:
    //   edge functions
    void read(String dir_input);
    void read(sensor_msgs::Image img_inp);
    void runDetector();
    void drawLines();
    Mat result();
    Mat edgeResult();
    void displayImage(Mat img_display);

    //    3D projection functions
    sensor_msgs::PointCloud get3DLocs(std::vector<cv::Point> edgesVec);
    void readDepth(sensor_msgs::Image img_inp);

    //   edge variables
    Mat img_input;
    Mat img_output;
    Mat edges;
    Mat cdst, cdstP;
    int counter = 0;

    //    3D projection variables
    sensor_msgs::CameraInfo camInfo_global;
    Mat depthImg_input;
};
}
