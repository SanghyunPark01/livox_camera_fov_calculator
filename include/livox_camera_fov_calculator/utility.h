#ifndef UTILITY_H
#define UTILITY_H

#include <signal.h>
#include <thread>
#include <mutex>
#include <queue>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <cstdlib>
#include <boost/shared_ptr.hpp>
#include <algorithm>
#include <cmath>
#include <fstream>

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <livox_ros_driver/CustomMsg.h>

void mySigintHandler(int sig){
    ros::shutdown();
}

cv::Mat convertROSImage2CvMat(const sensor_msgs::ImageConstPtr& msgImg)
{
    try {
        cv_bridge::CvImagePtr ptrCvBridge = cv_bridge::toCvCopy(msgImg, sensor_msgs::image_encodings::BGR8);
        return ptrCvBridge->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return cv::Mat();
    }
}
cv::Mat convertROSCompressedImage2CvMat(const sensor_msgs::CompressedImageConstPtr& msgImg)
{
    try {
        cv::Mat rawData(msgImg->data);
        cv::Mat decodedImage = cv::imdecode(rawData, cv::IMREAD_COLOR);
        if (decodedImage.empty()) {
            ROS_ERROR("Failed to decode compressed image.");
            return cv::Mat();
        }
        return decodedImage;
    } catch (const std::exception& e) {
        ROS_ERROR("Exception while converting compressed image: %s", e.what());
        return cv::Mat();
    }
}

pcl::PointCloud<pcl::PointXYZI> convertLivox2Pcl(livox_ros_driver::CustomMsg msgLivox)
{
    pcl::PointCloud<pcl::PointXYZI> pclLivoxData;
    pcl::PointXYZI Point;
    for(int i = 0; i < msgLivox.point_num; i++)
    {
        Point.x = msgLivox.points[i].x;
        Point.y = msgLivox.points[i].y;
        Point.z = msgLivox.points[i].z;
        Point.intensity = msgLivox.points[i].reflectivity;
        pclLivoxData.points.push_back(Point);
    }
    pclLivoxData.width = pclLivoxData.points.size();
    pclLivoxData.height = 1;

    return pclLivoxData;
}

#endif UTILITY_H