#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// Callback function for processing laser scan data
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    size_t numBeams = msg->ranges.size();
    ROS_INFO("Number of laser beams: %zu", numBeams);

    float angleIncrement = msg->angle_increment;
    ROS_INFO("Angle increment: %.2f", angleIncrement);

    for (size_t i = 0; i < numBeams; ++i) {
        float range = msg->ranges[i];
        float angle = msg->angle_min + i * angleIncrement;
        ROS_INFO("Beam %zu: Range=%.2f, Angle=%.2f", i, range, angle);
    }
    for (size_t i = 0; i < numBeams; ++i) {
        float range = msg->ranges[i];
        float angle = msg->angle_min + i * angleIncrement;
        float x = range * cos(angle);
        float y = range * sin(angle);
        ROS_INFO("Beam %zu: Cartesian=(%.2f, %.2f)", i, x, y);
    }
    for (size_t i = 0; i < numBeams; ++i) {
        float range = msg->ranges[i];
        if (range < 0.5) {
        }
    }
}
// Callback function for processing left stereo image data
void leftImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImagePtr cvImage;
    try {
        cvImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("CV Bridge Exception: %s", e.what());
        return;
    }
    cv::Mat processedImage;
    cv::cvtColor(cvImage->image, processedImage, cv::COLOR_BGR2GRAY);
    cv::imshow("Left Image", processedImage);
    cv::waitKey(1);
}

// Callback function for processing right stereo image data
void rightImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImagePtr cvImage;
    try {
        cvImage = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("CV Bridge Exception: %s", e.what());
        return;
    }
    cv::Mat processedImage;
    cv::cvtColor(cvImage->image, processedImage, cv::COLOR_BGR2GRAY);
    cv::imshow("Right Image", processedImage);
    cv::waitKey(1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pioneer_controller");
    ros::NodeHandle nh;
    ros::Subscriber laserSub = nh.subscribe("laser_scan", 10, laserScanCallback);
    ros::Subscriber leftImageSub = nh.subscribe("left_image", 10, leftImageCallback);
    ros::Subscriber rightImageSub = nh.subscribe("right_image", 10, rightImageCallback);
    ros::Publisher twistPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    while (ros::ok()) {
    geometry_msgs::Twist twistMsg;
    // Set linear and angular velocities
    twistMsg.linear.x = 0.5; 
    twistMsg.angular.z = 0.2; 
    twistPub.publish(twistMsg);

    ros::spinOnce();

    loop_rate.sleep();
    }
    return 0;
}
