/**
 * This node will process gmaps
 *
 * March. 2023
 */

#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "boost/thread/mutex.hpp"

/* ROS Timer */
ros::Timer tim_50hz;

/* ROS Subscriber */
ros::Subscriber sub_frame_bgr;

/* ROS publisher */
ros::Publisher pub_final_results;

/* Global variables */
cv::Mat frame_bgr;

boost::mutex mutex_frame_bgr;

void cllbck_tim50hz(const ros::TimerEvent &event);
void cllbck_sub_frame_bgr(const sensor_msgs::ImageConstPtr &msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gmaps");
    ros::NodeHandle NH;
    image_transport::ImageTransport IT(NH);
    ros::MultiThreadedSpinner MTS;

    sub_frame_bgr = NH.subscribe("vision_bgr", 1, cllbck_sub_frame_bgr);
    tim_50hz = NH.createTimer(ros::Duration(0.02), cllbck_tim50hz);

    MTS.spin();

    tim_50hz.stop();

    return 0;
}

void cllbck_tim50hz(const ros::TimerEvent &event)
{
    mutex_frame_bgr.lock();
    // Do it here...
    mutex_frame_bgr.unlock();
}

void cllbck_sub_frame_bgr(const sensor_msgs::ImageConstPtr &msg)
{
    mutex_frame_bgr.lock();
    frame_bgr = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    mutex_frame_bgr.unlock();
}
