/**
 * This node is main processing of realsense,
 * In this node there will be object detection
 * And the last is getting depth from detection
 *
 * March. 2023
 */

#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "boost/thread/mutex.hpp"
#include "custom_msgs/realsense.h"

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
    ros::init(argc, argv, "realsense_main");
    ros::NodeHandle NH;
    image_transport::ImageTransport IT(NH);
    ros::MultiThreadedSpinner MTS;

    pub_final_results = NH.advertise<custom_msgs::realsense>("realsense_results", 1);
    sub_frame_bgr = NH.subscribe("vision_bgr", 1, cllbck_sub_frame_bgr);
    tim_50hz = NH.createTimer(ros::Duration(0.02), cllbck_tim50hz);

    MTS.spin();

    tim_50hz.stop();

    return 0;
}

void cllbck_tim50hz(const ros::TimerEvent &event)
{
    /* Prepare local variables */
    static float final_point[3];
    static char *object_name = "Meja";

    mutex_frame_bgr.lock();
    // Do it here...
    mutex_frame_bgr.unlock();

    /* Publish to audio interface node */
    custom_msgs::realsense msg_realsense;
    msg_realsense.pos3d.x = final_point[0];
    msg_realsense.pos3d.y = final_point[1];
    msg_realsense.pos3d.z = final_point[2];
    msg_realsense.object.data = object_name;
    pub_final_results.publish(msg_realsense);
}

void cllbck_sub_frame_bgr(const sensor_msgs::ImageConstPtr &msg)
{
    mutex_frame_bgr.lock();
    frame_bgr = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    mutex_frame_bgr.unlock();
}
