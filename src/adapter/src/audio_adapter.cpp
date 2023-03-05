/**
 * This node will be an audio processing
 * The output is an audio that has 3d stereo
 *
 * March, 2023
 */

#include "string"
#include "ros/ros.h"
#include "custom_msgs/realsense.h"

/* ROS Timer */
ros::Timer tim_50hz;

/* ROS Subscriber */
ros::Subscriber sub_realsense_final;

/* Global Variables */
float realsense_point[3];
char *realsense_object;

void cllbck_tim50hz(const ros::TimerEvent &event);
void cllbck_sub_realsense(const custom_msgs::realsenseConstPtr &msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "audio_adapter");
    ros::NodeHandle NH;
    ros::MultiThreadedSpinner MTS;

    sub_realsense_final = NH.subscribe("realsense_results", 1, cllbck_sub_realsense);
    tim_50hz = NH.createTimer(ros::Duration(0.02), cllbck_tim50hz);

    MTS.spin();

    tim_50hz.stop();

    return 0;
}

void cllbck_tim50hz(const ros::TimerEvent &event)
{
}

void cllbck_sub_realsense(const custom_msgs::realsenseConstPtr &msg)
{
    realsense_point[0] = msg->pos3d.x;
    realsense_point[1] = msg->pos3d.y;
    realsense_point[2] = msg->pos3d.z;

    strcpy(realsense_object, msg->object.data.c_str());
}
