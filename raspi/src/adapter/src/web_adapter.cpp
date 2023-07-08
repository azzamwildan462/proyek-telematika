/**
 * This is a web adapter,
 * Raspberry PI will accessible over internet
 * so this is an adapter for web
 *
 * March, 2023
 */

#include "string"
#include "ros/ros.h"

/* ROS Timer */
ros::Timer tim_50hz;

void cllbck_tim50hz(const ros::TimerEvent &event);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "web_adapter");
    ros::NodeHandle NH;
    ros::MultiThreadedSpinner MTS;

    MTS.spin();

    tim_50hz.stop();

    return 0;
}

void cllbck_tim50hz(const ros::TimerEvent &event)
{
}
