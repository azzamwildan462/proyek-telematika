/**
 * This node will be an audio processing
 * The output is an audio that has 3d stereo
 *
 * March, 2023
 */

#include "string"
#include "ros/ros.h"
#include "custom_msgs/realsense.h"
#include "std_msgs/UInt8.h"

#define TOGGLE_0 0b0000
#define TOGGLE_1 0b0010
#define TOGGLE_2 0b0100
#define TOGGLE_3 0b1000
#define TOGGLE_0_SHIFT 0x00
#define TOGGLE_1_SHIFT 0x01
#define TOGGLE_2_SHIFT 0x02
#define TOGGLE_3_SHIFT 0x03

/* ROS Timer */
ros::Timer tim_50hz;

/* ROS Subscriber */
ros::Subscriber sub_realsense_final;
ros::Subscriber sub_hardware;

/* Global Variables */
float realsense_point[3];
char *realsense_object;

uint8_t toggle_0;
uint8_t toggle_1;
uint8_t toggle_2;

/* ROS prototypes */
void cllbck_tim50hz(const ros::TimerEvent &event);
void cllbck_sub_realsense(const custom_msgs::realsenseConstPtr &msg);
void cllbck_sub_hardware(const std_msgs::UInt8ConstPtr &msg);

/* Another prototypes */
void audio_outPlay(const char *text);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "audio_adapter");
    ros::NodeHandle NH;
    ros::MultiThreadedSpinner MTS;

    sub_realsense_final = NH.subscribe("realsense_results", 1, cllbck_sub_realsense);
    sub_hardware = NH.subscribe("hardware_toggle", 1, cllbck_sub_hardware);
    tim_50hz = NH.createTimer(ros::Duration(0.02), cllbck_tim50hz);

    MTS.spin();

    tim_50hz.stop();

    return 0;
}

void cllbck_tim50hz(const ros::TimerEvent &event)
{
    static double last_time_speak = 0;

    if (ros::Time::now().toSec() - last_time_speak > 1)
    {
        if (toggle_0)
            audio_outPlay(realsense_object);

        last_time_speak = ros::Time::now().toSec();
    }
}

void cllbck_sub_realsense(const custom_msgs::realsenseConstPtr &msg)
{
    realsense_point[0] = msg->pos3d.x;
    realsense_point[1] = msg->pos3d.y;
    realsense_point[2] = msg->pos3d.z;

    strcpy(realsense_object, msg->object.data.c_str());
}

void cllbck_sub_hardware(const std_msgs::UInt8ConstPtr &msg)
{
    toggle_0 = (msg->data & TOGGLE_0) >> TOGGLE_0_SHIFT;
    toggle_1 = (msg->data & TOGGLE_1) >> TOGGLE_1_SHIFT;
    toggle_2 = (msg->data & TOGGLE_2) >> TOGGLE_2_SHIFT;
}

void audio_outPlay(const char *text)
{
    char cmd[128];
    sprintf(cmd, "gtts-cli '%s' -l id --output aud_temp.wav", text);
    system(cmd);
}
