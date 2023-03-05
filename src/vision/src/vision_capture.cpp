/**
 * This node is to capture frame from RealSense Camare
 * Captured frame will be published to another Nodes
 *
 * March, 2023
 */

#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"

image_transport::Publisher pub_frame_bgr;

cv::VideoCapture cam;

void cllbck_main();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_capture");
    ros::NodeHandle NH;
    image_transport::ImageTransport IT(NH);

    pub_frame_bgr = IT.advertise("vision_bgr", 3); // Adjust it by the number of subscriber

    /* Camera Open */
    cam.open("/dev/v4l/by-id/usb-Goodong_Industry_Co.__Ltd._USB2.0_HD_UVC_WebCam_0x0001-video-index0");

    /**
     * Linux command that important for vision prorgamming:
     * v4l2-ctl -L
     * v4l2-ctl --list-formats-ext
     * v4l2-ctl --list-devices
     *
     */

    /* Camera pre-processing */
    while (ros::ok())
    {
        cllbck_main();
    }

    return 0;
}

void cllbck_main()
{
    cv::Mat frame_bgr;
    cam >> frame_bgr;

    sensor_msgs::ImagePtr msg_frame_bgr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_bgr).toImageMsg();
    pub_frame_bgr.publish(msg_frame_bgr);
}