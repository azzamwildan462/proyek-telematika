/**
 * This node is to capture frame from RealSense Camare
 * Captured frame will be published to another Nodes
 *
 * March, 2023
 */

#include "ros/ros.h"
#include "ros/package.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "yaml-cpp/yaml.h"

/* Frame publisher */
image_transport::Publisher pub_frame_bgr;

/* Camera video interface */
cv::VideoCapture cam;

/* Configs and tuningable camera params */
char camera_path[100];

/**
 * 0 is for Brightness
 * 1 is for Contrast
 * 2 is for Saturation
 */
uint16_t camera_params[3];

void cllbck_main();
void load_config();
void set_camera_param(const char *path, const char *param, int value);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vision_capture");
    ros::NodeHandle NH;
    image_transport::ImageTransport IT(NH);

    load_config();

    pub_frame_bgr = IT.advertise("vision_bgr", 3); // Adjust it by the number of subscriber

    /* Camera Open */
    cam.open(camera_path);

    /**
     * Linux command that important for vision prorgamming:
     * v4l2-ctl -L
     * v4l2-ctl --list-formats-ext
     * v4l2-ctl --list-devices
     *
     */
    set_camera_param(camera_path, "Brightness", camera_params[0]);
    set_camera_param(camera_path, "Constrast", camera_params[1]);
    set_camera_param(camera_path, "Saturation", camera_params[2]);

    /* Camera pre-processing */
    while (ros::ok())
    {
        cllbck_main();
    }

    return 0;
}

void load_config()
{
    char static_config_file[100], dynamic_config_file[100];
    std::string current_dir = ros::package::getPath("vision");
    sprintf(static_config_file, "%s/../../config/static_conf.yaml", current_dir.c_str());
    sprintf(dynamic_config_file, "%s/../../config/dynamic_conf.yaml", current_dir.c_str());

    YAML::Node static_config = YAML::LoadFile(static_config_file);
    YAML::Node dynamic_config = YAML::LoadFile(dynamic_config_file);

    strcpy(camera_path, static_config["Camera"]["path"].as<std::string>().c_str());

    printf("camera path: %s\n", camera_path);

    camera_params[0] = dynamic_config["Camera"]["Brightness"].as<int>();
    camera_params[1] = dynamic_config["Camera"]["Contrast"].as<int>();
    camera_params[2] = dynamic_config["Camera"]["Saturation"].as<int>();

    printf("camera params: %d %d %d\n", camera_params[0], camera_params[1], camera_params[2]);
}

void set_camera_param(const char *path, const char *param, int value)
{
    char cmd[128];
    sprintf(cmd, "v4l2-ctl -d %s -c %s=%d", path, param, value);
    system(cmd);
}

void cllbck_main()
{
    cv::Mat frame_bgr;
    cam >> frame_bgr;

    sensor_msgs::ImagePtr msg_frame_bgr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_bgr).toImageMsg();
    pub_frame_bgr.publish(msg_frame_bgr);
}