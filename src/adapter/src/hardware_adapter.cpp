/**
 * This node will be a pre-processing hardware
 *
 * March, 2023
 */

#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/UInt8.h"
#include "yaml-cpp/yaml.h"

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

/* ROS Publisher */
ros::Publisher pub_toggle_datas;

uint8_t toggles_pin[4] = {0, 0, 0, 0};

void cllbck_tim50hz(const ros::TimerEvent &event);
void load_config();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hardware_adapter");
    ros::NodeHandle NH;
    ros::MultiThreadedSpinner MTS;

    pub_toggle_datas = NH.advertise<std_msgs::UInt8>("hardware_toggle", 1);

    tim_50hz = NH.createTimer(ros::Duration(0.02), cllbck_tim50hz);

    MTS.spin();

    tim_50hz.stop();

    return 0;
}

void cllbck_tim50hz(const ros::TimerEvent &event)
{
    static uint8_t data_temp = 0;

    /**
     * Read toggle data here,
     * then assign to data_temp using 'or' operation.
     *
     * Example assign of toggle0,
     * data_temp |= (toggle0 << TOGGLE_0_SHIFT);
     *
     *
     */

    std_msgs::UInt8 msg_toggle;
    msg_toggle.data = data_temp;
    pub_toggle_datas.publish(msg_toggle);
}

void load_config()
{
    char static_config_file[100], dynamic_config_file[100];
    std::string current_dir = ros::package::getPath("vision");
    sprintf(static_config_file, "%s/../../config/static_conf.yaml", current_dir.c_str());
    sprintf(dynamic_config_file, "%s/../../config/dynamic_conf.yaml", current_dir.c_str());

    YAML::Node static_config = YAML::LoadFile(static_config_file);
    YAML::Node dynamic_config = YAML::LoadFile(dynamic_config_file);

    toggles_pin[0] = static_config["Hardware"]["toggle0_pin"].as<int>();
    toggles_pin[1] = static_config["Hardware"]["toggle1_pin"].as<int>();
    toggles_pin[2] = static_config["Hardware"]["toggle2_pin"].as<int>();
    toggles_pin[3] = static_config["Hardware"]["toggle3_pin"].as<int>();
}