#include "ros/ros.h"

ros::NodeHandle nh;


void ins570dCallback(const common_msgs::HUAT_ASENSING::ConstPtr &msgs)
{
    // since the gps and imu data are from the same source, their created time should be the same too.
    // or two data are updated differently, need to judge mannuly.

}

int msgHandlerInit()
{
    // do 

    // designed for INS570D IMU&GPS Module
    ros::Subscriber INSsub = nh.subscribe<common_msgs::HUAT_ASENSING>("/INS/ASENSING_INS", 1, doINSMsg);
}
