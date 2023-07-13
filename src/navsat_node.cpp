// copyright
#include "ros/ros.h"
#include "math.h"
#include "sensor_msgs/NavSatFix.h"
#include <common_msgs/HUAT_ASENSING.h>

ros::NodeHandle nh;
ros::Publisher sat_pub = nh.advertise<sensor_msgs::NavSatFix>("/gps/fix", 10);

void processRawGPSData(const common_msgs::HUAT_ASENSING::ConstPtr &msg)
{
    sensor_msgs::NavSatFix sat_fix_msg;
    sat_fix_msg.header.stamp = ros::Time::now();
    sat_fix_msg.header.frame_id = "odom";
    sat_fix_msg.latitude = msg -> latitude;
    sat_fix_msg.longitude = msg -> longitude;
    sat_fix_msg.altitude = msg -> altitude;

    sat_pub.publish(sat_fix_msg);
}

int main()
{
    double freq;
    double delay;
    bool publish_filtered_gps = true;

    ros::Subscriber sat_fix = nh.subscribe("/INS/ASENSING_INS", 1, processRawGPSData);
    ros::spin();

    return 0;
}