// copyright
#include "ros/ros.h"
#include <ros/console.h>
#include <ostream>
#include <common_msgs/HUAT_ASENSING.h>
#include "common_msgs/HUAT_Carstate.h"

common_msgs::HUAT_ASENSING imu_msg;

using namespace std;

void handleIMUMsg(const common_msgs::HUAT_ASENSING::ConstPtr &msg)
{
    // cout << "handling msg from IMU" << endl;
    imu_msg.latitude = msg -> latitude;
    imu_msg.longitude = msg -> longitude;
    imu_msg.altitude = msg -> altitude;

    // cout << msg -> ins_status << endl;
    cout.precision(15);
    cout << "lat:" << imu_msg.latitude << "\t" << "lon:" << imu_msg.longitude << endl;
    cout << "lat_std:" << imu_msg.latitude_std << "\t" << "lon:" << imu_msg.longitude_std << endl;
    cout << "azimuth:" << msg -> azimuth << endl << endl;

}

void publishOdomMsg()
{

}

void publishIMUMsg()
{
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vehicle_localization");
    // ROS_DEBUG("VL up and running...");

    ros::NodeHandle nh;
    ros::Subscriber imu = nh.subscribe("/INS/ASENSING_INS", 1, handleIMUMsg);

    ros::Rate rate(10);
    // 10 secs?


    while (ros::ok())
    {
        // ros::spinOnce();
        // rate.sleep();
        // why sleep?
        ros::spin();
    }
}