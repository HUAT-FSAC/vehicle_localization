#include "ros/ros.h"
#include <fstream>
#include <string>
#include "common_msgs/HUAT_ASENSING.h"

void ins570dCallback(const common_msgs::HUAT_ASENSING::ConstPtr &msgs);

// TODO: relocate the csv file to root or tmp folder (when running in reak vehicle)
std::ofstream gps_out("../../../src/gps_imu_fusion/data/gps.csv", std::ios::app);
std::ofstream gyro_out("../../../src/gps_imu_fusion/data/gyro.csv", std::ios::app);
std::ofstream acc_out("../../../src/gps_imu_fusion/data/acc.csv", std::ios::app);

void ins570dCallback(const common_msgs::HUAT_ASENSING::ConstPtr &msgs)
{
    // since the gps and imu data are from the same source, their created time should be the same too.
    // or two data are updated differently, need to judge mannuly.

    // precision more than 10 is useless
    gps_out << std::fixed << std::setprecision(10) << 
        msgs->latitude << "," << msgs->longitude << "," << msgs->altitude << "," << 
        msgs -> north_velocity << "," << msgs -> east_velocity << "," << msgs -> ground_velocity << std::endl;
    
    acc_out << std::fixed << std::setprecision(10) << msgs->x_acc << "," << msgs->y_acc << "," << msgs->z_acc << std::endl;
    gyro_out << std::fixed << std::setprecision(10) << msgs->x_angular_velocity << "," << msgs->y_angular_velocity << "," << msgs->z_angular_velocity << std::endl;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gps_imu_fusion");

    ros::NodeHandle nh;
    ros::Subscriber INSsub = nh.subscribe<common_msgs::HUAT_ASENSING>("/INS/ASENSING_INS", 1, ins570dCallback);

    std::cout << "spining..." << std::endl;
    ros::spin();
}