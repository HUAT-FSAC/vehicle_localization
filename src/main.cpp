/* vehicle_localization
 *
 * handles msgs form imu and republish to /imu/data and
 *
 *
 */

#include "ros/ros.h"
#include <ros/console.h>
#include <ostream>
#include <common_msgs/HUAT_ASENSING.h>
#include "common_msgs/HUAT_Carstate.h"

// funcs
void handleIMUMsg(const common_msgs::HUAT_ASENSING::ConstPtr &msg);


void handleIMUMsg(const common_msgs::HUAT_ASENSING::ConstPtr &msg)
{
    std::cout << "handling msg from IMU" << std::endl;

    std::cout.precision(15);
    std::cout << "lat:" << msg->latitude << "\t"
              << "lon:" << msg->longitude << std::endl;
    std::cout << "sec of week:" << msg->sec_of_week	<< std::endl;
}

// geometry_msgs::Quaternion calcQuaternion(double roll, double pitch, double yaw)
// {
//     geometry_msgs::Quaternion orientation;

//     orientation.w = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) +
//                     sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
//     orientation.x = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) -
//                     cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
//     orientation.y = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) +
//                     sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
//     orientation.z = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) -
//                     sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);

//     std::cout << "w:" << orientation.w << " x:" << orientation.x << " y:" << orientation.y << " z:" << orientation.z << std::endl;

//     return orientation;
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vehicle_localization");

    ros::NodeHandle nh;
    ros::Subscriber imu = nh.subscribe("/INS/ASENSING_INS", 1, handleIMUMsg);

    // imu_pub = nh.advertise<geometry_msgs::Imu>("/ins/data", 1);

    while (ros::ok())
    {
        ros::spin();
    }
}