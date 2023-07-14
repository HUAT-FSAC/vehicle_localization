/* vehicle_localization
 *
 * handles msgs form imu and republish to /imu/data and
 */

#include "ros/ros.h"
#include <ros/console.h>
#include <ostream>
#include <common_msgs/HUAT_ASENSING.h>
#include "common_msgs/HUAT_Carstate.h"
#include "filter_flow.h"
#include "imu_flow.h"
#include "gps_flow.h"

FilterFlow filter_flow("/home/neko/Code/dev_ws/src/vehicle_localization");
GPSData gps;
IMUData imu;

bool init_flag = false;

// callback method must have void return type
void insDataCallback(const common_msgs::HUAT_ASENSING::ConstPtr &msg)
{
    gps.time = imu.time = ros::Time::now().toSec();
    // std::cout << "gps&imu time is: " << gps.time << std::endl;

    gps.position_lla.x() = msg -> latitude;
    gps.position_lla.y() = msg -> longitude;
    gps.position_lla.z() = msg -> altitude;

    // TODO: could have a problem
    gps.velocity.x() = msg -> north_velocity;
    gps.velocity.y() = msg -> east_velocity;
    gps.velocity.z() = msg -> ground_velocity;

    GPSFlow::LLA2NED(gps);

    // imu part
    imu.angle_velocity.x() = msg -> x_angular_velocity;
    imu.angle_velocity.y() = msg -> y_angular_velocity;
    imu.angle_velocity.z() = msg -> z_angular_velocity;

    imu.linear_accel.x() = msg -> x_acc;
    imu.linear_accel.y() = msg -> y_acc;
    imu.linear_accel.z() = msg -> z_acc;

    if (init_flag) {
        std::cout << gps.position_lla << std::endl;
        std::cout << imu.linear_accel << std::endl;
        filter_flow.Predict(); 
    } else {
        filter_flow.InitFilter();
        init_flag = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vehicle_localization");
    ros::NodeHandle nh;
    std::cout << "init ros node (subscriber)..." << std::endl;
    ros::Subscriber sub_ins = nh.subscribe("/INS/ASENSING_INS", 10, insDataCallback);
    
    while (ros::ok())
    {
        ros::spin();
        // filter_flow.insertGpsData(gps);
        // filter_flow.insertImuData(imu); 
    }
}