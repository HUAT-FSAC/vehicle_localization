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
#include "filter_flow.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vehicle_localization");

    FilterFlow filter_flow("/home/neko/Code/dev_ws");

    filter_flow.Run();

    while (ros::ok())
    {
        ros::spin();
    }
}