//
// Modified by NekoRect on 2023/07/14
//
#include "filter_flow.h"
#include "tool.h"
#include <string>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <common_msgs/HUAT_ASENSING.h>
#include "ros/ros.h"

// for msg
#include "gps_data.h"
#include "imu_data.h"

// callback method must have void return type
void insDataCallback(const common_msgs::HUAT_ASENSING::ConstPtr &msg)
{
    GPSData gps;
    IMUData imu;

    gps.time = imu.time = ros::Time::now().toSec();
    std::cout << "gps&imu time is: " << gps.time << std::endl;;

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

    // FilterFlow::imu_data_buff_.push_back(imu);
    // FilterFlow::gps_data_buff_.push_back(gps);
}

bool FilterFlow::PushData(GPSData gps, IMUData imu)
{
    imu_data_buff_.push_back(imu);
    gps_data_buff_.push_back(gps);
}

void initRos()
{
    ros::NodeHandle nh;
    ros::Subscriber sub_ins = nh.subscribe("/INS/ASENSING", 1000, insDataCallback);
}

Eigen::Matrix4d Vector2Matrix(const Eigen::Vector3d &vec)
{
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix.block<3, 1>(0, 3) = vec;

    return matrix;
}

// constrct func
FilterFlow::FilterFlow(const std::string &work_space_path)
    // clearify private var "work_space_path_" equals to arg "w_s_p"
    : work_space_path_(work_space_path)
{
    std::string config_file_path = work_space_path_ + "/config/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    data_path_ = config_node["data_path"].as<std::string>();

    filter_ptr_ = std::make_shared<ESKF>(config_node);
}

// 读取GPS和IMU数据

// no longer usable, because we are tying to provide IMU & GPS msgs though direct topics

// bool FilterFlow::ReadData()
// {
//     const std::string data_path = work_space_path_ + data_path_;

//     if (imu_flow_ptr_->ReadIMUData(data_path, imu_data_buff_) &&
//         gps_flow_ptr_->ReadGPSData(data_path, gps_data_buff_))
//     {
//         return false;
//     }

//     return false;
// }

// IMIU和GPS时间差不能超过10ms
bool FilterFlow::ValidGPSAndIMUData()
{
    curr_imu_data_ = imu_data_buff_.front();
    curr_gps_data_ = gps_data_buff_.front();

    double delta_time = curr_imu_data_.time - curr_gps_data_.time;

    // this part can loop, until it get its data right
    if (delta_time > 0.01)
    {
        // IMU来得晚了10ms，则GPS不满足条件
        gps_data_buff_.pop_front();
        return false;
    }

    if (delta_time < -0.01)
    {
        imu_data_buff_.pop_front();
        return false;
    }

    // no problem, poped just because latest msg has been saved to "curr_imu_data_" (private var)
    imu_data_buff_.pop_front();
    gps_data_buff_.pop_front();

    return true;
}


// don't know its use
bool FilterFlow::ValidIMUData()
{
    curr_imu_data_ = imu_data_buff_.front();
    imu_data_buff_.front();

    return true;
}

bool FilterFlow::ValidGPSData()
{
    curr_gps_data_ = gps_data_buff_.front();
    gps_data_buff_.pop_front();

    return true;
}

// 主函数流程
bool FilterFlow::Run()
{

    std::cout << "init ros node (subscriber)..." << std::endl;
    initRos();

    // no longer uses this func

    // ReadData();
    // printf("data read successfully： imu size = %d, gps size = %d\n", (int)imu_data_buff_.size(), (int)gps_data_buff_.size());
    

    // until queue pops out all elements!
    //
    // Since the imu and gps msg were published same time, there's no need to Vaild msgs time
    //
    // while (!imu_data_buff_.empty() && !gps_data_buff_.empty())
    // {
    //     if (!ValidGPSAndIMUData())
    //     { 
    //         // 确保初始的GPS和IMU时间不能相差超过10ms
    //         continue;
    //     }
    //     else
    //     {
    //         filter_ptr_->Init(curr_gps_data_, curr_imu_data_); // ESKF初始化
    //         break;
    //     }
    // }

    filter_ptr_ -> Init(curr_gps_data_, curr_imu_data_);

    // for some reasons
    gps_data_buff_.pop_front();
    imu_data_buff_.pop_front();

    // init file handles
    std::ofstream gt_file(work_space_path_ + "/data/gt.txt", std::ios::trunc);
    std::ofstream fused_file(work_space_path_ + "/data/fused.txt", std::ios::trunc);
    std::ofstream measured_file(work_space_path_ + "/data/measured.txt", std::ios::trunc);


    // Remove unnecessary while loop
    while (!imu_data_buff_.empty() && !gps_data_buff_.empty())
    {
        // since last while loop has vaild GPS and IMU data, curr_data_ now is ONE time after privious data_
        curr_imu_data_ = imu_data_buff_.front();
        curr_gps_data_ = gps_data_buff_.front();
        if (curr_imu_data_.time < curr_gps_data_.time)
        {                                         // IMU数据比当前GPS数据时间早，则使用IMU预测
            filter_ptr_->Predict(curr_imu_data_); // IMU预测
            imu_data_buff_.pop_front();
        }
        else
        { // IMU数据时间比GPS晚一点
            // filter_ptr_->Predict(curr_imu_data_); // 预测
            // imu_data_buff_.pop_front();

            filter_ptr_->Correct(curr_gps_data_); // GPS数据观测

            SavePose(fused_file, filter_ptr_->GetPose());
            SavePose(measured_file, Vector2Matrix(curr_gps_data_.position_ned));

            // SavePose(gt_file, Vector2Matrix(GPSFlow::LLA2NED(curr_gps_data_.true_position_lla)));
            gps_data_buff_.pop_front();
        }

        if (imu_data_buff_.empty() || gps_data_buff_.empty()) {
            std::cout << "imu / gps is empty" << std::endl;
        }
    }


    return true;
}

// 保存前3行4列
void FilterFlow::SavePose(std::ofstream &ofs, const Eigen::Matrix4d &pose)
{
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            ofs << pose(i, j);

            if (i == 2 && j == 3)
            {
                ofs << std::endl;
            }
            else
            {
                ofs << " ";
            }
        }
    }
}
