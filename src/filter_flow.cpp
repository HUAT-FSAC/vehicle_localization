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

std::ofstream gt_file("/home/neko/Code/dev_ws/src/vehicle_localization/data/gt.txt", std::ios::trunc);
std::ofstream fused_file("/home/neko/Code/dev_ws/src/vehicle_localization/data/fused.txt", std::ios::trunc);
std::ofstream measured_file("/home/neko/Code/dev_ws/src/data/measured.txt", std::ios::trunc);

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

    // std::cout << config_file_path << std::endl;

    YAML::Node config_node = YAML::LoadFile(config_file_path);
    data_path_ = config_node["data_path"].as<std::string>();
    // direct designate filter
    filter_ptr_ = std::make_shared<ESKF>(config_node);
}

bool FilterFlow::InitFilter()
{
    std::cout << "filter init" << std::endl;
    filter_ptr_->Init(curr_gps_data_, curr_imu_data_);
    // init file handles
}

int i =0;

bool FilterFlow::Predict()
{
    std::cout << "predicting..." << std::endl;

    // since last while loop has vaild GPS and IMU data, curr_data_ now is ONE time after privious data_
    
    if (i < 9)
    {                                         // IMU数据比当前GPS数据时间早，则使用IMU预测
        filter_ptr_->Predict(curr_imu_data_); // IMU预测
        // imu_data_buff_.pop_front();
        i++;
    }
    else
    { // IMU数据时间比GPS晚一点
        filter_ptr_->Predict(curr_imu_data_); // 预测

        filter_ptr_->Correct(curr_gps_data_); // GPS数据观测

        SavePose(fused_file, filter_ptr_->GetPose());
        SavePose(measured_file, Vector2Matrix(curr_gps_data_.position_ned));
        i = 0;
    }
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