//
// Created by meng on 2021/2/24.
//
#ifndef FILTER_FLOW_H
#define FILTER_FLOW_H

#include "eskf.h"
#include "ekf.h"
#include "imu_flow.h"
#include "gps_flow.h"
#include "filter_interface.h"
#include "ros/ros.h"
#include <memory>
#include <deque>
#include <iostream>
#include "common_msgs/HUAT_ASENSING.h"

class FilterFlow
{
public:
    FilterFlow() = default;
    FilterFlow(const std::string &work_space_path);

    bool ValidGPSData();

    //custom funcs
    bool InitFilter();
    bool Predict();

    /*!
     * 保存位姿，为kitti格式
     * @param ofs
     * @param pose
     */
    void SavePose(std::ofstream &ofs, const Eigen::Matrix4d &pose);

    // void insertImuData(const IMUData &imu)
    // {
    //     imu_data_buff_.push_back(imu);
    // }

    // void insertGpsData(const GPSData &gps)
    // {
    //     gps_data_buff_.push_back(gps);
    // }

private:
    std::shared_ptr<FilterInterface> filter_ptr_; // 滤波方法
    std::shared_ptr<IMUFlow> imu_flow_ptr_;
    std::shared_ptr<GPSFlow> gps_flow_ptr_;

    // std::deque<IMUData> imu_data_buff_(500);
    // std::deque<GPSData> gps_data_buff_(500);

    IMUData curr_imu_data_;
    GPSData curr_gps_data_;

    const std::string work_space_path_;
    std::string data_path_;
};

#endif // GPS_IMU_FUSION_ESKF_FLOW_H
