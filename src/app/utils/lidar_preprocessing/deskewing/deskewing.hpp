/**
 * @copyright (c) AI LAB - Konkuk Uni.
 * <br>All rights reserved. Subject to limited distribution and restricted disclosure only.
 * @author pauljiwon96@gmail.com, kchsu531@gmail.com, wodud3743@gmail.com
 * @file deskewing.hpp
 * @brief pointcloud deskwing input point cloud need time field header file
 * @version 1.0
 * @date 14-07-2022
 * @bug No known bugs
 * @warning No warnings
 */
#ifndef __DESKEWING_HPP__
#define __DESKEWING_HPP__

/* Includes */
#include <string>
// STL
#include <vector>
#include <deque>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>

class CDeskewing{
    CDeskewing(){}
    ~CDeskewing(){}

    template <typename pointType>
    bool Deskewing( const pcl::PointCloud<pointType>& i_pc_skewed,
                    double lidar_time_sec,
                    std::deque<std::pair<double, std::vector<double>>> time_motion_que,
                    pcl::PointCloud<pointType>& o_pc_deskewd);

    std::vector<double> PrediceModelCV(double prev_state[],
                            double angular_velocity[],
                            double motion_dt);
    std::vector<double> quaternionPredictionModel(double prev_state[],
                            double angular_velocity[],
                            double motion_dt);

    double AngleDiff_deg(double dRef_deg, double dRel_deg);
};

#endif // __DESKEWING_HPP__