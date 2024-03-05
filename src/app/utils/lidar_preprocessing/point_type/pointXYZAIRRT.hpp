/**
 * @copyright (c) AI LAB - Konkuk Uni.
 * <br>All rights reserved. Subject to limited distribution and restricted disclosure only.
 * @author pauljiwon96@gmail.com dudfhr3349@gmail.com
 * @file lidar_preprocessing.hpp
 * @brief preprocessing tools for lidar
 * @version 1.0
 * @date 14-07-2022
 * @bug No known bugs
 * @warning No warnings
 */
#ifndef __POINTXYZAIRRT_HPP__
#define __POINTXYZAIRRT_HPP__

/* Includes */
#include <string>
// STL
#include <vector>
#include <deque>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>

/* Types */

struct pointXYZAIRRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(pointXYZAIRRT,
                                 (float, x, x) 
                                 (float, y, y) 
                                 (float, z, z) 
                                 (float, intensity, intensity)
                                 (uint32_t, t, t) 
                                 (uint16_t, reflectivity, reflectivity)
                                 (uint8_t, ring, ring) 
                                 (uint16_t, noise, noise) 
                                 (uint32_t, range, range)
)
#endif // __POINTXYZAIRRT_HPP__


