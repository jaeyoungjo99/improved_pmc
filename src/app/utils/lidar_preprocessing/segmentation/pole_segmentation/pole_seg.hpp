/**
 * @copyright (c) AI LAB - Konkuk Uni.
 * <br>All rights reserved. Subject to limited distribution and restricted disclosure only.
 * @author pauljiwon96@gmail.com, wodud3743@gmail.com
 * @file pole_seg.hpp
 * @brief Pole segmentation
 * @version 1.0
 * @date 08-09-2022
 * @bug No known bugs
 * @warning No warnings
 */


#ifndef __POLE_SEG_HPP__
#define __POLE_SEG_HPP__

#include <string>
#include "util/ini_handler_cpp/c_ini.hpp"
#include "range_image/range_image.hpp"
#include "point_type/pointXYZIRT.hpp"

#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <numeric>


class CPoleSeg{
    const float DEG2RAD = 0.0174532925199;
    const float RAD2DEG = 57.2957795131;

    typedef std::pair<int, int> PixelCoord;
    typedef std::vector<PixelCoord> Pixels;
    struct by_size_decent
    {
        bool operator()(Pixels const &left, Pixels const &right)
        {
            return left.size() > right.size();
        }
    };
public:
    CPoleSeg();
    ~CPoleSeg();

    std::vector<Pixels> m_v_pole_pixels;

    // Configuration
    CINI_H  m_ini_handler;
    bool    m_cfg_b_use_ini_config;

    float   m_cfg_f_max_cluster_width;
    float   m_cfg_f_min_cluster_length;
    float   m_cfg_f_max_cluster_length;
    float   m_cfg_f_min_width_to_length;
    float   m_cfg_f_min_diameter_to_height;
    float   m_cfg_f_min_radius;
    float   m_cfg_f_max_radius;
    float   m_cfg_f_max_flattening;
    float   m_cfg_f_min_linearity;
    float   m_cfg_f_min_highest_point;
    float   m_cfg_f_max_lowest_point;
    float   m_cfg_f_min_pole_angle_threshold_deg;

    bool    m_cfg_b_print_message;

    int poleCount = 0;

    /* Functions */
public:
    bool Init(std::string ini_path);

    void Reset();
    bool ParseIni();
    
    bool FilterPole(CRangeImage& range_image);
    bool FilterPole(CRangeImage& range_image, pcl::PointCloud<PointXYZIRT>::Ptr rph_calibrated_pcptr);
    bool CheckPole(pcl::PointCloud<PointXYZIRT>& i_object_point_cloud, float o_cluster_info[]);

    void fitCircle(float cluster_info[], float eigen_value[], 
                   std::vector<float> cluster_x, std::vector<float> cluster_y, std::vector<float> cluster_z);
    static bool cmp_vector(std::vector<float> &v1, std::vector<float> &v2);
};


#endif // __POLE_SEG_HPP__