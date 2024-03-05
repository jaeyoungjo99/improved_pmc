/**
 * @copyright (c) AI LAB - Konkuk Uni.
 * <br>All rights reserved. Subject to limited distribution and restricted disclosure only.
 * @author pauljiwon96@gmail.com wodud3743@gmail.com
 * @file line_fit.hpp
 * @brief line_fit based ground segmentation header file. Origin article is "M. Himmelsbach, F. v. Hundelshausen and
 * H. . -J. Wuensche, "Fast segmentation of 3D point clouds for ground vehicles,"
 * 2010 IEEE Intelligent Vehicles Symposium, 2010, pp. 560-565, doi: 10.1109/IVS.2010.5548059."
 * @version 1.0
 * @date 03-08-2022
 * @bug No warnings
 * @warning No warnings
 */

#ifndef __LINE_FIT_HPP__
#define __LINE_FIT_HPP__

#include <string>
#include "util/ini_handler_cpp/c_ini.hpp"
#include "range_image/range_image.hpp"
#include "point_type/pointXYZIRT.hpp"

class CLineFit
{
public:
    CLineFit ();
    ~CLineFit();

    /* Variables */
private:
    CINI_H m_cini_handler;

    // Configuration
    bool    m_cfg_b_use_ini_config;
    float   m_cfg_b_ground_point_max_distance;
    float   m_cfg_f_ground_angle_threshold;
    float   m_cfg_f_ground_height_angle_threshold;
    float   m_f_lidar_height_m;
    bool    m_cfg_b_use_angle_from_ground;

    /* Functions */
public:
    bool Init(std::string ini_path);
    bool FilterGround(CRangeImage& range_image);
    bool FilterGround(CRangeImage& range_image, pcl::PointCloud<PointXYZIRT>::Ptr rph_calibrated_pcptr);

public:
    bool ParsingIni();
};
#endif //  __LINE_FIT_HPP__