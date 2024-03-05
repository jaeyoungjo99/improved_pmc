/**
 * @copyright (c) AI LAB - Konkuk Uni.
 * <br>All rights reserved. Subject to limited distribution and restricted disclosure only.
 * @author pauljiwon96@gmail.com, wodud3743@gmail.com
 * @file label_seg.hpp
 * @brief Range image based fast and efficent segmentation header file. Origin algorithm is [I. Bogoslavskyi and C. Stachniss,
 * “Fast Range Image-based Segmentation of Sparse 3D Laser Scans for Online Operation,”
 * Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems, pp. 163-169, 2016.].
 * This algorithm has some changes
 * @version 1.0
 * @date 07-09-2022
 * @bug No known bugs
 * @warning No warnings
 */

#ifndef __LABEL_SEG_HPP__
#define __LABEL_SEG_HPP__

#include <string>
#include <vector>
#include <utility>
#include "util/ini_handler_cpp/c_ini.hpp"
#include "range_image/range_image.hpp"
#include "point_type/pointXYZIRT.hpp"

class CLabelSeg{
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
    CLabelSeg();
    ~CLabelSeg();

    /* Variables */
    // Object segmentation
    std::vector<std::pair<int8_t, int8_t> > m_v_neighbor_iter; // neighbor iterator for segmentaiton process

    std::vector<uint16_t> m_v_ui16_pushed_idx_x; // array for tracking points of a segmented object
    std::vector<uint16_t> m_v_ui16_pushed_idx_y;

    std::vector<uint16_t> m_v_ui16_queue_idx_x; // array for breadth-first search process of segmentation, for speed
    std::vector<uint16_t> m_v_ui16_queue_idx_y;

    std::vector<std::vector<bool>> m_v_b_segmented_image;

    int m_i_row_size;
    int m_i_col_size;

    std::vector<Pixels> m_v_object_pixels;
    boost::shared_ptr<pcl::PointCloud<PointXYZIRT>> pcptr_input;

    // Configuration
    CINI_H  m_ini_handler;
    bool    m_cfg_b_use_ini_config;
    int     m_cfg_i_neighbor_size;
    float   m_cfg_f_range_threshold_m;
    float   m_cfg_f_dist_threshold_m;
    float   m_cfg_f_segment_theta_deg;
    int     m_cfg_i_segment_min_point_num;
    int     m_cfg_i_segment_valid_point_num;
    int     m_cfg_i_segment_valid_line_num;

    bool    m_cfg_b_print_message;

    /* Functions */
public:
    bool Init(std::string ini_path);
    void NeighborAssign(unsigned int neighbor_size);
    bool FilterObject(CRangeImage& range_image);
    bool FilterObject(CRangeImage& range_image, pcl::PointCloud<PointXYZIRT>::Ptr rph_calibrated_pcptr);
    void LabelComponents(CRangeImage& range_image, uint16_t row, uint16_t col);
    void Reset();
    bool ParseIni();
};
#endif //__LABEL_SEG_HPP__