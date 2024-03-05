/**
 * @copyright (c) AI LAB - Konkuk Uni.
 * <br>All rights reserved. Subject to limited distribution and restricted disclosure only.
 * @author pauljiwon96@gmail.com
 * @file range_image.hpp
 * @brief range_image generation module header file
 * @version 1.0
 * @date 25-07-2022
 * @bug No known bugs
 * @warning No warnings
 */

#ifndef __RANGE_IMAGE_HPP__
#define __RANGE_IMAGE_HPP__

/* Includes */
#include <string>
// STL
#include <vector>
#include <deque>
#include <map>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/io.h>

// OpenCV
#include <opencv2/core/mat.hpp>

// Custom include
#include <util/ini_handler_cpp/c_ini.hpp>
#include <util/debugprintutil/debug_print.hpp>

#include <point_type/pointXYZIRT.hpp>

class CRangeImage{
public:
    enum EnumMultiEchoFilteringMethod
    {
        COMMING_FIRST,
        COMMING_LAST,
        DISTANCE_NEAR,
        DISTANCE_FAR,
        INTENSITY_STRONG
    };

    CRangeImage(){};
    CRangeImage(std::string ini_file_path)
    {Init(ini_file_path);}
    ~CRangeImage()
    {
        ReleaseMemory(m_p_range_img_data);
        for(auto iter = m_map_image_mask.begin();iter != m_map_image_mask.end(); iter++)
            ReleaseMemory(iter->second);
        for(auto iter = m_map_float_image_mask.begin();iter != m_map_float_image_mask.end(); iter++)
            ReleaseMemory(iter->second);
    }

public:
    /* Variables */
    bool        m_b_is_init;

    std::string m_str_lidar_name;
public:
    // Configuration
    CINI_H      m_ini_handler;
    bool        m_b_init_complete;
    bool        m_b_use_ring_data;
    bool        m_b_is_ring_ascent;

    std::vector<double>   m_vecf_horizontal_section_angle_deg;
    std::vector<double>   m_vecf_horizontal_section_resolution_deg;
    std::vector<uint16_t> m_veci_horizontal_section_start_index;
    std::vector<double>   m_vecd_horizontal_section_boundary_deg;
    std::vector<double>   m_vecf_vertical_section_angle_deg;
    std::vector<double>   m_vecf_vertical_section_resolution_deg;
    std::vector<uint16_t> m_veci_vertical_section_start_index;
    std::vector<double>   m_vecd_vertical_section_boundary_deg;

    EnumMultiEchoFilteringMethod m_filtering_method;

    // Image specs
    uint16_t    m_n_height;
    uint16_t    m_n_width;
    uint32_t    m_n_img_size;
    uint16_t    m_min_range_threshold_m;
    uint16_t    m_max_range_threshold_m;

    // Data
    std::map<std::string, uint8_t**>                m_map_image_mask;
    std::map<std::string, float**>                  m_map_float_image_mask;
    boost::shared_ptr<pcl::PointCloud<PointXYZIRT>> m_pcptr_input_pc;
    std::vector<std::vector<int>>                   m_i_v_point_to_image_idx;

    float    **m_p_range_img_data;
    int      **m_p_index_img_data;

    // Debug
    bool m_b_print_message;

public:
    /* Functions */
    bool Init(std::string ini_path);
protected:
    template<typename typeOfVar>
    void AllocateMemory(typeOfVar** &pointer);
    template<typename typeOfVar>
    void ReleaseMemory(typeOfVar** &pointer);
protected:
    void ResetValue();
    bool SetSectionBoundary();
    bool SetSectionStartIndices();
public:
    template <typename pointType>
    bool SetCloud(boost::shared_ptr<pcl::PointCloud<pointType>> pcptr_input_pc);
    boost::shared_ptr<pcl::PointCloud<PointXYZIRT>> GetCloud();

protected:
    bool GenerateRangeImg(boost::shared_ptr<pcl::PointCloud<PointXYZIRT>> i_pcptr);
    bool GenerateRangeImg();

public:
    void GenerateMask(std::string key);
    void GenerateFloatMask(std::string key);
    bool IsMask(std::string key);

protected:
    uint16_t GetRowIndex( float in_vertical_angle );
    uint16_t GetColIndex( float in_horizontal_horizontal_angle );

public:
    uint8_t**           GetMaskImg(std::string key);
    float**             GetFloatMaskImg(std::string key);
    std::vector<int>    GetPixelPointIdx(int point_idx);

    int GetIdx(uint16_t row, uint16_t col);
    float      GetRange(uint16_t row, uint16_t col);

    cv::Mat GetRangeCVImage();
    cv::Mat GetIndexCVImage();
    cv::Mat GetMaskCVImage(std::string key);
    cv::Mat GetFloatMaskCVImage(std::string key);

    bool GetMaskIdx(std::string key, std::vector<int>& o_idx_vector);
    bool GetNonMaskIdx(std::string key, std::vector<int>& o_idx_vector);
    bool GetMaskIndices(std::string key, std::vector<std::vector<int>>& o_idx_vectors);
    bool GetMaskPointCloud(std::string key, boost::shared_ptr<pcl::PointCloud<PointXYZIRT>> output_pcs);
    bool GetMaskPointClouds(std::string key, std::vector<boost::shared_ptr<pcl::PointCloud<PointXYZIRT>>>& output_pcs);

    uint16_t Height();
    uint16_t Width();
    uint32_t Size();

    bool IsInit();
protected:
    // Configuration
    bool ParseINIFile();
};


template<typename typeOfVar>
void CRangeImage::AllocateMemory(typeOfVar** &pointer)
{
    pointer = new typeOfVar*[m_n_height];
    for(int idx = 0; idx < m_n_height; idx ++)
    {
        pointer[idx] = new typeOfVar[m_n_width];
        memset(pointer[idx], 0, m_n_width*sizeof(typeOfVar));
    }

    return;
}
template<typename typeOfVar>
void CRangeImage::ReleaseMemory(typeOfVar** &pointer)
{
    if(pointer == NULL)
        return;

    for(int idx = 0; idx < m_n_height; idx ++)
        delete[] pointer[idx];

    delete[] pointer;

    return;
}

/*
Brief
- Point cloud input function of range image
Input
- pcptr_input_pc: input point cloud to make range image
Output
- None
Return
- false: class is not initizilized or input oint cloud is empty
- ture : Normaly operated.
*/
template <typename pointType>
bool CRangeImage::SetCloud(boost::shared_ptr<pcl::PointCloud<pointType>> pcptr_input_pc)
{
    if(m_b_is_init == false)
        return false;
    if(pcptr_input_pc == NULL || pcptr_input_pc->empty() == true)
        return false;
    if(m_pcptr_input_pc == NULL)
        m_pcptr_input_pc.reset(new pcl::PointCloud<PointXYZIRT>);

    pcl::copyPointCloud(*pcptr_input_pc, *m_pcptr_input_pc);
    // m_pcptr_input_pc = pcptr_input_pc;
    if(GenerateRangeImg(m_pcptr_input_pc) == false)
        return false;

    return true;
}

#endif // __RANGE_IMAGE_HPP__