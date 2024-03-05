/**
 * @copyright (c) AI LAB - Konkuk Uni.
 * <br>All rights reserved. Subject to limited distribution and restricted disclosure only.
 * @author pauljiwon96@gmail.com dudfhr3349@gmail.com
 * @file line_based_segmentation.hpp
 * @brief line based segmentation module header file
 * @version 1.0
 * @date 03-08-2022
 * @bug No known bugs
 * @warning No warnings
 */

#ifndef __LINE_BASE_SEGMENTATION_HPP__
#define __LINE_BASE_SEGMENTATION_HPP__

/* Includes */
#include <string>
// STL
#include <vector>
#include <set>
#include <memory> // shared_ptr
#include <chrono>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h> // pcl::copyPointCloud();
#include <pcl/common/eigen.h>

// Utiles
#include "util/ini_handler_cpp/c_ini.hpp"
#include "point_type/pointXYZIRT.hpp"

// Segmentation tools
#include "range_image/range_image.hpp"
#include "segmentation/ground_segmentation/line_fit.hpp"
#include "segmentation/object_segmentation/label_seg.hpp"
#include "segmentation/object_segmentation/vertical_seg.hpp"
#include "segmentation/pole_segmentation/pole_seg.hpp"

using pointType = PointXYZIRT;

class CLineBasedSegmentation
{
    enum EnumGroundSegmentationMethod
    {
        GROUND_SEG_OFF,
        LINEFIT,
        IMAGEPATCHWORK
    };
    enum EnumObjectSegmentationMethod
    {
        OBJECT_SEG_OFF,
        LABELSEG
    };
    enum EnumPoleSegmentationMethod
    {
        POLESEGNAME
    };

public:
    CLineBasedSegmentation()
    {
        m_p_line_fit.reset(new CLineFit);
        m_p_label_seg.reset(new CLabelSeg);
        m_p_vertical_seg.reset(new CVerticalSeg);
        m_p_pole_seg.reset(new CPoleSeg);
        return ;}
    ~CLineBasedSegmentation(){return ;}

    /* Variables */
public:
    // Configuration
    CINI_H m_ini_handler;
    std::string                  m_str_ini_path;
    std::string                  m_str_range_image_ini_path;
    EnumGroundSegmentationMethod m_enum_ground_segmentation_method;
    EnumObjectSegmentationMethod m_enum_object_segmentation_method;
    EnumPoleSegmentationMethod   m_enum_pole_segmentation_method;

    float       m_f_lidar_x_m;
    float       m_f_lidar_y_m;
    float       m_f_lidar_z_m;
    float       m_f_lidar_roll_deg;
    float       m_f_lidar_pitch_deg;
    float       m_f_lidar_yaw_deg;



    // Input point cloud
    pcl::PointCloud<PointXYZIRT>::Ptr   m_pcptr_input;

    // Calibrated point cloud
    pcl::PointCloud<PointXYZIRT>::Ptr   m_pcptr_rph_calibrated;

    // Output point cloud for debug
    pcl::PointCloud<PointXYZIRT>::Ptr   m_pcptr_output;

    // Segmented indexes of points at input point cloud each segment
    std::vector<int>               m_vec_ground_idx;
    std::vector<int>               m_vec_non_ground_idx;
    std::vector<std::vector<int>>  m_vec_objects_idx;
    std::vector<std::vector<int>>  m_vec_poles_idx;

    std::shared_ptr<CRangeImage> m_p_range_image; // Range image class

    // Ground segmentation
    std::shared_ptr<CLineFit>    m_p_line_fit;
    // Object segmentation
    std::shared_ptr<CLabelSeg>   m_p_label_seg;
    
    std::shared_ptr<CVerticalSeg> m_p_vertical_seg;
    // Pole segmentation
    std::shared_ptr<CPoleSeg>    m_p_pole_seg;


    /* Functions */
public:
    bool Init(std::string ini_path);
    void Reset();

    bool SetRangeImage(std::shared_ptr<CRangeImage>& i_p_range_image);
    std::shared_ptr<CRangeImage> GetRangeImage();

    template <typename pointType>
    bool SetCloud(boost::shared_ptr<pcl::PointCloud<pointType>> i_pcptr);
    boost::shared_ptr<pcl::PointCloud<PointXYZIRT>> GetCloud();
    boost::shared_ptr<pcl::PointCloud<PointXYZIRT>> GetRPHCalibCloud();

    bool FilterGrounds();
    bool FilterObjects();
    bool FilterVertical();
    bool FilterPoles();

    bool EuclideanCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr i_point_cloud);

    bool filterCluster(std::vector<float> cluster_x,
                       std::vector<float> cluster_y,
                       std::vector<float> cluster_z);
    void fitCircle(float cluster_info[],float eigen_value[],
            std::vector<float> cluster_x, std::vector<float> cluster_y, std::vector<float> cluster_z);

    bool GetGroundIdx(      std::vector<int>               &vec_grounds_idx);
    bool GetNonGroundIdx(   std::vector<int>               &vec_non_ground_idx);
    bool GetObjectsIdx(     std::vector<std::vector<int>>  &vec_objects_idx);
    bool GetPolesIdx(       std::vector<std::vector<int>>  &vec_poles_idx);

    bool GetGroundPC(       pcl::PointCloud<PointXYZIRT>::Ptr               o_pcptr_ground);
    bool GetNonGroundPC(    pcl::PointCloud<PointXYZIRT>::Ptr               o_pcptr_non_ground);
    bool GetObjectsPC(      std::vector<pcl::PointCloud<PointXYZIRT>::Ptr>  &o_vec_pcptr_objects);
    bool GetPolesPC(        std::vector<pcl::PointCloud<PointXYZIRT>::Ptr>  &o_vec_pcptr_poles);
    bool GetPolesBottomPC(        std::vector<pcl::PointCloud<PointXYZIRT>::Ptr>  &o_vec_pcptr_poles);
    bool GetPolesPlanePC(        std::vector<pcl::PointCloud<PointXYZIRT>::Ptr>  &o_vec_pcptr_poles, double* model_coeff);
    bool GetPolesMidPC(        std::vector<pcl::PointCloud<PointXYZIRT>::Ptr>  &o_vec_pcptr_poles);


    bool GetObjectsPolygon(); // Need to update definition
    bool GetPolesPolygon(); // Need to update definition

    /* tools */
private:
    bool ParsingIni();
};

#endif // __LINE_BASE_SEGMENTATION_HPP__