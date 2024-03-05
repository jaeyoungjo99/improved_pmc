/**
 * @copyright (c) AI LAB - Konkuk Uni.
 * <br>All rights reserved. Subject to limited distribution and restricted disclosure only.
 * @author pauljiwon96@gmail.com, wns913@gmail.com
 * @file zrpEstimation.hpp
 * @brief sensor height roll pitch estimation module header file.
 * @version 1.0
 * @date 18-07-2022
 * @bug No known bugs
 * @warning No warnings
 */

#ifndef __ZRPESTIMATION_HPP__
#define __ZRPESTIMATION_HPP__

/* Includes */
#include <string>
// STL
#include <vector>
#include <deque>
#include <algorithm>
#include <chrono>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/passthrough.h>


// Utils
#include <util/ini_handler_cpp/c_ini.hpp>
#include <util/debugprintutil/debug_print.hpp>

class CZRPEstimation{
private:
    typedef std::vector<double> ROI; // ROI = x_min_m, x_max_m, y_min_m, y_max_m, z_min_m, z_max_m

public:

    CZRPEstimation();
    ~CZRPEstimation(){}

public:
    bool Init(std::string ini_path);

public:
    template<typename pointType>
    bool run(const pcl::PointCloud<pointType>& i_pc);

public:
    bool GetZRP(std::vector<double>& o_vec_zrp_m_rad);

public:
    bool GetRMSE(std::vector<double>& o_vec_point_to_plane_distance_rmse);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> tmp_debug_pcptr_roi_pc;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_debug_pcptr_sampling_points;
    std::vector<ROI> tmp_debug_vec_ROI;
    std::vector<Eigen::Vector4f> tmp_debug_vec_plane_model;
    std::vector<double> tmp_debug_vec_point_to_plane_distance_rmse;

private:
    // Ini Parser
    CINI_H m_ini_parser;

    // cfg variables
    bool m_cfg_b_debug_flag;
    std::vector<ROI> m_cfg_vec_ROI;
    int m_cfg_n_seed_points;
    int m_cfg_n_plane_point_treshold;
    double m_cfg_d_init_margin_of_patchwork;
    double m_cfg_d_margin_of_patchwork;
    int m_cfg_n_plane_fitting_iteration;
    double m_cfg_d_flat_gorund_included_angle_threshold_deg;
    double m_cfg_d_convergence_dtheta_deg;
    double m_cfg_d_convergence_dheight_m;
    double m_cfg_b_plane_rmse_treshold;
    double m_cfg_n_moving_average_window_size;

    // variables for overal run
    bool m_b_run_success_flag; // Flag for check run success

    // variables for initialize
    bool m_b_initialize_flag;

    // zrpCalculation result variables
    std::vector<double> m_vec_zrp_m_rad; // m_vec_zrp_m_rad : [z_m, roll_rad, pitch_rad]
    std::vector<double> m_vec_point_to_plane_distance_rmse; // m_vec_point_to_plane_distance_rmse : [roi_1_rmse, roi_2_rmse, ...]

private:
    bool ProcessINI();

private:
    bool PCRoiExtractor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& i_pcptr,
                        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& o_vec_pcptr_roi);

    bool PlaneDetection(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& i_vec_pcptr_roi,
                        std::vector<Eigen::Vector4f>& o_vec_plane_model);

    bool zrpCalculation(const std::vector<Eigen::Vector4f>& i_vec_plane_model);

private:
    void PlaneFitting(const pcl::PointCloud<pcl::PointXYZ>::Ptr& i_plane_pcptr, Eigen::Vector4f& o_plane_coeff);

    void PlanePointSampling(const pcl::PointCloud<pcl::PointXYZ>::Ptr& i_roi_pc,const  Eigen::Vector4f& i_plane_coeff, pcl::PointCloud<pcl::PointXYZ>::Ptr& o_plane_pc, double& o_RMSE);
};

#endif // __ZRPESTIMATION_HPP__