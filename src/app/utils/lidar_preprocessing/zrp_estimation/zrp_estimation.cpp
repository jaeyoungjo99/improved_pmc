/**
 * @copyright (c) AI LAB - Konkuk Uni.
 * <br>All rights reserved. Subject to limited distribution and restricted disclosure only.
 * @author pauljiwon96@gmail.com, wns913@gmail.com
 * @file zrpEstimation.cpp
 * @brief sensor height roll pitch estimation module source file.
 * @version 1.0
 * @date 06-10-2022
 * @bug No known bugs
 * @warning No warnings
 */

#include "zrp_estimation/zrp_estimation.hpp"

CZRPEstimation::CZRPEstimation():
m_cfg_b_debug_flag(false),
m_b_initialize_flag(false),
m_b_run_success_flag(false)
{}

/*
Brief
- Read algorithm configuration
Input
- ini_path: path of configuration ini file
Output
- none
Return
- true/false, false has some error like fail to read some configuration parameter
*/
bool CZRPEstimation::Init(std::string ini_path)
{
    m_ini_parser.Init(ini_path.c_str());
    // std::cout<< ini_path.c_str()<<std::endl;
    DebugPrintInfo("[zrpEstimation] Ini file path : " + ini_path, m_cfg_b_debug_flag);
    if(ProcessINI() == false)
    {
        DebugPrintError("[zrpEstimation] Ini file parsing fail!",m_cfg_b_debug_flag);
        return false;
    }

    tmp_debug_pcptr_sampling_points.reset(new pcl::PointCloud<pcl::PointXYZ>);

    m_b_initialize_flag = true;

    tmp_debug_vec_ROI = m_cfg_vec_ROI;

    DebugPrintInfo("[zrpEstimation] Initialize done",m_cfg_b_debug_flag);
    return true;
}

bool CZRPEstimation::ProcessINI(){
    // Debug
    bool check = true;
    if(m_ini_parser.ParseConfig("zrpEstimation", "m_cfg_b_debug_flag", m_cfg_b_debug_flag) != true)	return false;

    // ROI filtering
    // ROI = x_min_m, x_max_m, y_min_m, y_max_m, z_min_m, z_max_m
    double number_of_ROI;
    if(m_ini_parser.ParseConfig("zrpEstimation", "number_of_ROI", number_of_ROI) != true)	return false;
    for(int roi_idx = 0; roi_idx < number_of_ROI; roi_idx++)
    {
        std::vector<double> ROI;
        std::string ROI_name = "ROI_" + std::to_string(roi_idx); // ROI_0, ROI_1, ...
        if(m_ini_parser.ParseConfig("zrpEstimation", ROI_name, ROI) != true)	return false;
        m_cfg_vec_ROI.push_back(ROI);
    }
    if(m_ini_parser.ParseConfig("zrpEstimation", "m_cfg_n_plane_point_treshold", m_cfg_n_plane_point_treshold) != true)	return false;

    // Plane Extraction
    if(m_ini_parser.ParseConfig("zrpEstimation", "m_cfg_n_seed_points", m_cfg_n_seed_points) != true)	return false;
    if(m_ini_parser.ParseConfig("zrpEstimation", "m_cfg_d_init_margin_of_patchwork", m_cfg_d_init_margin_of_patchwork) != true)	return false;
    if(m_ini_parser.ParseConfig("zrpEstimation", "m_cfg_n_plane_fitting_iteration", m_cfg_n_plane_fitting_iteration) != true)	return false;
    if(m_ini_parser.ParseConfig("zrpEstimation", "m_cfg_d_margin_of_patchwork", m_cfg_d_margin_of_patchwork) != true)	return false;
    if(m_ini_parser.ParseConfig("zrpEstimation", "m_cfg_d_convergence_dtheta_deg", m_cfg_d_convergence_dtheta_deg) != true)	return false;
    if(m_ini_parser.ParseConfig("zrpEstimation", "m_cfg_d_convergence_dheight_m", m_cfg_d_convergence_dheight_m) != true)	return false;
    if(m_ini_parser.ParseConfig("zrpEstimation", "m_cfg_b_plane_rmse_treshold", m_cfg_b_plane_rmse_treshold) != true)	return false;

    // zrp calculation
    if(m_ini_parser.ParseConfig("zrpEstimation", "m_cfg_n_moving_average_window_size", m_cfg_n_moving_average_window_size) != true)	return false;

    return true;
}


/*
Brief
- Extract the sensor height, roll, and pitch information from the ground point cloud.
Input
- i_pc: Input point cloud, expecially please use ground segmented point cloud
Output
- height_m: (it is Z) Sensor's heightin meter
- roll_rad: Sensor's roll in *Radian* (rad)
- pitch_rad: Sensor's pitch in *Radian* (rad)
Return
- true/false, false has some error like input point cloud is NONE
*/
template<typename pointType>
bool CZRPEstimation::run( const pcl::PointCloud<pointType>& i_pc)
{
    MeasureComputingTime measure_time; // For time debugging

    // Copy input point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr i_pcptr(new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(i_pc, *i_pcptr);

    m_b_run_success_flag = false; // Flag for check run success

    // Initialize check
    if(m_b_initialize_flag == false){
        DebugPrintError("[zrpEstimation] initialize is not done",m_cfg_b_debug_flag);
        return false;
    }

    // Extract point cloud in roi
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vec_pcptr_roi;
    if(PCRoiExtractor(i_pcptr, vec_pcptr_roi) == false){
        return false;
    }
    DebugPrintInfoTime("[zrpEstimation] run - PCRoiExtractor Done", measure_time, m_cfg_b_debug_flag);

    tmp_debug_pcptr_roi_pc = vec_pcptr_roi;

    // Fitting plane model
    std::vector<Eigen::Vector4f> vec_plane_model;
    if(PlaneDetection(vec_pcptr_roi, vec_plane_model) == false){
        return false;
    }
    DebugPrintInfoTime("[zrpEstimation] run - PlaneDetection Done", measure_time, m_cfg_b_debug_flag);

    m_vec_point_to_plane_distance_rmse;
    tmp_debug_vec_plane_model = vec_plane_model;

    // Calcualte height, roll, pitch
    if(zrpCalculation(vec_plane_model) == false){
        return false;
    }
    DebugPrintInfoTime("[zrpEstimation] run - zrpCalculation Done", measure_time, m_cfg_b_debug_flag);

    m_b_run_success_flag = true;
    return true;
}

/*
Brief
- Get height, roll, pitch from memeber variables
Input
Output
- height_m: (it is Z) Sensor's heightin meter
- roll_rad: Sensor's roll in *Radian* (rad)
- pitch_rad: Sensor's pitch in *Radian* (rad)
Return
- true/false, false has some error like process run fail
*/
bool CZRPEstimation::GetZRP(std::vector<double>& o_vec_zrp_m_rad)
{
    if(m_b_run_success_flag == false)
        return false;

    o_vec_zrp_m_rad = m_vec_zrp_m_rad;
    return true;
}

/*
Brief
- Get point to plane distance rmse
Input
Output
- o_vec_point_to_plane_distance_rmse: rmse for each roi
Return
- true/false, false has some error like process run fail
*/
bool CZRPEstimation::GetRMSE(std::vector<double>& o_vec_point_to_plane_distance_rmse)
{
    if(m_b_run_success_flag == false)
        return false;

    o_vec_point_to_plane_distance_rmse = m_vec_point_to_plane_distance_rmse;
    return true;
}

/*
Brief
- Extract points in ROI
Input
- i_pc: Input point cloud, expecially please use ground segmented point cloud
Output
- o_vec_pc_roi : vector of point cloud in roi
- o_vec_seed_points : vector of seed points for patch work
Configuration
-
Return
- true/false, false has some error like no points in roi
*/

bool CZRPEstimation::PCRoiExtractor(const pcl::PointCloud<pcl::PointXYZ>::Ptr& i_pc,
                                    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& o_vec_pc_roi)
{
    // Iterate on all ROI
    for(int i = 0; i < m_cfg_vec_ROI.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc_roi(new pcl::PointCloud<pcl::PointXYZ>);
        copyPointCloud(*i_pc, *pc_roi);

        // ROI filtering
        pcl::PassThrough<pcl::PointXYZ> passthrouh_pc_roi_filter;
        passthrouh_pc_roi_filter.setInputCloud(pc_roi);
        passthrouh_pc_roi_filter.setFilterFieldName("x");
        passthrouh_pc_roi_filter.setFilterLimits( m_cfg_vec_ROI[i][0], m_cfg_vec_ROI[i][1] ); // min, max
        passthrouh_pc_roi_filter.filter(*pc_roi);

        passthrouh_pc_roi_filter.setFilterFieldName("y");
        passthrouh_pc_roi_filter.setFilterLimits( m_cfg_vec_ROI[i][2], m_cfg_vec_ROI[i][3] ); // min, max
        passthrouh_pc_roi_filter.filter(*pc_roi);

        passthrouh_pc_roi_filter.setFilterFieldName("z");
        passthrouh_pc_roi_filter.setFilterLimits( m_cfg_vec_ROI[i][4], m_cfg_vec_ROI[i][5] ); // min, max
        passthrouh_pc_roi_filter.filter(*pc_roi);

        // If there are no points in roi return false
        if(pc_roi->points.size() <= m_cfg_n_plane_point_treshold)
        {
            DebugPrintError("[zrpEstimation] The number of points inside the ROI is less than the cfg num of palne point threshold",m_cfg_b_debug_flag);
            return false;
        }
        // Insert roi point cloud to vector
        o_vec_pc_roi.push_back(pc_roi);
    }
    return true;
}

/*
Brief
- Plane fitting using patchwork method. Plane is fitted considering outliers.
Input
- i_vec_pc_roi : vector of point cloud in roi
Output
- o_vec_plane_model : vector of fitted plane model
Return
- true/false, false has some error like fewer sampling points
*/
bool CZRPEstimation::PlaneDetection(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& i_vec_pc_roi,
                                    std::vector<Eigen::Vector4f>& o_vec_plane_model)
{
    // Variables initialize
    tmp_debug_pcptr_sampling_points->clear();
    pcl::PointCloud<pcl::PointXYZ>::Ptr seed_points(new pcl::PointCloud<pcl::PointXYZ>);
    double d_sum_seed_point_height_m, d_ave_seed_pont_height_m;
    pcl::PointCloud<pcl::PointXYZ>::Ptr sampling_points(new pcl::PointCloud<pcl::PointXYZ>);
    m_vec_point_to_plane_distance_rmse.clear();
    tmp_debug_vec_point_to_plane_distance_rmse.clear();

    // Iterate on all ROI point cloud in vector
    for(auto pc_roi : i_vec_pc_roi)
    {
        // Check number of points in roi
        if(pc_roi->points.size() < m_cfg_n_seed_points)
        {
            DebugPrintError("[zrpEstimation] The number of point clouds is less than m_cfg_i_num_of_seed_points.",m_cfg_b_debug_flag);
            return false;
        }
        // 1. Extract seed point and calculate average height of seed_points
        seed_points->clear();
        sampling_points->clear();
        d_sum_seed_point_height_m = 0;

        // - Copy input point cloud and sort
        copyPointCloud(*pc_roi, *seed_points);
        std::sort(seed_points->points.begin(), seed_points->points.end(),
                            [](const pcl::PointXYZ& a, const pcl::PointXYZ& b){ return a.z < b.z; });

        // - Erase points so that the number of seed points becomes m_cfg_n_seed_points
        seed_points->points.erase(seed_points->points.begin() + m_cfg_n_seed_points, seed_points->points.end());

        // - Calculate the sum of seed points z values
        for(auto point_it = seed_points->points.begin(); point_it != seed_points->points.end(); ++point_it)
        {
            d_sum_seed_point_height_m += point_it->z;
        }

        // - Calculate average seed points z value
        d_ave_seed_pont_height_m = d_sum_seed_point_height_m / m_cfg_n_seed_points;

        // 2. Initial plane fitting
        // - Get sample points that are lower height than the average height of the seed points + margin 
        pcl::PassThrough<pcl::PointXYZ> passthrouh_pc_roi_filter;
        passthrouh_pc_roi_filter.setInputCloud(pc_roi);
        passthrouh_pc_roi_filter.setFilterFieldName("z");
        double min_z = d_ave_seed_pont_height_m - m_cfg_d_init_margin_of_patchwork;
        double max_z = d_ave_seed_pont_height_m + m_cfg_d_init_margin_of_patchwork;
        passthrouh_pc_roi_filter.setFilterLimits( min_z, max_z ); // min, max
        passthrouh_pc_roi_filter.filter(*sampling_points);

        // - Fitting Initial plane
        Eigen::Vector4f plane_coeff;
        PlaneFitting(sampling_points, plane_coeff);

        double plane_rmse = 0;
        Eigen::Vector4f prev_plane_coeff;
        double d_inner_product, d_plane_norm, d_prev_plane_norm, d_acos, d_included_angle_deg, d_plane_height_m, d_prev_plane_height_m, d_heigt_diff_m;
        for (int iteration = 0; iteration < m_cfg_n_plane_fitting_iteration; iteration++)
        {
            // 3. Point sampling within plane area
            sampling_points->clear();
            PlanePointSampling(pc_roi, plane_coeff, sampling_points, plane_rmse);

            // 4. Fitting plane
            prev_plane_coeff = plane_coeff;
            PlaneFitting(sampling_points, plane_coeff);

            // - Check plane angle change
            d_inner_product = plane_coeff(0)*prev_plane_coeff(0)+plane_coeff(1)*prev_plane_coeff(1)+plane_coeff(2)*prev_plane_coeff(2);
            d_plane_norm = sqrt(plane_coeff(0)*plane_coeff(0)+plane_coeff(1)*plane_coeff(1)+plane_coeff(2)*plane_coeff(2));
            d_prev_plane_norm = sqrt(prev_plane_coeff(0)*prev_plane_coeff(0)+prev_plane_coeff(1)*prev_plane_coeff(1)+prev_plane_coeff(2)*prev_plane_coeff(2));
            d_acos = d_inner_product/(d_plane_norm*d_prev_plane_norm);
            if(d_acos >= 1.)
                d_acos = 1.;
            d_included_angle_deg = acos(d_acos) / M_PI * 180.0;

            // - Check plane height change
            d_plane_height_m = abs(plane_coeff(3)) / sqrt(plane_coeff(0)*plane_coeff(0) + plane_coeff(1)*plane_coeff(1) + plane_coeff(2)*plane_coeff(2));
            d_prev_plane_height_m = abs(prev_plane_coeff(3)) / sqrt(prev_plane_coeff(0)*prev_plane_coeff(0) + prev_plane_coeff(1)*prev_plane_coeff(1) + prev_plane_coeff(2)*prev_plane_coeff(2));
            d_heigt_diff_m = abs(d_prev_plane_height_m - d_plane_height_m);

            if(d_included_angle_deg < m_cfg_d_convergence_dtheta_deg && d_heigt_diff_m < m_cfg_d_convergence_dheight_m)
            {
                DebugPrintInfo("[zrpEstimation] Patchwork iteration break because plane change is small. iter : ", iteration+1, m_cfg_b_debug_flag);
                break;
            }
        }
        m_vec_point_to_plane_distance_rmse.push_back(plane_rmse);
        tmp_debug_vec_point_to_plane_distance_rmse.push_back(plane_rmse);

        if(sampling_points->points.size() < m_cfg_n_plane_point_treshold)
        {
            DebugPrintError("[zrpEstimation] The number of sampling plane point is less than m_cfg_n_plane_point_treshold",m_cfg_b_debug_flag);
            return false;
        }
        if(plane_rmse > m_cfg_b_plane_rmse_treshold)
        {
            DebugPrintError("[zrpEstimation] Plane RMSE is over than m_cfg_b_plane_rmse_treshold. rmse : ",plane_rmse, m_cfg_b_debug_flag);
            return false;
        }

        o_vec_plane_model.push_back(plane_coeff);

        // sampling_point_save_for_visualization
        *tmp_debug_pcptr_sampling_points += *sampling_points;
    }
    return true;
}

/*
Brief
- Flatness determination and zrp estimation
Input
- i_vec_plane_model : vector of fitted plane model
Output
- None
Return
- true/false, false has some error like no points in roi
*/
bool CZRPEstimation::zrpCalculation(const std::vector<Eigen::Vector4f>& i_vec_plane_model)
{
    // Calculate zrp
    double d_roi_height_sum_m = 0;
    Eigen::Vector4f roi_plane_coeff_sum;
    roi_plane_coeff_sum = roi_plane_coeff_sum*0.0;
    for(auto plane_coeff : i_vec_plane_model)
    {
        // - Height sum
        d_roi_height_sum_m += abs(plane_coeff(3)) / sqrt(plane_coeff(0)*plane_coeff(0) + plane_coeff(1)*plane_coeff(1) + plane_coeff(2)*plane_coeff(2));
        // - Normal vector sum
        roi_plane_coeff_sum += plane_coeff;
    }

    // - Average height
    double d_height_m = (d_roi_height_sum_m)/i_vec_plane_model.size();

    // - Normalize normal vector
    double norm = sqrt(roi_plane_coeff_sum(0)*roi_plane_coeff_sum(0) + roi_plane_coeff_sum(1)*roi_plane_coeff_sum(1) + roi_plane_coeff_sum(2)*roi_plane_coeff_sum(2));
    roi_plane_coeff_sum = roi_plane_coeff_sum / norm;

    // - Calculate roll pitch
    double d_pitch_rad = -atan2(roi_plane_coeff_sum(0), roi_plane_coeff_sum(2));
    Eigen::Vector3d rotated_vector;
    rotated_vector(0) = roi_plane_coeff_sum(0);
    rotated_vector(1) = roi_plane_coeff_sum(1);
    rotated_vector(2) = roi_plane_coeff_sum(2);
    rotated_vector = Eigen::AngleAxisd(d_pitch_rad, Eigen::Vector3d::UnitY()) * rotated_vector;

    double d_roll_rad = atan2(rotated_vector(1), rotated_vector(2));

    // Apply moving average window
    static std::deque<double> vec_window_height_m, vec_window_roll_x, vec_window_roll_y, vec_window_pitch_x, vec_window_pitch_y;
    static double d_window_height_sum, d_window_roll_x_sum, d_window_roll_y_sum, d_window_pitch_x_sum, d_window_pitch_y_sum;

    // - Sum values in window
    d_window_height_sum += d_height_m;
    vec_window_height_m.push_back(d_height_m);
    d_window_roll_x_sum += cos(d_roll_rad);
    vec_window_roll_x.push_back(cos(d_roll_rad));
    d_window_roll_y_sum += sin(d_roll_rad);
    vec_window_roll_y.push_back(sin(d_roll_rad));
    d_window_pitch_x_sum += cos(d_pitch_rad);
    vec_window_pitch_x.push_back(cos(d_pitch_rad));
    d_window_pitch_y_sum += sin(d_pitch_rad);
    vec_window_pitch_y.push_back(sin(d_pitch_rad));

    // - Subtract values leaving window
    if(vec_window_height_m.size() > m_cfg_n_moving_average_window_size)
    {
        d_window_height_sum -= vec_window_height_m.front();
        vec_window_height_m.pop_front();
        d_window_roll_x_sum -= vec_window_roll_x.front();
        vec_window_roll_x.pop_front();
        d_window_roll_y_sum -= vec_window_roll_y.front();
        vec_window_roll_y.pop_front();
        d_window_pitch_x_sum -= vec_window_pitch_x.front();
        vec_window_pitch_x.pop_front();
        d_window_pitch_y_sum -= vec_window_pitch_y.front();
        vec_window_pitch_y.pop_front();
    }

    // - Compute average value
    double d_window_roll_x_average, d_window_roll_y_average, d_window_pitch_x_average, d_window_pitch_y_average;
    int i_current_window_size = vec_window_height_m.size();
    d_window_roll_x_average = d_window_roll_x_sum / ((double) i_current_window_size);
    d_window_roll_y_average = d_window_roll_y_sum / ((double) i_current_window_size);
    d_window_pitch_x_average = d_window_pitch_x_sum / ((double) i_current_window_size);
    d_window_pitch_y_average = d_window_pitch_y_sum / ((double) i_current_window_size);

    double d_window_height_average = d_window_height_sum / ((double) i_current_window_size);
    double d_window_roll_average = atan2(d_window_roll_y_average, d_window_roll_x_average);
    double d_window_pitch_average = atan2(d_window_pitch_y_average, d_window_pitch_x_average);

    // - Insert to zrp member variable
    m_vec_zrp_m_rad.clear();
    m_vec_zrp_m_rad.push_back(d_window_height_average);
    m_vec_zrp_m_rad.push_back(d_window_roll_average);
    m_vec_zrp_m_rad.push_back(d_window_pitch_average);

    return true;
}

/*
Brief
- Plane fitting by solving least square problem
Input
- i_plane_pcptr : point cloud to compute plane
Output
- o_plane_coeff : computed plane model

Return
- none
*/
void CZRPEstimation::PlaneFitting(const pcl::PointCloud<pcl::PointXYZ>::Ptr& i_plane_pc,
                                    Eigen::Vector4f& o_plane_coeff)
{

    // make points matrix
    Eigen::MatrixXf points_matrix = i_plane_pc->getMatrixXfMap(3,4,0).transpose();

    // make normal equation : ax + by + d = z -> Ax = b
    // - A : [[X_0 Y_0 1]; [X_1 Y_1 1]; ... [X_n Y_n 1]]
    // - x : [a, b, d]'
    // - b : [Z_0 Z_1 ... Z_n]'
    Eigen::MatrixXf XY1 = points_matrix.leftCols(2);
    XY1.conservativeResize(XY1.rows(), XY1.cols() + 1);
    XY1.col(2) = Eigen::VectorXf::Ones(XY1.rows());

    Eigen::VectorXf Z = points_matrix.rightCols(1);

    // Fitting plane by solve normal equation
    Eigen::Vector3f ABD = (XY1.transpose() * XY1).ldlt().solve(XY1.transpose() * Z);
    double magnitude = sqrt(ABD(0)*ABD(0) + ABD(1)*ABD(1) + 1);

    // Make plane coefficient with normal vector
    Eigen::Vector4f ABCD;
    ABCD << -ABD(0), -ABD(1), 1, -ABD(2);
    ABCD = ABCD / magnitude;

    if(ABCD(2) < 0)
    {
        ABCD = -ABCD;
    }

    o_plane_coeff = ABCD;
}

/*
Brief
- Point used to plane fitting
Input
- i_plane_pcptr : point cloud to compute plane
- i_plane_coeff : plane coefficient to compute point to plane distance
Output
- o_plane_coeff : Point sampling to be used for plane fit base on point to plane distance
- o_RMSE : point to plane distance rmse
Return
- none
*/
void CZRPEstimation::PlanePointSampling(const pcl::PointCloud<pcl::PointXYZ>::Ptr& i_roi_pc,
                                       const  Eigen::Vector4f& i_plane_coeff,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr& o_plane_pc,
                                       double& o_RMSE)
{
    // plane coeff
    double a = i_plane_coeff(0);
    double b = i_plane_coeff(1);
    double c = i_plane_coeff(2);
    double d = i_plane_coeff(3);

    double square_error_sum = 0;
    for(auto point : i_roi_pc->points)
    {
        double point_to_plane_distance = abs(a*point.x + b*point.y + c*point.z+d) / sqrt(a*a + b*b + c*c);
        if(point_to_plane_distance < m_cfg_d_margin_of_patchwork)
        {
            square_error_sum += point_to_plane_distance*point_to_plane_distance;
            o_plane_pc->points.push_back(point);
        }
    }
    o_RMSE = sqrt(square_error_sum / o_plane_pc->points.size());
}