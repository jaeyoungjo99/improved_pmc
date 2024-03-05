// /**
//  * @copyright (c) AI LAB - Konkuk Uni.
//  * <br>All rights reserved. Subject to limited distribution and restricted disclosure only.
//  * @author pauljiwon96@gmail.com, wodud3743@gmail.com
//  * @file pole_seg.cpp
//  * @brief Range image based fast and efficent segmentation source file. Origin algorithm is [I. Bogoslavskyi and C. Stachniss,
//  * “Fast Range Image-based Segmentation of Sparse 3D Laser Scans for Online Operation,”
//  * Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems, pp. 163-169, 2016.].
//  * This algorithm has some changes
//  * @version 1.0
//  * @date 06-10-2022
//  * @bug No known bugs
//  * @warning No warnings
//  */

#include "segmentation/pole_segmentation/pole_seg.hpp"

CPoleSeg::CPoleSeg()
{
    m_cfg_b_use_ini_config  = false;
    m_cfg_b_print_message = false;

    return;
}
CPoleSeg::~CPoleSeg()
{
    return;
}

bool CPoleSeg::Init(std::string ini_path)
{
    if(m_ini_handler.Init(ini_path) == false)
        return false;
    if(ParseIni() == false)
        return false;
    return true;
}

bool CPoleSeg::FilterPole(CRangeImage& range_image){

    return FilterPole(range_image, range_image.GetCloud());
}

bool CPoleSeg::FilterPole(CRangeImage& range_image, pcl::PointCloud<PointXYZIRT>::Ptr rph_calibrated_pcptr){

    if(m_ini_handler.IsFileUpdated())
    {
        ParseIni();
    }
    
    if(range_image.IsMask("object") == false)
        return false;

    Reset();

    std::vector<std::vector<int>> v_objects_indices;
    range_image.GetMaskIndices("object",v_objects_indices);

    if(v_objects_indices.size() == 0)
    {
        DebugPrintWarn("[CPoleSeg] No segment segmented ", m_cfg_b_print_message);
        return false;
    }
    std::vector<boost::shared_ptr<pcl::PointCloud<PointXYZIRT>>> v_pcptr_objects;

    int point_idx;
    for (auto objects_iter = v_objects_indices.begin(); objects_iter != v_objects_indices.end(); objects_iter++)
    {
		pcl::PointCloud<PointXYZIRT>::Ptr tmp_pcptr_object(new pcl::PointCloud<PointXYZIRT>);
		pcl::copyPointCloud(*rph_calibrated_pcptr, *objects_iter, *tmp_pcptr_object);
		v_pcptr_objects.push_back(tmp_pcptr_object);
    
    }

	// Check object is pole
    std::vector<int> v_range_image_idx;
    float cluster_info[] = {0.,0.,0.,0.,0.,0.}; // [x_m, y_m, z_min_m, z_max_m, radius_mean_m , radius_var_m]
	for (int object_idx = 0; object_idx < v_objects_indices.size(); object_idx++)
	{
		if (CheckPole(*v_pcptr_objects[object_idx], cluster_info) == true)
		{   
            std::cout<<"object_idx: "<< object_idx <<" Pole x: "<<cluster_info[0]<<" y: "<<cluster_info[1]<<" length: "<<cluster_info[3] - cluster_info[2]<<std::endl;
            Pixels cur_label_pixels;

            for (int object_point_idx = 0 ; object_point_idx < v_objects_indices[object_idx].size(); object_point_idx++)
            {   
                v_range_image_idx = range_image.GetPixelPointIdx(v_objects_indices[object_idx][object_point_idx]);
                PixelCoord cur_pixel = {v_range_image_idx[0], v_range_image_idx[1]};
                cur_label_pixels.push_back(cur_pixel);
            }
            m_v_pole_pixels.push_back(cur_label_pixels);
		}
	}

    if(m_v_pole_pixels.size() == 0)
    {
        DebugPrintWarn("[CPoleSeg] No Pole filtered ", m_cfg_b_print_message);
        return false;
    }
    std::sort(m_v_pole_pixels.begin(), m_v_pole_pixels.end(), by_size_decent());
    if(m_v_pole_pixels.size() > UINT8_MAX)
    {
        m_v_pole_pixels.erase(m_v_pole_pixels.begin() + UINT8_MAX-1, m_v_pole_pixels.end()); // Erase small size of pole
    }

    // Generate pole image
    uint8_t** pole_image = range_image.GetMaskImg("pole");
    int label = 1;
    for(auto pole_pixels_iter = m_v_pole_pixels.begin(); pole_pixels_iter != m_v_pole_pixels.end(); pole_pixels_iter++)
    {
        for(auto pixel_iter = pole_pixels_iter->begin(); pixel_iter != pole_pixels_iter->end(); pixel_iter++)
            pole_image[pixel_iter->first][pixel_iter->second] = label;

        label++;
    }

    
    DebugPrintInfo("[CPoleSeg] Number of pole: " + std::to_string(m_v_pole_pixels.size()), m_cfg_b_print_message);
    return true;

}

bool CPoleSeg::CheckPole(pcl::PointCloud<PointXYZIRT>& i_object_point_cloud, float o_cluster_info[]){

    float cluster_x_length, cluster_y_length, cluster_z_length, cluster_width;
    std::vector<float> cluster_x,cluster_y,cluster_z;
    float cluster_info[] = {0.,0.,0.,0.,0.,0.};     // x,y,zmin,zmax,radius,radius_var
    float eigen_value[] = {0., 0., 0., 0.};          // e1,e2,circularity
    float diameter_to_height, flattening, linearity;
    bool b_valid_eigen;
    double pole_distance;

    size_t i_point_cloud_size = i_object_point_cloud.points.size();
    for(size_t point_idx = 0 ; point_idx < i_point_cloud_size ; ++point_idx)
    {
        cluster_x.push_back(i_object_point_cloud.points[point_idx].x);
        cluster_y.push_back(i_object_point_cloud.points[point_idx].y);
        cluster_z.push_back(i_object_point_cloud.points[point_idx].z);
    }

    cluster_x_length = *max_element(cluster_x.begin(),cluster_x.end()) - *min_element(cluster_x.begin(),cluster_x.end());  
    cluster_y_length = *max_element(cluster_y.begin(),cluster_y.end()) - *min_element(cluster_y.begin(),cluster_y.end());  
    cluster_z_length = *max_element(cluster_z.begin(),cluster_z.end()) - *min_element(cluster_z.begin(),cluster_z.end());  

    cluster_width = sqrt(pow(cluster_x_length,2) + pow(cluster_y_length,2)); // Max length of a cluster

    double width_to_height_ratio = cluster_z_length / cluster_width;

    // 1. Ratio > 1.0   2. width < 1.0  3. Length > 0.3
    if (cluster_width < m_cfg_f_max_cluster_width  && 
        cluster_z_length > m_cfg_f_min_cluster_length && cluster_z_length < m_cfg_f_max_cluster_length && 
        width_to_height_ratio > m_cfg_f_min_width_to_length)
        {

        fitCircle(cluster_info, eigen_value, cluster_x, cluster_y, cluster_z);
        
        diameter_to_height = cluster_z_length / (cluster_info[4]*2.0);
        flattening = (eigen_value[1] - eigen_value[0])/eigen_value[1];
        linearity  = eigen_value[2]/eigen_value[1];
        pole_distance = sqrt(cluster_info[0]*cluster_info[0] + cluster_info[1]*cluster_info[1]);

        // Filter : 1. radius >= rad_min   2. radius <= rad_max   3. cluster_width < radius  4. eigen width_to_height_ratio < 15.0
        if (diameter_to_height > m_cfg_f_min_diameter_to_height && 
            cluster_info[4] >= m_cfg_f_min_radius &&  cluster_info[4] <= m_cfg_f_max_radius && 
            flattening < m_cfg_f_max_flattening &&
            linearity  > m_cfg_f_min_linearity &&
            cluster_info[2] < m_cfg_f_max_lowest_point && cluster_info[3] > m_cfg_f_min_highest_point && 
            abs(eigen_value[3]*180./M_PI) > m_cfg_f_min_pole_angle_threshold_deg && pole_distance < 100.)
        {       
            o_cluster_info[0] = cluster_info[0];
            o_cluster_info[1] = cluster_info[1];
            o_cluster_info[2] = cluster_info[2];
            o_cluster_info[3] = cluster_info[3];
            o_cluster_info[4] = cluster_info[4];
            o_cluster_info[5] = cluster_info[5];
            return true;       
        }
        return false;
    }
    return false;      
}


void CPoleSeg::fitCircle(float cluster_info[], float eigen_value[], std::vector<float> cluster_x, std::vector<float> cluster_y, std::vector<float> cluster_z){

    int cluster_size = cluster_x.size();
    float x_average, y_average, z_average, x_estimated, y_estimated, z_estimated, u ,v, w, e1, e2, e3;
    float z_min, z_max;
    float radius_sum = 0.; float radius_mean = 0.; float radius_var = 0.;
    float Suv = 0.; float Suu = 0.; float Svv = 0.; float Suuv = 0.; float Suvv = 0.; float Suuu = 0.; float Svvv = 0.;
    
    float cov_xx = 0.; float cov_xy = 0.; float cov_yy = 0.; float cov_xz = 0.; float cov_yz = 0.; float cov_zz = 0.;
    float cov_xxm = 0.; float cov_xym = 0.; float cov_yym = 0.; float cov_xzm = 0.; float cov_yzm = 0.; float cov_zzm = 0.;

    float eigen_v_x, eigen_v_y, eigen_v_z;

    std::vector<std::vector<float>> v_eigen_info;
    
    x_average = (std::accumulate(cluster_x.begin(), cluster_x.end(), 0.0)) / (float)cluster_size; // x mean
    y_average = (std::accumulate(cluster_y.begin(), cluster_y.end(), 0.0)) / (float)cluster_size; // y mean 
    z_average = (std::accumulate(cluster_z.begin(), cluster_z.end(), 0.0)) / (float)cluster_size; // z mean 

    z_min = *min_element(cluster_z.begin(),cluster_z.end());
    z_max = *max_element(cluster_z.begin(),cluster_z.end());
    
    for (int i = 0 ; i < cluster_size; i++){
        u = cluster_x[i] - x_average;
        v = cluster_y[i] - y_average;
        w = cluster_z[i] - z_average;

        cov_xxm += u*u;
        cov_yym += v*v;
        cov_zzm += w*w;

        cov_xym += u*v;
        cov_xzm += u*w;
        cov_yzm += v*w;

        Suv += u*v;
        Suu += pow(u,2);
        Svv += pow(v,2);
        Suuv += pow(u,2) * v;
        Suvv += u * pow(v,2);
        Suuu += pow(u,3);
        Svvv += pow(v,3); 
    }
    Eigen::Matrix2f m_A;
    Eigen::Vector2f v_B;
    Eigen::Vector2f v_X;
    m_A << Suu,Suv,  Suv,Svv;
    v_B << (Suuu + Suvv)/2.0,(Svvv + Suuv)/2.0;

    v_X = m_A.colPivHouseholderQr().solve(v_B); // Ax = B

    x_estimated = x_average + v_X[0];
    y_estimated = y_average + v_X[1];
    z_estimated = z_average;
    
    // Get Radius mean
    for (int j = 0 ; j < cluster_size ; j++){
        radius_sum += sqrt(pow(cluster_x[j] - x_estimated,2) + pow(cluster_y[j] - y_estimated,2));
    }
    radius_mean = radius_sum/ (float)cluster_size;

    // Get Radius Variance 
    for (int k = 0; k < cluster_size ; k ++){
        float distance = sqrt(pow(cluster_x[k] - x_estimated,2) + pow(cluster_y[k] - y_estimated,2));
        radius_var += pow(radius_mean - distance,2);
    }
    radius_var = radius_var / cluster_size;

    
    for (int i = 0 ; i < cluster_size; i++){
        u = cluster_x[i] - x_estimated;
        v = cluster_y[i] - y_estimated;
        w = cluster_z[i] - z_estimated;

        cov_xx += u*u;
        cov_yy += v*v;
        cov_zz += w*w;

        cov_xy += u*v;
        cov_xz += u*w;
        cov_yz += v*w;
        
    }

    // Eigen value based on estimated center
    cov_xx = cov_xx / (float)cluster_size;
    cov_yy = cov_yy / (float)cluster_size;
    cov_zz = cov_zz / (float)cluster_size;

    cov_xy = cov_xy / (float)cluster_size;
    cov_xz = cov_xz / (float)cluster_size;
    cov_yz = cov_yz / (float)cluster_size;
    

    Eigen::Matrix3f m_C;
    m_C << cov_xx,cov_xy,cov_xz,
           cov_xy,cov_yy,cov_yz,
           cov_xz,cov_yz,cov_zz;

    Eigen::EigenSolver<Eigen::Matrix3f> eigen_solver(m_C);

    v_eigen_info.clear();
    for (int i = 0 ; i < 3 ; i++){
        std::vector<float> v_eigen;
        v_eigen.push_back(eigen_solver.eigenvalues().real()[i]);
        for(int j = 0 ; j < 3 ; j++){
            v_eigen.push_back(eigen_solver.eigenvectors().real()(j,i));
        }
        v_eigen_info.push_back(v_eigen);
    }
    std::sort(v_eigen_info.begin(), v_eigen_info.end(), cmp_vector);

    eigen_v_x = v_eigen_info[2][1];
    eigen_v_y = v_eigen_info[2][2];
    eigen_v_z = v_eigen_info[2][3];

    eigen_value[0] = v_eigen_info[0][0];    // e1
    eigen_value[1] = v_eigen_info[1][0];    // e2
    eigen_value[2] = v_eigen_info[2][0];    // e3
    eigen_value[3] = atan2(eigen_v_z, sqrt(eigen_v_x*eigen_v_x + eigen_v_y*eigen_v_y)); //eigen angle

    cluster_info[0] = x_estimated;
    cluster_info[1] = y_estimated;
    cluster_info[2] = z_min;
    cluster_info[3] = z_max;
    cluster_info[4] = radius_mean;
    cluster_info[5] = radius_var;
}

bool CPoleSeg::cmp_vector(std::vector<float> &v1, std::vector<float> &v2){
    return v1[0] < v2[0];
}

void CPoleSeg::Reset()
{
    m_v_pole_pixels.clear();
    poleCount = 0;
}

bool CPoleSeg::ParseIni()
{
    if (m_ini_handler.ParseConfig("LabelSeg" , "m_cfg_b_use_ini_config",                m_cfg_b_use_ini_config ) != true){ return false;}
    if (m_cfg_b_use_ini_config == false)
        return true;

    if (m_ini_handler.ParseConfig("LabelSeg" , "m_cfg_b_print_message",           m_cfg_b_print_message ) != true){ return false;}

    if (m_ini_handler.ParseConfig("PoleSeg" , "m_cfg_f_max_cluster_width",                  m_cfg_f_max_cluster_width ) != true){ return false;}
    if (m_ini_handler.ParseConfig("PoleSeg" , "m_cfg_f_min_cluster_length",                 m_cfg_f_min_cluster_length ) != true){ return false;}
    if (m_ini_handler.ParseConfig("PoleSeg" , "m_cfg_f_max_cluster_length",                 m_cfg_f_max_cluster_length ) != true){ return false;}
    if (m_ini_handler.ParseConfig("PoleSeg" , "m_cfg_f_min_width_to_length",                m_cfg_f_min_width_to_length ) != true){ return false;}
    if (m_ini_handler.ParseConfig("PoleSeg" , "m_cfg_f_min_diameter_to_height",             m_cfg_f_min_diameter_to_height ) != true){ return false;}
    if (m_ini_handler.ParseConfig("PoleSeg" , "m_cfg_f_min_radius",                         m_cfg_f_min_radius ) != true){ return false;}
    if (m_ini_handler.ParseConfig("PoleSeg" , "m_cfg_f_max_radius",                         m_cfg_f_max_radius ) != true){ return false;}
    if (m_ini_handler.ParseConfig("PoleSeg" , "m_cfg_f_max_flattening",                     m_cfg_f_max_flattening ) != true){ return false;}
    if (m_ini_handler.ParseConfig("PoleSeg" , "m_cfg_f_min_linearity",                      m_cfg_f_min_linearity ) != true){ return false;}
    if (m_ini_handler.ParseConfig("PoleSeg" , "m_cfg_f_min_highest_point",                  m_cfg_f_min_highest_point ) != true){ return false;}
    if (m_ini_handler.ParseConfig("PoleSeg" , "m_cfg_f_max_lowest_point",                   m_cfg_f_max_lowest_point ) != true){ return false;}
    if (m_ini_handler.ParseConfig("PoleSeg" , "m_cfg_f_min_pole_angle_threshold_deg",       m_cfg_f_min_pole_angle_threshold_deg ) != true){ return false;}

    return true;
}
