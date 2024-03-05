/**
 * @copyright (c) AI LAB - Konkuk Uni.
 * <br>All rights reserved. Subject to limited distribution and restricted disclosure only.
 * @author pauljiwon96@gmail.com dudfhr3349@gmail.com
 * @file line_based_segmentation.cpp
 * @brief line based segmentation module source file
 * @version 1.0
 * @date 06-10-2022
 * @bug No known bugs
 * @warning No warnings
 */
#include "segmentation/line_based_segmentation.hpp"

bool CLineBasedSegmentation::Init(std::string ini_path)
{
    std::cout << "ini_path "<<ini_path<<std::endl;

    if(m_ini_handler.Init(ini_path.c_str()) == false)
    {
        return false;
    }
    
    if(ParsingIni() == false)
        return false;

    m_str_ini_path = ini_path;

    m_p_range_image.reset( new CRangeImage(m_str_range_image_ini_path));

    m_pcptr_input.reset( new pcl::PointCloud<PointXYZIRT> );
    m_pcptr_rph_calibrated.reset( new pcl::PointCloud<PointXYZIRT> );
    m_pcptr_output.reset( new pcl::PointCloud<PointXYZIRT> );

    m_p_line_fit->Init(ini_path);
    m_p_label_seg->Init(ini_path);
    m_p_vertical_seg->Init(ini_path);
    m_p_pole_seg->Init(ini_path);

    return true;
}

/*
Brief
- Reset all member functions
Input
- None
Output
- None
Return
- None
*/
void CLineBasedSegmentation::Reset()
{
    m_p_range_image.reset( new CRangeImage(m_str_range_image_ini_path));

    m_pcptr_input.reset( new pcl::PointCloud<PointXYZIRT> );
    m_pcptr_rph_calibrated.reset( new pcl::PointCloud<PointXYZIRT> );
    m_pcptr_output.reset( new pcl::PointCloud<PointXYZIRT> );

    m_vec_ground_idx.clear();
    m_vec_non_ground_idx.clear();
    m_vec_objects_idx.clear();
    m_vec_poles_idx.clear();
}

/*
Brief
- Set point cloud and generate range image for segmentation
Input
- i_point_cloud: input point cloud
Output
- None
Return
- bool: true- normaly operate, false- input point cloud is null or empty or make range image return false
*/
bool CLineBasedSegmentation::SetRangeImage(std::shared_ptr<CRangeImage>& i_p_range_image)
{
    if(i_p_range_image == NULL)
        return false;

    ParsingIni();
    Reset();

    m_p_range_image = i_p_range_image;
    m_pcptr_input = i_p_range_image->GetCloud();

    if (fabs(m_f_lidar_z_m) < FLT_MIN && fabs(m_f_lidar_roll_deg) < FLT_MIN && fabs(m_f_lidar_pitch_deg) < FLT_MIN){
        m_pcptr_rph_calibrated = m_pcptr_input; 
    }
    else{
        Eigen::Affine3f rph_tf = pcl::getTransformation(0., 0., m_f_lidar_z_m, 
                                        m_f_lidar_roll_deg * M_PI / 180., m_f_lidar_pitch_deg * M_PI / 180., 0.);
        pcl::transformPointCloud(*m_pcptr_input, *m_pcptr_rph_calibrated, rph_tf.matrix());
    }

    return true;
}
std::shared_ptr<CRangeImage> CLineBasedSegmentation::GetRangeImage()
{
    if(m_p_range_image == NULL)
        throw "member RangeImage class is not initialized";
    return m_p_range_image;
}

template <typename pointType>
bool CLineBasedSegmentation::SetCloud(boost::shared_ptr<pcl::PointCloud<pointType>> i_pcptr)
{
    if(i_pcptr == NULL)
        return false;

    Reset();

    if(m_p_range_image == NULL)
        m_p_range_image.reset(new CRangeImage(m_str_range_image_ini_path));

    boost::shared_ptr<pcl::PointCloud<PointXYZIRT>> input_pcptr_xyzirt(new pcl::PointCloud<PointXYZIRT>);
    pcl::copyPointCloud(i_pcptr, input_pcptr_xyzirt);
    m_p_range_image->SetCloud(input_pcptr_xyzirt);

    // Generate rph calibrated input pointcloud ptr
    if (fabs(m_f_lidar_z_m) < FLT_MIN && fabs(m_f_lidar_roll_deg) < FLT_MIN && fabs(m_f_lidar_pitch_deg) < FLT_MIN){
        m_pcptr_rph_calibrated = input_pcptr_xyzirt; 
    }
    else{
        Eigen::Affine3f rph_tf = pcl::getTransformation(0., 0., m_f_lidar_z_m,
                                        m_f_lidar_roll_deg * M_PI / 180., m_f_lidar_pitch_deg * M_PI / 180., 0.);
        pcl::transformPointCloud(*input_pcptr_xyzirt, *m_pcptr_rph_calibrated, rph_tf.matrix());
    }

}

boost::shared_ptr<pcl::PointCloud<PointXYZIRT>> CLineBasedSegmentation::GetCloud()
{
    if(m_p_range_image == NULL)
        throw "member RangeImage class is not initialized";
    return m_p_range_image->GetCloud();
}

boost::shared_ptr<pcl::PointCloud<PointXYZIRT>> CLineBasedSegmentation::GetRPHCalibCloud()
{
    if(m_p_range_image == NULL)
        throw "member RangeImage class is not initialized";

    return m_pcptr_rph_calibrated;
}

bool CLineBasedSegmentation::FilterGrounds()
{
    if(m_p_range_image == NULL)
        return false;

    if(m_enum_ground_segmentation_method == LINEFIT)
    {
        if(m_p_line_fit->FilterGround(*m_p_range_image, m_pcptr_rph_calibrated) == false)
            return false;
    }

    return true;
}

bool CLineBasedSegmentation::FilterObjects()
{
    if(m_p_range_image == NULL)
        return false;

    if(m_enum_object_segmentation_method == LABELSEG)
    {
        if(m_p_label_seg->FilterObject(*m_p_range_image, m_pcptr_rph_calibrated) == false)
            return false;
    }

    return true;
}

bool CLineBasedSegmentation::FilterVertical()
{
    if(m_p_range_image == NULL)
        return false;

    if(m_enum_object_segmentation_method == LABELSEG)
    {
        if(m_p_vertical_seg->FilterObject(*m_p_range_image, m_pcptr_rph_calibrated) == false)
            return false;
    }

    return true;
}

bool CLineBasedSegmentation::FilterPoles()
{
    if(m_p_range_image == NULL)
        return false;


    if(m_p_pole_seg->FilterPole(*m_p_range_image, m_pcptr_rph_calibrated) == false)
        return false;


    return true;
}

/*
Brief
- Get ground point's idx at input point cloud, if not fouded then find ground indexes
Input
- None
Output
- vec_grounds_idx: ground's idx in setted point cloud
Return
- True/False: False mean no ground extracted
*/
bool CLineBasedSegmentation::GetGroundIdx(std::vector<int> &vec_grounds_idx)
{
    if(m_p_range_image->IsMask("ground") == false)
        return false;

    if(m_p_range_image->GetMaskIdx("ground", vec_grounds_idx) == false)
        return false;
    return true;
}

/*
Brief
- Get non ground point's idx at input point cloud, if not fouded then find non ground indexes
Input
- None
Output
- vec_grounds_idx: ground's idx in setted point cloud
Return
- True/False: False mean no ground extracted
*/
bool CLineBasedSegmentation::GetNonGroundIdx(std::vector<int> &vec_non_ground_idx)
{
    if(m_p_range_image->IsMask("ground") == false)
        return false;

    if(m_p_range_image->GetNonMaskIdx("ground", vec_non_ground_idx) == false)
        return false;

    return true;
}

/*
Brief
- Get objects point's idx at input point cloud, if not fouded then find object idxes from range image
Input
- None
Output
- vec_objects_idx: object's idx in setted point cloud
Return
- True/False: False mean no object extracted
*/
bool CLineBasedSegmentation::GetObjectsIdx(std::vector<std::vector<int>> &vec_objects_idx)
{
    if(m_p_range_image->IsMask("object") == false)
        return false;

    if(m_p_range_image->GetMaskIndices("object", vec_objects_idx) == false)
        return false;

    return true;
}

/*
Brief
- Get poles point's idx at input point cloud, if not fouded then find pole idxes from range image
Input
- None
Output
- vec_poles_idx: pole's idx in setted point cloud
Return
- True/False: False mean no pole extracted
*/
bool CLineBasedSegmentation::GetPolesIdx(std::vector<std::vector<int>> &vec_poles_idx)
{
    if(m_p_range_image->IsMask("pole") == false)
        return false;

    if(m_p_range_image->GetMaskIndices("pole", vec_poles_idx) == false)
        return false;

    return true;
}

/*
Brief
- Get ground point's point cloud
Input
- None
Output
- o_pcptr_ground: ground's idx in setted point cloud
Return
- True/False: False mean no ground extracted
*/
bool CLineBasedSegmentation::GetGroundPC(pcl::PointCloud<PointXYZIRT>::Ptr o_pcptr_ground)
{
    if(o_pcptr_ground==nullptr)
        o_pcptr_ground.reset(new pcl::PointCloud<PointXYZIRT>);
    std::vector<int> i_vec_ground_idx;
    if(GetGroundIdx(i_vec_ground_idx) == false)
        return false;

    pcl::copyPointCloud(*m_pcptr_input,i_vec_ground_idx, *o_pcptr_ground);

    return true;
}

/*
Brief
- Get non ground point's point cloud
Input
- None
Output
- o_pcptr_non_ground: ground's idx in setted point cloud
Return
- True/False: False mean no ground extracted
*/
bool CLineBasedSegmentation::GetNonGroundPC(pcl::PointCloud<PointXYZIRT>::Ptr o_pcptr_non_ground)
{
    if(o_pcptr_non_ground==nullptr)
        o_pcptr_non_ground.reset(new pcl::PointCloud<PointXYZIRT>);
    std::vector<int> i_vec_non_ground_idx;
    if(GetNonGroundIdx(i_vec_non_ground_idx) == false)
        return false;

    pcl::copyPointCloud(*m_pcptr_input,i_vec_non_ground_idx, *o_pcptr_non_ground);

    return true;
}

/*
Brief
- Get objects point's point cloud
Input
- None
Output
- o_pcptr_object: object's idx in setted point cloud
Return
- True/False: False mean no object extracted
*/
bool CLineBasedSegmentation::GetObjectsPC(std::vector<pcl::PointCloud<PointXYZIRT>::Ptr>  &o_vec_pcptr_objects)
{
    std::vector<std::vector<int>> i_vec_objects_idx;
    if(GetObjectsIdx(i_vec_objects_idx) == false)
        return false;
    for(auto objects_iter = i_vec_objects_idx.begin(); objects_iter != i_vec_objects_idx.end(); objects_iter++)
    {
        pcl::PointCloud<PointXYZIRT>::Ptr tmp_pcptr_object(new pcl::PointCloud<PointXYZIRT>);
        pcl::copyPointCloud(*m_pcptr_input,*objects_iter,*tmp_pcptr_object);
        o_vec_pcptr_objects.push_back(tmp_pcptr_object);
    }

    return true;
}

/*
Brief
- Get poles point's point cloud
Input
- None
Output
- o_pcptr_pole: pole's idx in setted point cloud
Return
- True/False: False mean no pole extracted
*/
bool CLineBasedSegmentation::GetPolesPC(std::vector<pcl::PointCloud<PointXYZIRT>::Ptr>  &o_vec_pcptr_poles)
{
    std::vector<std::vector<int>> i_vec_poles_idx;
    if(GetPolesIdx(i_vec_poles_idx) == false)
        return false;
    for(auto poles_iter = i_vec_poles_idx.begin(); poles_iter != i_vec_poles_idx.end(); poles_iter++)
    {
        pcl::PointCloud<PointXYZIRT>::Ptr tmp_pcptr_pole(new pcl::PointCloud<PointXYZIRT>);
        pcl::copyPointCloud(*m_pcptr_input,*poles_iter,*tmp_pcptr_pole);
        o_vec_pcptr_poles.push_back(tmp_pcptr_pole);
    }

    return true;
}

bool CLineBasedSegmentation::GetPolesBottomPC(std::vector<pcl::PointCloud<PointXYZIRT>::Ptr>  &o_vec_pcptr_poles)
{
    std::vector<std::vector<int>> i_vec_poles_idx;
    if(GetPolesIdx(i_vec_poles_idx) == false)
        return false;

    
    // for(auto poles_iter = i_vec_poles_idx.begin(); poles_iter != i_vec_poles_idx.end(); poles_iter++)
    // {

    for(int vec_idx = 0; vec_idx < i_vec_poles_idx.size(); vec_idx++)
    {
        std::vector<int> iter_vec = i_vec_poles_idx[vec_idx];
        pcl::PointCloud<PointXYZIRT>::Ptr tmp_pcptr_pole(new pcl::PointCloud<PointXYZIRT>);
        // pcl::copyPointCloud(*m_pcptr_input,*poles_iter,*tmp_pcptr_pole);

        double x_sum = 0.0; double y_sum = 0.0;  double time_sum = 0.0;
        

        for(int i = 0; i < iter_vec.size(); i++){
            x_sum += m_pcptr_input->points[iter_vec[i]].x;
            y_sum += m_pcptr_input->points[iter_vec[i]].y;
            time_sum += m_pcptr_input->points[iter_vec[i]].time;
        }

        PointXYZIRT temp_pc;
        temp_pc.x = x_sum / iter_vec.size();
        temp_pc.y = y_sum / iter_vec.size();
        temp_pc.z = 0.0;
        temp_pc.time = time_sum / iter_vec.size();
        tmp_pcptr_pole->points.push_back(temp_pc);

        o_vec_pcptr_poles.push_back(tmp_pcptr_pole);
    }

    return true;
}

bool CLineBasedSegmentation::GetPolesPlanePC(std::vector<pcl::PointCloud<PointXYZIRT>::Ptr>  &o_vec_pcptr_poles ,double* model_coeff)
{
    std::vector<std::vector<int>> i_vec_poles_idx;
    if(GetPolesIdx(i_vec_poles_idx) == false)
        return false;

    double a = model_coeff[0];
    double b = model_coeff[1];
    double c = model_coeff[2];
    double d = model_coeff[3];

    for(int vec_idx = 0; vec_idx < i_vec_poles_idx.size(); vec_idx++)
    {
        std::vector<int> iter_vec = i_vec_poles_idx[vec_idx];
        pcl::PointCloud<PointXYZIRT>::Ptr tmp_pcptr_pole(new pcl::PointCloud<PointXYZIRT>);

        double x_sum = 0.0; double y_sum = 0.0;  double time_sum = 0.0;
        

        for(int i = 0; i < iter_vec.size(); i++){
            x_sum += m_pcptr_input->points[iter_vec[i]].x;
            y_sum += m_pcptr_input->points[iter_vec[i]].y;
            time_sum += m_pcptr_input->points[iter_vec[i]].time;
        }

        PointXYZIRT temp_pc;
        temp_pc.x = x_sum / iter_vec.size();
        temp_pc.y = y_sum / iter_vec.size();
        temp_pc.z =  -(a*temp_pc.x + b*temp_pc.y + d) / c;
        temp_pc.time = time_sum / iter_vec.size();
        tmp_pcptr_pole->points.push_back(temp_pc);

        o_vec_pcptr_poles.push_back(tmp_pcptr_pole);
    }

    return true;
}


/* Tools */
bool CLineBasedSegmentation::ParsingIni()
{
    if ( m_ini_handler.IsFileUpdated() == false)
        return true;

    if ( m_ini_handler.ParseConfig("LineBasedSegmentation", "range_image_ini_path", m_str_range_image_ini_path) == false ){return false;}

    std::string tmpstr;
    if ( m_ini_handler.ParseConfig("LineBasedSegmentation", "ground_segmentation_method", tmpstr) == false ){return false;}
    if      (tmpstr == "GROUND_SEG_OFF")
        m_enum_ground_segmentation_method = GROUND_SEG_OFF;
    else if (tmpstr == "LINEFIT" )
        m_enum_ground_segmentation_method = LINEFIT;
    else if (tmpstr == "IMAGEPATCHWORK" )
        m_enum_ground_segmentation_method = IMAGEPATCHWORK;
    else
        m_enum_ground_segmentation_method = LINEFIT;

    if ( m_ini_handler.ParseConfig("LineBasedSegmentation", "object_segmentation_method", tmpstr) == false ){return false;}
    if      (tmpstr == "OBJECT_SEG_OFF")
        m_enum_object_segmentation_method = OBJECT_SEG_OFF;
    else if (tmpstr == "LABELSEG" )
        m_enum_object_segmentation_method = LABELSEG;
    else
        m_enum_object_segmentation_method = LABELSEG;


    if ( m_ini_handler.ParseConfig("Calibration", "m_f_lidar_x_m", m_f_lidar_x_m) == false ){return false;}
    if ( m_ini_handler.ParseConfig("Calibration", "m_f_lidar_y_m", m_f_lidar_y_m) == false ){return false;}
    if ( m_ini_handler.ParseConfig("Calibration", "m_f_lidar_z_m", m_f_lidar_z_m) == false ){return false;}
    if ( m_ini_handler.ParseConfig("Calibration", "m_f_lidar_roll_deg", m_f_lidar_roll_deg) == false ){return false;}
    if ( m_ini_handler.ParseConfig("Calibration", "m_f_lidar_pitch_deg", m_f_lidar_pitch_deg) == false ){return false;}
    if ( m_ini_handler.ParseConfig("Calibration", "m_f_lidar_yaw_deg", m_f_lidar_yaw_deg) == false ){return false;}


    return true;
}