/**
 * @copyright (c) AI LAB - Konkuk Uni.
 * <br>All rights reserved. Subject to limited distribution and restricted disclosure only.
 * @author pauljiwon96@gmail.com wodud3743@gmail.com
 * @file line_fit.cpp
 * @brief line_fit based ground segmentation source file. Origin article is "M. Himmelsbach, F. v. Hundelshausen and
 * H. . -J. Wuensche, "Fast segmentation of 3D point clouds for ground vehicles,"
 * 2010 IEEE Intelligent Vehicles Symposium, 2010, pp. 560-565, doi: 10.1109/IVS.2010.5548059."
 * @version 1.0
 * @date 06-10-2022
 * @bug No warnings
 * @warning No warnings
 */

#include "segmentation/ground_segmentation/line_fit.hpp"

CLineFit::CLineFit()
{
    // Default setting of parameter
    m_cfg_b_use_ini_config                  = false;
    m_cfg_f_ground_angle_threshold          = 10.0;
    m_cfg_f_ground_height_angle_threshold   = 8.0;

    m_f_lidar_height_m                        = 1.5;

    return;
}
CLineFit::~CLineFit()
{
    return;
}

bool CLineFit::FilterGround(CRangeImage& range_image)
{
    return FilterGround(range_image,range_image.GetCloud());
}

bool CLineFit::FilterGround(CRangeImage& range_image, pcl::PointCloud<PointXYZIRT>::Ptr rph_calibrated_pcptr)
{
    if(m_cfg_b_use_ini_config == true)
        ParsingIni();

    if(range_image.IsInit() == false)
        return false;

    int i_img_height    = range_image.Height();
    int i_img_width     = range_image.Width();

    boost::shared_ptr<pcl::PointCloud<PointXYZIRT>> pcptr_input = rph_calibrated_pcptr;

    // Generate angle images
    range_image.GenerateFloatMask("angle");
    range_image.GenerateFloatMask("smooth_angle");
    range_image.GenerateFloatMask("hor_angle");
    float** f_angle_image      = range_image.GetFloatMaskImg("angle");
    float** f_smooth_angle_img = range_image.GetFloatMaskImg("smooth_angle");
    float** f_hor_angle_img    = range_image.GetFloatMaskImg("hor_angle");

    // Generate ground mask
    range_image.GenerateMask("ground");
    uint8_t** ui8_ground_mask_img = range_image.GetMaskImg("ground");

    // Ground filtering
    float f_angle, f_horizontal_distance, f_vertical_distance;
    int curr_point_idx, next_point_idx;
    int left_point_idx, right_point_idx;
    float f_curr_point_x, f_curr_point_y, f_curr_point_z;
    float f_next_point_x, f_next_point_y, f_next_point_z;
    float f_curr_point_range, f_next_point_range;

    float f_angle_btw_points;

    // Vertical Angle Search

    for(int idx_col = 0 ; idx_col < i_img_width ; idx_col++)
    {
        for(int idx_row = i_img_height-1 ; idx_row > 0 ; idx_row--)
        {
            f_curr_point_range = range_image.GetRange(idx_row, idx_col);
            f_next_point_range = range_image.GetRange(idx_row-1, idx_col);

            curr_point_idx = range_image.GetIdx(idx_row, idx_col);
            next_point_idx = range_image.GetIdx(idx_row-1, idx_col);

            if(curr_point_idx == __INT32_MAX__)
            {
                f_angle_btw_points = 90.;
            }
            // No point(range) in currunt search pixel, get angle by using
            else if(next_point_idx == __INT32_MAX__)
            {
                f_angle_btw_points = FLT_MAX;
                if( idx_row - 2 > 0 )
                {
                    if(range_image.GetIdx(idx_row-2, idx_col) != __INT32_MAX__)
                    {
                        next_point_idx = range_image.GetIdx(idx_row-2, idx_col);

                        f_curr_point_x = pcptr_input->points[curr_point_idx].x;
                        f_curr_point_y = pcptr_input->points[curr_point_idx].y;
                        f_curr_point_z = pcptr_input->points[curr_point_idx].z;

                        f_next_point_x = pcptr_input->points[next_point_idx].x;
                        f_next_point_y = pcptr_input->points[next_point_idx].y;
                        f_next_point_z = pcptr_input->points[next_point_idx].z;

                        f_horizontal_distance = sqrt(pow(f_curr_point_x - f_next_point_x,2) +
                                                pow(f_curr_point_y - f_next_point_y,2));
                        f_vertical_distance = (f_next_point_z - f_curr_point_z);

                        f_angle_btw_points = abs(atan2(f_vertical_distance,f_horizontal_distance)*180.0/M_PI);
                    }
                }
            }
            else if( abs(f_curr_point_range - f_next_point_range) > m_cfg_b_ground_point_max_distance  || 
                         f_curr_point_range - f_next_point_range > 0.0)
            {
                f_angle_btw_points = 90.;
            }
            else
            {
                f_curr_point_x = pcptr_input->points[curr_point_idx].x;
                f_curr_point_y = pcptr_input->points[curr_point_idx].y;
                f_curr_point_z = pcptr_input->points[curr_point_idx].z;

                f_next_point_x = pcptr_input->points[next_point_idx].x;
                f_next_point_y = pcptr_input->points[next_point_idx].y;
                f_next_point_z = pcptr_input->points[next_point_idx].z;

                f_horizontal_distance = sqrt(pow(f_curr_point_x - f_next_point_x,2) +
                                    pow(f_curr_point_y - f_next_point_y,2));
                f_vertical_distance = (f_next_point_z - f_curr_point_z);

                f_angle_btw_points = abs(atan2(f_vertical_distance,f_horizontal_distance)*180.0/M_PI);
            }

            f_angle_image[idx_row-1][idx_col] = f_angle_btw_points;
        }
    }

    // // smoothing Savitchky Goley
    // float A[] = { -2., 3., 6., 7., 6., 3., -2. };
    // int window_size = 3;
    // int idx_row,idx_valid_row;

    // for(int idx_col = 0 ; idx_col < i_img_width ; idx_col++)
    // {
    //     std::vector<double> vec_raw_angle;
    //     std::vector<int> vec_row_idx;

    //     // Ignore FLT_MAX points
    //     for(idx_row = 1 ; idx_row < i_img_height ; idx_row++)
    //     {
    //         if (f_angle_image[idx_row-1][idx_col] < 90.)
    //         {
    //             vec_raw_angle.push_back(f_angle_image[idx_row-1][idx_col]);
    //             vec_row_idx.push_back(idx_row);
    //         }
    //         else
    //         {
    //             f_smooth_angle_img[idx_row-1][idx_col] = f_angle_image[idx_row-1][idx_col];
    //         }
    //     }

    //     int point_number = vec_raw_angle.size();
    //     if(point_number < 5)
    //     {
    //         continue;
    //     }

    //     for(idx_valid_row = 0; idx_valid_row < window_size ; idx_valid_row++)
    //     {
    //         f_smooth_angle_img[vec_row_idx[idx_valid_row]-1][idx_col] = vec_raw_angle[idx_valid_row];
    //     }

    //     for(idx_valid_row = window_size; idx_valid_row < point_number - window_size ; idx_valid_row++)
    //     {
    //         double x = vec_row_idx[idx_valid_row];
    //         int window_idx = 0;
    //         double nominator = 0;
    //         double denominator = 0;
    //         for(window_idx = - window_size ; window_idx <= window_size; window_idx++)
    //         {
    //             nominator += (A[window_size + window_idx] * vec_raw_angle[idx_valid_row]);
    //             denominator += A[window_size + window_idx];
    //         }
    //         double smooth_angle = nominator / denominator;
    //         f_smooth_angle_img[vec_row_idx[idx_valid_row]-1][idx_col] = smooth_angle;

    //     }
    //     for(idx_valid_row = point_number - window_size; idx_valid_row < point_number; idx_valid_row++)
    //     {
    //         f_smooth_angle_img[vec_row_idx[idx_valid_row]-1][idx_col] = vec_raw_angle[idx_valid_row];
    //     }
    // }

    // Index masking
    int i_non_ground_count;
    int i_ground_count;
    float f_distance_from_ground;
    float f_angle_from_ground;
    bool b_point_near_ground;

    for(int idx_col = 0 ; idx_col < i_img_width ; idx_col++)
    {
        i_non_ground_count = 0;
        i_ground_count = 0;
        for(int idx_row = i_img_height-1 ; idx_row > 0 ; idx_row--)
        {   
            float f_angle_btw_points = f_angle_image[idx_row-1][idx_col];
            curr_point_idx = range_image.GetIdx(idx_row, idx_col);
            if(curr_point_idx == __INT32_MAX__)
            {
                f_angle_from_ground = FLT_MAX;
            }
            else
            {
                f_curr_point_x = pcptr_input->points[curr_point_idx].x;
                f_curr_point_y = pcptr_input->points[curr_point_idx].y;
                f_curr_point_z = pcptr_input->points[curr_point_idx].z;

                f_distance_from_ground = f_curr_point_z + m_f_lidar_height_m;
                f_horizontal_distance = sqrt(f_curr_point_x*f_curr_point_x + f_curr_point_y*f_curr_point_y);
                f_angle_from_ground = atan2(f_distance_from_ground,f_horizontal_distance)*180./M_PI;
            }
            
            if (f_angle_from_ground < m_cfg_f_ground_height_angle_threshold)
            {   
                b_point_near_ground = true;
            }
            else
            {
                b_point_near_ground = false;
                if(!m_cfg_b_use_angle_from_ground){b_point_near_ground = true;}
            }

            // Check ground based on angle value
            if ((idx_row == i_img_height-1) && b_point_near_ground)
            {
                i_non_ground_count = 0;
                i_ground_count++;
                ui8_ground_mask_img[idx_row][idx_col] = 1;
            }

            if ((f_angle_btw_points < m_cfg_f_ground_angle_threshold) && b_point_near_ground)
            {
                i_non_ground_count = 0;
                i_ground_count++;
                ui8_ground_mask_img[idx_row-1][idx_col] = 1;
                ui8_ground_mask_img[idx_row][idx_col] = 1;
            }
        }
    }
    return true;
}

bool CLineFit::Init(std::string ini_path)
{
    m_cini_handler.Init(ini_path);
    if( ParsingIni() == false)
        return false;
    return true;
}

bool CLineFit::ParsingIni()
{
    if(m_cini_handler.IsFileUpdated() == false)
        return true;
    if (m_cini_handler.ParseConfig("LineFit" , "m_cfg_b_use_ini_config",                m_cfg_b_use_ini_config ) != true){ return false;}
    if (m_cfg_b_use_ini_config == false)
        return true;

    if (m_cini_handler.ParseConfig("LineFit" , "m_cfg_f_ground_angle_threshold",        m_cfg_f_ground_angle_threshold ) != true){ return false;}
    if (m_cini_handler.ParseConfig("LineFit" , "m_cfg_b_ground_point_max_distance" ,    m_cfg_b_ground_point_max_distance ) != true){ return false;}
    if (m_cini_handler.ParseConfig("LineFit" , "m_cfg_f_ground_height_angle_threshold", m_cfg_f_ground_height_angle_threshold ) != true){ return false;}
    if (m_cini_handler.ParseConfig("LineFit" , "m_f_lidar_height_m" ,                   m_f_lidar_height_m ) != true){ return false;}
    if (m_cini_handler.ParseConfig("LineFit" , "m_cfg_b_use_angle_from_ground" ,        m_cfg_b_use_angle_from_ground ) != true){ return false;}
    return true;
}