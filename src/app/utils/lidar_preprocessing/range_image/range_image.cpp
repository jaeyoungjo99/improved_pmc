/**
 * @copyright (c) AI LAB - Konkuk Uni.
 * <br>All rights reserved. Subject to limited distribution and restricted disclosure only.
 * @author pauljiwon96@gmail.com
 * @file range_image.cpp
 * @brief range_image generation module source file
 * @version 1.0
 * @date 06-10-2022
 * @bug No known bugs
 * @warning No warnings
 */

#include "range_image/range_image.hpp"

bool CRangeImage::Init(std::string ini_path)
{
    // Get ini parameter
    try
    {
        m_ini_handler.Init(ini_path);
        if(ParseINIFile() == false)
            return false;
        DebugPrintInfo("[RangeImage] Init - Initialization complete", m_b_print_message);
    }
    catch(std::exception &e)
    {
        std::cout << typeid(e).name() << std::endl;
        std::cerr << e.what() << std::endl;
        std::cout << "[RangeImage] Init - Initialization failed" << std::endl;
        return false;
    }

    // Set ini parameter into class
    m_n_img_size = m_n_height * m_n_width;
    // Set size of m_p_range_img_data array as the height, width parameter.
    AllocateMemory(m_p_range_img_data);
    AllocateMemory(m_p_index_img_data);
    m_i_v_point_to_image_idx = std::vector<std::vector<int>>(2*m_n_img_size,{-1,-1});

    ResetValue(); // Reset all index as zero
    SetSectionBoundary();
    SetSectionStartIndices();

    m_b_is_init = true;
    return true;
}

/*
Brief
- Reset all value with zero
Input
- None
Output
- None
Return
- none
*/
void CRangeImage::ResetValue()
{
    if(m_map_image_mask.empty() == false)
    {
        for(auto iter = m_map_image_mask.begin();iter != m_map_image_mask.end(); iter++)
        {
            ReleaseMemory(iter->second);
        }
        m_map_image_mask.clear();
    }
    if(m_map_float_image_mask.empty() == false)
    {
        for(auto iter = m_map_float_image_mask.begin();iter != m_map_float_image_mask.end(); iter++)
        {
            ReleaseMemory(iter->second);
        }
        m_map_float_image_mask.clear();
    }
    // reset value in array of pointIdR
    for(uint32_t row_idx = 0; row_idx < m_n_height; row_idx++)
    {
        for (uint32_t col_idx = 0; col_idx < m_n_width; col_idx++)
        {
            m_p_range_img_data[row_idx][col_idx] = FLT_MAX;
            m_p_index_img_data[row_idx][col_idx] = __INT32_MAX__;
        }
    }

    // reset point to image idx vector
    size_t vector_size =  m_i_v_point_to_image_idx.size();
    for(int vec_idx = 0; vec_idx < vector_size; vec_idx++)
    {
        // -1 mean not allocated point idx
        m_i_v_point_to_image_idx[vec_idx][0] = -1;
        m_i_v_point_to_image_idx[vec_idx][1] = -1;
    }
}

/*
Brief
- Set Section boundary using parameter
Input
- None
Output
- None
Return
- false: if the section and section resolution of ring and horizontal have wrong configuration.
- ture : Normal operated.
*/
bool CRangeImage::SetSectionBoundary()
{
    if( m_vecf_horizontal_section_angle_deg.size() < 2  || m_vecf_horizontal_section_resolution_deg.empty() == true  )
        return false;
    if(m_vecf_horizontal_section_angle_deg.size() - m_vecf_horizontal_section_resolution_deg.size() != 1 )
        return false;
    // Get horizontal boundary vector
    int i_horizontal_boundary_final_idx = m_vecf_horizontal_section_angle_deg.size() - 1;
    for ( int section_idx = 0; section_idx < m_vecf_horizontal_section_angle_deg.size(); section_idx++)
    {
        double d_section_boundary = 0;
        if( section_idx == 0)
        {
            double d_horizontal_section_half_resolution_angle_current = m_vecf_horizontal_section_resolution_deg[section_idx ] / 2.0;
            d_section_boundary = m_vecf_horizontal_section_angle_deg[section_idx] - d_horizontal_section_half_resolution_angle_current;
            m_vecd_horizontal_section_boundary_deg.push_back( d_section_boundary );
        }
        else if( 0 < section_idx && section_idx < i_horizontal_boundary_final_idx)
        {
            double d_horizontal_section_half_resolution_angle_prev = m_vecf_horizontal_section_resolution_deg[section_idx - 1] / 2.0;
            double d_horizontal_section_half_resolution_angle_current = m_vecf_horizontal_section_resolution_deg[section_idx ] / 2.0;
            // Find smaller one.
            if (d_horizontal_section_half_resolution_angle_prev < d_horizontal_section_half_resolution_angle_current)
                d_section_boundary = m_vecf_horizontal_section_angle_deg[section_idx] - d_horizontal_section_half_resolution_angle_prev;
            else // if (d_horizontal_section_half_resolution_angle_prev > d_horizontal_section_half_resolution_angle_current)
                d_section_boundary = m_vecf_horizontal_section_angle_deg[section_idx] - d_horizontal_section_half_resolution_angle_current;

            m_vecd_horizontal_section_boundary_deg.push_back( d_section_boundary );
        }
        else // if(section_idx == i_horizontal_boundary_final_idx)
        {
            double d_horizontal_section_half_resolution_angle_prev = m_vecf_horizontal_section_resolution_deg[section_idx - 1] / 2.0;
            d_section_boundary = m_vecf_horizontal_section_angle_deg[section_idx] + d_horizontal_section_half_resolution_angle_prev;
            m_vecd_horizontal_section_boundary_deg.push_back( d_section_boundary );
        }
    }

    // Get Vertical boudary vector
    if ( m_b_use_ring_data == false)
    {
        if ( m_vecf_vertical_section_angle_deg.size() < 2 ||  m_vecf_vertical_section_resolution_deg.empty() == true)
            return false;
        if ( m_vecf_vertical_section_angle_deg.size() - m_vecf_vertical_section_resolution_deg.size() != 1 )
            return false;
        int i_vertical_boundary_final_idx = m_vecf_vertical_section_angle_deg.size() - 1;
        for ( int section_idx = 0; section_idx < m_vecf_vertical_section_angle_deg.size(); section_idx++)
        {
            double d_section_boundary = 0;
            if( section_idx == 0)
            {
                double d_vertical_section_half_resolution_angle_current = m_vecf_vertical_section_resolution_deg[section_idx ] / 2.0;
                d_section_boundary = m_vecf_vertical_section_angle_deg[section_idx] - d_vertical_section_half_resolution_angle_current;
                m_vecd_vertical_section_boundary_deg.push_back( d_section_boundary );
            }
            else if( 0 < section_idx && section_idx < i_vertical_boundary_final_idx)
            {
                double d_vertical_section_half_resolution_angle_prev  = m_vecf_vertical_section_resolution_deg[section_idx - 1] / 2.0;
                double d_vertical_section_half_resolution_angle_current = m_vecf_vertical_section_resolution_deg[section_idx ] / 2.0 ;
                // Find smaller one.
                if (d_vertical_section_half_resolution_angle_prev < d_vertical_section_half_resolution_angle_current)
                    d_section_boundary = m_vecf_vertical_section_angle_deg[section_idx] - d_vertical_section_half_resolution_angle_prev;
                else // if (d_vertical_section_half_resolution_angle_prev > d_vertical_section_half_resolution_angle_current)
                    d_section_boundary = m_vecf_vertical_section_angle_deg[section_idx] - d_vertical_section_half_resolution_angle_current;

                m_vecd_vertical_section_boundary_deg.push_back( d_section_boundary );
            }
            else // (section_idx == i_vertical_boundary_final_idx)
            {
                double d_vertical_section_half_resolution_angle_prev = m_vecf_vertical_section_resolution_deg[section_idx - 1] / 2.0;
                d_section_boundary = m_vecf_vertical_section_angle_deg[section_idx] + d_vertical_section_half_resolution_angle_prev;
                m_vecd_vertical_section_boundary_deg.push_back( d_section_boundary );
            }
        }
    }
    return true;
}

/*
Brief
- Set section start index.
Input
- None
Output
- None
Return
- false: if the section and section resolution of ring and horizontal have wrong configuration.
- ture : Normal operated.
*/
bool CRangeImage::SetSectionStartIndices()
{
    if( m_vecf_horizontal_section_angle_deg.size() < 2 || m_vecf_horizontal_section_resolution_deg.empty() == true )
        return false;
    if( m_vecf_horizontal_section_angle_deg.size() - m_vecf_horizontal_section_resolution_deg.size() != 1 )
        return false;

    // Find  additional index for horizontal.
    int horizontal_section_start_index = 0;
    m_veci_horizontal_section_start_index.push_back(horizontal_section_start_index);
    for( int section_idx = 0; section_idx < m_vecf_horizontal_section_angle_deg.size()-2; section_idx++) // Additional angle of first section is push_back before the for-loop.
    {
        float f_section_angle_deg   = m_vecf_horizontal_section_angle_deg[section_idx] - m_vecf_horizontal_section_angle_deg[section_idx + 1];
        float f_section_angle_per_resolution_deg = fabs(f_section_angle_deg / m_vecf_horizontal_section_resolution_deg[section_idx]);
        horizontal_section_start_index = horizontal_section_start_index + (int)round (f_section_angle_per_resolution_deg);
        m_veci_horizontal_section_start_index.push_back(horizontal_section_start_index);
    }

    // Find  additional index for ring.
    if ( m_b_use_ring_data == false)
    {
        if ( m_vecf_vertical_section_angle_deg.size() < 2  || m_vecf_vertical_section_resolution_deg.empty() == true )
            return false;
        if ( m_vecf_vertical_section_angle_deg.size() - m_vecf_vertical_section_resolution_deg.size() != 1  )
            return false;
        int i_vertical_section_start_index = 0.;
        m_veci_vertical_section_start_index.push_back(i_vertical_section_start_index);
        for ( int section_idx = 0; section_idx < m_vecf_vertical_section_angle_deg.size()-2; section_idx++) // Additional angle of first section is push_back before the for-loop.
        {
            float f_section_angle_deg   = m_vecf_vertical_section_angle_deg[section_idx] - m_vecf_vertical_section_angle_deg[section_idx + 1];
            float f_section_angle_per_resolution_deg = fabs(f_section_angle_deg / m_vecf_vertical_section_resolution_deg[section_idx]);
            i_vertical_section_start_index = i_vertical_section_start_index + (int)round (f_section_angle_per_resolution_deg);
            m_veci_vertical_section_start_index.push_back(i_vertical_section_start_index);
        }
    }
    return true;
}

/*
Brief
- Get range image's point cloud. If point clodu is not assiegned that throw error.
Input
- None
Output
- None
Return
- point cloud pointer: range image's point cloud.
*/
boost::shared_ptr<pcl::PointCloud<PointXYZIRT>> CRangeImage::GetCloud()
{
    if(m_pcptr_input_pc == NULL)
        throw "point cloud is not set";
    return m_pcptr_input_pc;
}

/*
Brief
- Generate range image data
Input
- i_pcptr: Input point cloud pointer
Output
- None
Return
- True/False: False mean some error(example input pcptr is null_ptr)
*/
bool CRangeImage::GenerateRangeImg(boost::shared_ptr<pcl::PointCloud<PointXYZIRT>> i_pcptr)
{
    if ( m_b_is_init == false )
        return false;
    // Check the input data
    if ( i_pcptr == nullptr || i_pcptr->size() <= 0)
    {
        DebugPrintError("[RangeImage] GenerateRangeImage - Input pcptr is wrong.", m_b_print_message);
        return false;
    }

    // Reset previous memory
    ResetValue();

    // Find index of points.
    for (uint32_t idx = 0; idx < i_pcptr->size(); idx++)
    {
        if( isnan(i_pcptr->points[idx].x) || isnan(i_pcptr->points[idx].y) || isnan(i_pcptr->points[idx].y) ) 
        {
            // std::cout << "[RangeImage] GenerateRangeImg - This point has Nan value. Skipped." << std::endl;
            continue;
        }
        if( m_b_use_ring_data == true )
        {
            if(isnan(i_pcptr->points[idx].ring))
                continue;
        }
        uint16_t ui16_row_idx = 0, ui16_col_idx = 0;
        float    f_horizontal_angle_deg = 0, f_vertical_angle_deg = 0;
        float    f_point_x = i_pcptr->points[idx].x;
        float    f_point_y = i_pcptr->points[idx].y;
        float    f_point_z = i_pcptr->points[idx].z;

        if(m_str_lidar_name == "VELOHDL64E" ) // KITTI Dataset need to be calibrated.
            f_point_z = i_pcptr->points[idx].z - 0.185;

        // Get raw index from Ring information
        if(m_b_use_ring_data == true)
        {
            uint16_t f_point_ring = i_pcptr->points[idx].ring;
            if (m_b_is_ring_ascent == false)
                ui16_row_idx = f_point_ring; //Ouseter
            else
                ui16_row_idx = -(f_point_ring - m_n_height + 1);
        }
        else
        {
            float xy_range = sqrt( pow(f_point_x,2) + pow(f_point_y,2) );
            f_vertical_angle_deg = RAD2DEG( atan2(f_point_z, xy_range) );
            ui16_row_idx = GetRowIndex( f_vertical_angle_deg );
        }

        // Calculate horizontal angle(horizontal of point).
        f_horizontal_angle_deg = RAD2DEG( atan2(f_point_y, f_point_x) );

        if ( f_horizontal_angle_deg < m_vecd_horizontal_section_boundary_deg.front() || m_vecd_horizontal_section_boundary_deg.back() <  f_horizontal_angle_deg )
            continue;

        ui16_col_idx = GetColIndex( f_horizontal_angle_deg ); // Find col index of point.

        float    range_m = sqrt( pow(f_point_x,2) + pow(f_point_y,2) + pow(f_point_z,2) );

        if( ui16_row_idx < 0 || m_n_height < ui16_row_idx ||
            ui16_col_idx < 0 || m_n_width  < ui16_col_idx)
            continue;

        if ( m_min_range_threshold_m > range_m || range_m > m_max_range_threshold_m ) // Ignore noisy data.
            continue;

        if ( m_p_range_img_data[ui16_row_idx][ui16_col_idx] != FLT_MAX ) // Find duplicated pixel
        {
            // Filter duplicated pixel as option.
            switch (m_filtering_method)
            {
            case COMMING_FIRST:
                continue;
            case COMMING_LAST:
                break;
            case DISTANCE_NEAR:
                if(range_m > m_p_range_img_data[ui16_row_idx][ui16_col_idx])
                    continue;   // move to next point.
                break;
            case DISTANCE_FAR:
                if(range_m < m_p_range_img_data[ui16_row_idx][ui16_col_idx])
                    continue;   // move to next point.
                break;
            case INTENSITY_STRONG:
                if( i_pcptr->points[idx].intensity < i_pcptr->points[m_p_index_img_data[ui16_row_idx][ui16_col_idx]].intensity)
                    continue;
                break;
            default: // Default behavior is same as INTENSITY_STRONG
                if( i_pcptr->points[idx].intensity < i_pcptr->points[m_p_index_img_data[ui16_row_idx][ui16_col_idx]].intensity)
                    continue;
                break;
            }
            int previus_assigned_point_idx = m_p_index_img_data[ui16_row_idx][ui16_col_idx];
            m_i_v_point_to_image_idx[previus_assigned_point_idx] = {-1,-1};
        }

        m_p_range_img_data[ui16_row_idx][ui16_col_idx] = range_m;
        m_p_index_img_data[ui16_row_idx][ui16_col_idx] = idx;
        m_i_v_point_to_image_idx[idx] = {ui16_row_idx,ui16_col_idx};


    }
    return true;
}

/*
Brief
- Generate range image data
Input
- i_pcptr: Input point cloud pointer
Output
- None
Return
- True/False: False mean some error(example input pcptr is null_ptr)
*/
bool CRangeImage::GenerateRangeImg()
{
    return GenerateRangeImg(m_pcptr_input_pc);
}

/*
Brief
- Generate "key"'s mask
Input
- key: name of mask. example) "ground"
Output
- None
Return
- None
*/
void CRangeImage::GenerateMask(std::string key)
{
    if ( m_b_is_init == false )
        return;

	if (m_map_image_mask.find(key) != m_map_image_mask.end())
	{
		for (int idx = 0; idx < m_n_height; idx++)
		{
			memset(m_map_image_mask[key][idx], 0, m_n_width * sizeof(uint8_t));
		}
	}
	else
	{
		uint8_t **tmp_image_pointer;
		AllocateMemory(tmp_image_pointer);
		m_map_image_mask[key] = tmp_image_pointer;
	}
    return;
}

/*
Brief
- Generate "key"'s mask
Input
- key: name of mask. example) "angle_image"
Output
- None
Return
- None
*/
void CRangeImage::GenerateFloatMask(std::string key)
{
    if ( m_b_is_init == false )
        return;

	if (m_map_float_image_mask.find(key) != m_map_float_image_mask.end())
	{
		for (int idx = 0; idx < m_n_height; idx++)
		{
			memset(m_map_float_image_mask[key][idx], 0, m_n_width * sizeof(float));
		}
	}
	else
	{
		float **tmp_image_pointer;
		AllocateMemory(tmp_image_pointer);
		m_map_float_image_mask[key] = tmp_image_pointer;
	}
    return;
}

/*
Brief
- Check specipic Key's mask is being
Input
- key: name of mask. example) "ground"
Output
- None
Return
- bool: true - mask is being, false - mask is not being
*/
bool CRangeImage::IsMask(std::string key)
{
    if(m_map_image_mask.find(key) != m_map_image_mask.end()||
        m_map_float_image_mask.find(key) != m_map_float_image_mask.end())
        return true;
    else
        return false;
}

/*
Brief
- Get vertical-direction(row) index of image.
Input
- vertical angle of point
Output
- None
Return
- uint16_t index of image
*/
uint16_t CRangeImage::GetRowIndex ( float in_vertical_angle )
{
    uint16_t out_row_index = 0;
    int section_size = m_vecd_vertical_section_boundary_deg.size() - 1;
    for ( int i_vertical_section = 0; i_vertical_section < section_size; i_vertical_section++)
    {
        if(m_vecd_vertical_section_boundary_deg[i_vertical_section]  <= in_vertical_angle && in_vertical_angle <= m_vecd_vertical_section_boundary_deg[i_vertical_section+1])
        {
            int i_additional_index = m_veci_vertical_section_start_index[i_vertical_section];
            int i_current_point_index = abs( (m_vecf_vertical_section_angle_deg[i_vertical_section] - in_vertical_angle)/m_vecf_vertical_section_resolution_deg[i_vertical_section] );
            out_row_index = -(i_additional_index + i_current_point_index - m_n_height + 1);
            return out_row_index;
        }
    }
    return std::numeric_limits<uint16_t>::max();
}

/*
Brief
- Get horizontal-direction(column) index of image.
Input
- horizontal angle of point
Output
- None
Return
- uint16_t index of image
*/
uint16_t CRangeImage::GetColIndex( float in_horizontal_horizontal_angle )
{
    int i_section_size = m_vecd_horizontal_section_boundary_deg.size() - 1;
    for (int section_idx = 0; section_idx < i_section_size; section_idx++)
    {
        if( m_vecd_horizontal_section_boundary_deg[section_idx]  <= in_horizontal_horizontal_angle &&  in_horizontal_horizontal_angle <= m_vecd_horizontal_section_boundary_deg[section_idx + 1])
        {
            int i_additional_index    = m_veci_horizontal_section_start_index[section_idx]; // Accumulated index along the seciton.
            int i_current_point_index = abs( ( m_vecf_horizontal_section_angle_deg[section_idx] - in_horizontal_horizontal_angle ) / m_vecf_horizontal_section_resolution_deg[section_idx] ); // Find section of index in section.
            int out_col_index = -(i_additional_index + i_current_point_index - m_n_width + 1); // Find column index of point in lidar frame and make it always positive sign.
            return out_col_index;
        }
    }
    return  std::numeric_limits<uint16_t>::max();
}

/*
Brief
- Return MaskImage
Input
- None
Output
- None
Return
- uint8 array of mask image
*/
uint8_t** CRangeImage::GetMaskImg(std::string key)
{
    if(m_b_is_init == false)
        return NULL;
    if(m_map_image_mask.find(key) == m_map_image_mask.end())
        GenerateMask(key);

    return m_map_image_mask[key];
}

/*
Brief
- Return MaskImage the type of float
Input
- None
Output
- None
Return
- uint8 array of mask image
*/
float** CRangeImage::GetFloatMaskImg(std::string key)
{
    if(m_b_is_init == false)
        return NULL;
    if(m_map_float_image_mask.find(key) == m_map_float_image_mask.end())
        GenerateFloatMask(key);

    return m_map_float_image_mask[key];
}

/*
Brief
- Return the image pixel idx corresponded the point of point cloud.
Input
- point_idx: point idx of point at point cloud
Output
- None
Return
- image idx of point at point cloud, {-1, -1} mean no matched pixel
*/
std::vector<int> CRangeImage::GetPixelPointIdx(int point_idx)
{
    if(m_i_v_point_to_image_idx.empty() == true ||
        m_i_v_point_to_image_idx.size() < point_idx)
    {
        std::vector<int> non_pixel_point = {-1,-1};
        return non_pixel_point;
    }
    return m_i_v_point_to_image_idx[point_idx];
}

/*
Brief
- Retern index range value
Input
- row: index of row
- col: index of col
Output
- None
Return
- index(int) value of pixel
*/
int CRangeImage::GetIdx(uint16_t row, uint16_t col)
{
    if(m_b_is_init == false || m_p_index_img_data == NULL)
        return __INT32_MAX__;
    if(row >= m_n_height || col >= m_n_width)
        return __INT32_MAX__;
    return m_p_index_img_data[row][col];
}

/*
Brief
- Retern index range value
Input
- row: index of row
- col: index of col
Output
- None
Return
- range(float) value of pixel
*/
float CRangeImage::GetRange(uint16_t row, uint16_t col)
{
    if(m_b_is_init == false || m_p_range_img_data == NULL)
        return -1.;
    if(row >= m_n_height || col >= m_n_width)
        return -1.;
    return m_p_range_img_data[row][col];
}

/*
Brief
- Retern openCV matrix image of range image
Input
- None
Output
- None
Return
- cv::Mat image(float type)
*/
cv::Mat CRangeImage::GetRangeCVImage()
{
    cv::Mat mat_range_img ((int)m_n_height , (int)m_n_width, CV_32F , cv::Scalar::all(FLT_MAX));
    for (int row = 0; row < m_n_height; row++)
    {
        for (int col = 0; col < m_n_width; col++)
        {
            float range = m_p_range_img_data[row][col];
            mat_range_img.at<float>(row,col)   = range;
        }
    }
    return mat_range_img;
}

/*
Brief
- Retern openCV matrix image of index image
Input
- None
Output
- None
Return
- cv::Mat image(int type)
*/
cv::Mat CRangeImage::GetIndexCVImage()
{
    cv::Mat mat_index_img ((int)m_n_height , (int)m_n_width, CV_32S , cv::Scalar::all(FLT_MAX));
    for (int row = 0; row < m_n_height; row++)
    {
        for (int col = 0; col < m_n_width; col++)
        {
            int index = m_p_index_img_data[row][col];
            mat_index_img.at<int>(row,col)   = (int)index;
        }
    }
    return mat_index_img;
}

/*
Brief
- Retern openCV matrix image of key's mask(uint8) image
Input
- None
Output
- None
Return
- cv::Mat image(uint8 type)
*/
cv::Mat CRangeImage::GetMaskCVImage(std::string key)
{
    cv::Mat mat_mask_img ((int)m_n_height , (int)m_n_width, CV_8U , cv::Scalar::all(0));
    if(m_map_image_mask.find(key) == m_map_image_mask.end())
        return mat_mask_img;
    uint8_t** mask_image = m_map_image_mask[key];
    uint8_t value = 0;
    for (int row = 0; row < m_n_height; row++)
    {
        for (int col = 0; col < m_n_width; col++)
        {
            value = mask_image[row][col];
            mat_mask_img.at<uint8_t>(row,col)   = value;
        }
    }
    return mat_mask_img;
}

/*
Brief
- Retern openCV matrix image of key's mask(float) image
Input
- None
Output
- None
Return
- cv::Mat image(float type)
*/
cv::Mat CRangeImage::GetFloatMaskCVImage(std::string key)
{
    cv::Mat mat_mask_img ((int)m_n_height , (int)m_n_width, CV_32F , cv::Scalar::all(0.));
    if(m_map_float_image_mask.find(key) == m_map_float_image_mask.end())
        return mat_mask_img;
    float** mask_image = m_map_float_image_mask[key];
    float value = 0.;
    for (int row = 0; row < m_n_height; row++)
    {
        for (int col = 0; col < m_n_width; col++)
        {
            value = mask_image[row][col];
            mat_mask_img.at<float>(row,col)   = value;
        }
    }
    return mat_mask_img;
}

/*
Brief
- Extract labeled point idx
Input
- None
Output
- None
Return
- true/false
*/
bool CRangeImage::GetMaskIdx(std::string key, std::vector<int>& o_idx_vector)
{
    if(m_map_image_mask.find(key) == m_map_image_mask.end())
        return false;

    uint8_t** key_image = GetMaskImg(key);
    if(key_image == NULL)
        return false;

    for(uint16_t idx_row = 0; idx_row < m_n_height ; ++idx_row)
    {
        for(uint16_t idx_col = 0; idx_col < m_n_width ; ++idx_col)
        {
            if (key_image[idx_row][idx_col] != 0)
            {
                int idx = GetIdx(idx_row, idx_col);
                if(idx == __INT32_MAX__)
                    continue;
                o_idx_vector.push_back(idx);
            }
        }
    }
    return true;
}

/*
Brief
- Extract labeled point idx at each label value
Input
- None
Output
- None
Return
- true/false
*/
bool CRangeImage::GetNonMaskIdx(std::string key, std::vector<int>& o_idx_vector)
{
    if(m_map_image_mask.find(key) == m_map_image_mask.end())
        return false;

    uint8_t** key_image = GetMaskImg(key);
    if(key_image == NULL)
        return false;

    for(uint16_t idx_row = 0; idx_row < m_n_height ; ++idx_row)
    {
        for(uint16_t idx_col = 0; idx_col < m_n_width ; ++idx_col)
        {
            if (key_image[idx_row][idx_col] == 0)
            {
                int idx = GetIdx(idx_row, idx_col);
                if(idx == __INT32_MAX__)
                    continue;
                o_idx_vector.push_back(idx);
            }
        }
    }
    return true;
}

/*
Brief
- Extract labeled point cloud
Input
- None
Output
- None
Return
- true/false
*/
bool CRangeImage::GetMaskIndices(std::string key, std::vector<std::vector<int>>& o_idx_vectors)
{
    if(m_map_image_mask.find(key) == m_map_image_mask.end())
        return false;

    uint8_t** key_image = GetMaskImg(key);
    if(key_image == NULL)
        return false;

    std::map<int, std::vector<int>> map_labels;
    for(uint16_t idx_row = 0; idx_row < m_n_height ; ++idx_row)
    {
        for(uint16_t idx_col = 0; idx_col < m_n_width ; ++idx_col)
        {
            if (key_image[idx_row][idx_col] != 0)
            {
                map_labels[key_image[idx_row][idx_col]].push_back(m_p_index_img_data[idx_row][idx_col]);
            }
        }
    }
    for(auto iter = map_labels.begin(); iter != map_labels.end(); iter++)
    {
        o_idx_vectors.push_back(iter->second);
    }
    return true;
}

/*
Brief
- Extract labeled point cloud
Input
- None
Output
- None
Return
- true/false
*/
bool CRangeImage::GetMaskPointCloud(std::string key, boost::shared_ptr<pcl::PointCloud<PointXYZIRT>> output_pc)
{
    std::vector<int> labeled_point_idx;
    if(GetMaskIdx(key, labeled_point_idx) == false)
        return false;

    if(output_pc==NULL)
        output_pc.reset(new pcl::PointCloud<PointXYZIRT>);

    pcl::copyPointCloud(*m_pcptr_input_pc, labeled_point_idx, *output_pc);

    return true;
}

/*
Brief
- Extract labeled point cloud each label value
Input
- None
Output
- None
Return
- true/false
*/
bool CRangeImage::GetMaskPointClouds(std::string key, std::vector<boost::shared_ptr<pcl::PointCloud<PointXYZIRT>>>& output_pcs)
{
    std::vector<std::vector<int>> i_vec_label_idx;
    if(GetMaskIndices(key, i_vec_label_idx) == false)
        return false;

    for(auto label_iter = i_vec_label_idx.begin(); label_iter != i_vec_label_idx.end(); label_iter++)
    {
        boost::shared_ptr<pcl::PointCloud<PointXYZIRT>> tmp_pcptr_object(new pcl::PointCloud<PointXYZIRT>);
        pcl::copyPointCloud(*m_pcptr_input_pc,*label_iter,*tmp_pcptr_object);
        output_pcs.push_back(tmp_pcptr_object);
    }

    return true;
}


uint16_t CRangeImage::Height()
{
    return m_n_height;
}

uint16_t CRangeImage::Width()
{
    return m_n_width;
}

uint32_t CRangeImage::Size()
{
    return m_n_img_size;
}

bool CRangeImage::IsInit()
{
    if(m_pcptr_input_pc == NULL||m_pcptr_input_pc->empty() == true)
        return false;
    return true;
}

bool CRangeImage::ParseINIFile()
{
    std::string strTmp;
    int         m_tmp_int;
    if (m_ini_handler.ParseConfig("RangeImage", "LiDAR_name"     , m_str_lidar_name ) != true){ return false;}
    if (m_ini_handler.ParseConfig("RangeImage", "b_print_message", m_b_print_message) != true){ return false;}

    if (m_ini_handler.ParseConfig(m_str_lidar_name, "num_of_ring", m_tmp_int        ) != true){ return false;}
    m_n_height = (uint16_t)m_tmp_int;

    if (m_ini_handler.ParseConfig(m_str_lidar_name, "num_of_horizontal_points", m_tmp_int ) != true){ return false;}
    m_n_width  = (uint16_t)m_tmp_int;

    if (m_ini_handler.ParseConfig(m_str_lidar_name, "min_range_threshold_m"            , m_min_range_threshold_m                  ) != true){ return false;}
    if (m_ini_handler.ParseConfig(m_str_lidar_name, "max_range_threshold_m"            , m_max_range_threshold_m                  ) != true){ return false;}
    if (m_ini_handler.ParseConfig(m_str_lidar_name, "multi_echo_filtering_method"      , strTmp                                   ) != true){ return false;}
    if      (strTmp == "COMMING_FIRST"   ) { m_filtering_method = COMMING_FIRST   ;}
    else if (strTmp == "COMMING_LAST"    ) { m_filtering_method = COMMING_LAST    ;}
    else if (strTmp == "DISTANCE_NEAR"   ) { m_filtering_method = DISTANCE_NEAR   ;}
    else if (strTmp == "DISTANCE_FAR"    ) { m_filtering_method = DISTANCE_FAR    ;}
    else if (strTmp == "INTENSITY_STRONG") { m_filtering_method = INTENSITY_STRONG;}
    else                                   { m_filtering_method = INTENSITY_STRONG;}

    if (m_ini_handler.ParseConfig(m_str_lidar_name, "horizontal_section_boundary_deg" , m_vecf_horizontal_section_angle_deg      ) != true){ return false;}
    if (m_ini_handler.ParseConfig(m_str_lidar_name, "horizontal_section_resolution_deg", m_vecf_horizontal_section_resolution_deg ) != true){ return false;}
    if (m_ini_handler.ParseConfig(m_str_lidar_name, "use_ring_data" , m_b_use_ring_data ) != true){ return false;}
    if( m_b_use_ring_data == true )
        if (m_ini_handler.ParseConfig(m_str_lidar_name, "is_ring_ascent", m_b_is_ring_ascent) != true){ return false;}
    if( m_b_use_ring_data == false )
    {
        if (m_ini_handler.ParseConfig(m_str_lidar_name, "vertical_section_boundary_deg" , m_vecf_vertical_section_angle_deg     ) != true){ return false;}
        if (m_ini_handler.ParseConfig(m_str_lidar_name, "vertical_section_resolution_deg", m_vecf_vertical_section_resolution_deg) != true){ return false;}
    }
    return true;
}