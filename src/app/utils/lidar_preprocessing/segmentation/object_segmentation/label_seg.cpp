/**
 * @copyright (c) AI LAB - Konkuk Uni.
 * <br>All rights reserved. Subject to limited distribution and restricted disclosure only.
 * @author pauljiwon96@gmail.com, wodud3743@gmail.com
 * @file label_seg.cpp
 * @brief Range image based fast and efficent segmentation source file. Origin algorithm is [I. Bogoslavskyi and C. Stachniss,
 * “Fast Range Image-based Segmentation of Sparse 3D Laser Scans for Online Operation,”
 * Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems, pp. 163-169, 2016.].
 * This algorithm has some changes
 * @version 1.0
 * @date 06-10-2022
 * @bug No known bugs
 * @warning No warnings
 */

#include "segmentation/object_segmentation/label_seg.hpp"

CLabelSeg::CLabelSeg()
{
    m_cfg_b_use_ini_config  = false;
    m_cfg_i_neighbor_size   = 2;
    m_cfg_f_range_threshold_m = 0.3;
    m_cfg_f_dist_threshold_m = 1.5;
    m_cfg_f_segment_theta_deg = 60.0;
    m_cfg_i_segment_min_point_num = 10;
    m_cfg_i_segment_valid_point_num = 7;
    m_cfg_i_segment_valid_line_num = 3;
    m_cfg_b_print_message = false;
    NeighborAssign(m_cfg_i_neighbor_size);
    return;
}
CLabelSeg::~CLabelSeg()
{
    return;
}

bool CLabelSeg::Init(std::string ini_path)
{
    if(m_ini_handler.Init(ini_path) == false)
        return false;
    if(ParseIni() == false)
        return false;
    NeighborAssign(m_cfg_i_neighbor_size);
    return true;
}

/*
Brief
- Assign neighbor vector by using setted size(neighbor_size)
Input
- neighbor_size: the size of search neighbor
Output
- None
Return
- None
*/
void CLabelSeg::NeighborAssign(unsigned int neighbor_size)
{
    m_v_neighbor_iter.clear();
    std::pair<int8_t, int8_t> neighbor;
    for(int i = 1; i < neighbor_size+1; i++)
    {
        neighbor.first = -i; neighbor.second =  0; m_v_neighbor_iter.push_back(neighbor);
        neighbor.first =  0; neighbor.second =  i; m_v_neighbor_iter.push_back(neighbor);
        neighbor.first =  0; neighbor.second = -i; m_v_neighbor_iter.push_back(neighbor);
        neighbor.first =  i; neighbor.second =  0; m_v_neighbor_iter.push_back(neighbor);
    }
}

/*
Brief
- Segmentation the object in range image, ground mask have to be generated by ground filter
Input
- range_image: range image
Output
- None
Return
- true/false
*/

bool CLabelSeg::FilterObject(CRangeImage& range_image)
{
    return FilterObject(range_image, range_image.GetCloud());
}

bool CLabelSeg::FilterObject(CRangeImage& range_image, pcl::PointCloud<PointXYZIRT>::Ptr rph_calibrated_pcptr)
{
    pcptr_input = rph_calibrated_pcptr;

    if(m_ini_handler.IsFileUpdated())
    {
        ParseIni();
        NeighborAssign(m_cfg_i_neighbor_size);
    }

    if(range_image.IsMask("ground") == false)
        return false;

    m_i_row_size = range_image.Height();
    m_i_col_size = range_image.Width();

    Reset();

    uint8_t** ground_image = range_image.GetMaskImg("ground");

    for(int idx_row = 0 ; idx_row < m_i_row_size ; ++idx_row)
        for(int idx_col = 0 ; idx_col < m_i_col_size ; ++idx_col)
            if(ground_image[idx_row][idx_col] == 0 && m_v_b_segmented_image[idx_row][idx_col] == false ) // If not ground
                LabelComponents(range_image, idx_row,idx_col);

    if(m_v_object_pixels.size() == 0)
    {
        DebugPrintWarn("[CLabelSeg] No segment segmented ", m_cfg_b_print_message);
        return false;
    }
    std::sort(m_v_object_pixels.begin(), m_v_object_pixels.end(), by_size_decent());
    if(m_v_object_pixels.size() > UINT8_MAX)
    {
        m_v_object_pixels.erase(m_v_object_pixels.begin() + UINT8_MAX-1, m_v_object_pixels.end()); // Erase small size of segment
    }

    // Generate object image
    uint8_t** object_image = range_image.GetMaskImg("object");
    int label = 1;
    for(auto object_pixels_iter = m_v_object_pixels.begin(); object_pixels_iter != m_v_object_pixels.end(); object_pixels_iter++)
    {
        for(auto pixel_iter = object_pixels_iter->begin(); pixel_iter != object_pixels_iter->end(); pixel_iter++)
            object_image[pixel_iter->first][pixel_iter->second] = label;

        label++;
    }

    DebugPrintInfo("[CLabelSeg] Number of segment: " + std::to_string(m_v_object_pixels.size()), m_cfg_b_print_message);
    return true;
}

void CLabelSeg::LabelComponents(CRangeImage& range_image, uint16_t row, uint16_t col)
{
    float f_longer_range_m, f_shoter_range_m, alpha_rad, point_angle_reverse_rad , range_threshold_m, dist_threshold_m;
    int fromIndX, fromIndY, thisIndX, thisIndY;
    std::vector<bool> lineCountFlag(m_i_row_size,false);
    Pixels cur_label_pixels;

    int queueSize = 1;
    int queueStartInd = 0;
    int queueEndInd = 1;

    m_v_ui16_queue_idx_x[0] = col;
    m_v_ui16_queue_idx_y[0] = row;

    m_v_ui16_pushed_idx_x[0] = col;
    m_v_ui16_pushed_idx_y[0] = row;

    int allPushedIndSize = 1;

    int point_this_idx, point_from_idx;
    float this_point_x,this_point_y,this_point_z;
    float from_point_x,from_point_y,from_point_z;

    float this_ver_angle_rad,this_hor_angle_rad,from_ver_angle_rad,from_hor_angle_rad;

    float f_distance_btw_point_m,f_range_diff_m;

    uint8_t** ground_image = range_image.GetMaskImg("ground");

    while(queueSize > 0)
    {
        // Pop point
        fromIndX = m_v_ui16_queue_idx_x[queueStartInd];
        fromIndY = m_v_ui16_queue_idx_y[queueStartInd];
        --queueSize;
        ++queueStartInd;

        m_v_b_segmented_image[fromIndY][fromIndX] = true;
        point_from_idx = range_image.GetIdx(fromIndY, fromIndX);
        if (point_from_idx == __INT32_MAX__)
            continue;

        PixelCoord cur_pixel = {fromIndY, fromIndX};
        cur_label_pixels.push_back(cur_pixel);

        // Loop through all the neighboring grids of popped grid
        for (auto iter = m_v_neighbor_iter.begin(); iter != m_v_neighbor_iter.end(); ++iter)
        {
            // new index
            thisIndX = fromIndX + (*iter).first; // Horizontal Direction
            thisIndY = fromIndY + (*iter).second; // Vertical Direction

            // Index should be within the boundary.
            if (thisIndY < 0 || thisIndY >= m_i_row_size)
                continue;

            // Check continuity of col idx
            if (thisIndX < 0)
                thisIndX = thisIndX + m_i_col_size;
            if (thisIndX >= m_i_col_size )
                thisIndX = thisIndX - m_i_col_size;

            // Check current pixel is already segmented or ground
            if (m_v_b_segmented_image[thisIndY][thisIndX] == true || ground_image[thisIndY][thisIndX] != 0)
                continue;

            // Get current pixels point idx
            point_this_idx = range_image.GetIdx(thisIndY, thisIndX);

            if (point_this_idx == __INT32_MAX__)
                continue;

            f_longer_range_m = std::max(range_image.GetRange(fromIndY, fromIndX),
                          range_image.GetRange(thisIndY, thisIndX));
            f_shoter_range_m = std::min(range_image.GetRange(fromIndY, fromIndX),
                          range_image.GetRange(thisIndY, thisIndX));

            this_point_x = pcptr_input->points[point_this_idx].x;
            this_point_y = pcptr_input->points[point_this_idx].y;
            this_point_z = pcptr_input->points[point_this_idx].z;

            from_point_x = pcptr_input->points[point_from_idx].x;
            from_point_y = pcptr_input->points[point_from_idx].y;
            from_point_z = pcptr_input->points[point_from_idx].z;

            f_distance_btw_point_m = sqrt(pow(this_point_x - from_point_x,2) +
                                        pow(this_point_y - from_point_y,2) +
                                        pow(this_point_z - from_point_z,2));
            f_range_diff_m = abs(f_longer_range_m - f_shoter_range_m);


            if ((*iter).second == 0) // horizontal direction
            {
                this_hor_angle_rad = atan2(this_point_y,this_point_x);
                from_hor_angle_rad = atan2(from_point_y,from_point_x);

                alpha_rad = abs(this_hor_angle_rad - from_hor_angle_rad);
                if(alpha_rad > M_PI)
                    alpha_rad = abs(alpha_rad - 2.*M_PI);

                alpha_rad = alpha_rad * abs((*iter).first);
            }
            else // vertical direction
            {
                this_ver_angle_rad = atan2(this_point_z,sqrt(this_point_x*this_point_x + this_point_y*this_point_y));
                from_ver_angle_rad = atan2(from_point_z,sqrt(from_point_x*from_point_x + from_point_y*from_point_y));

                alpha_rad = abs(this_ver_angle_rad - from_ver_angle_rad);
                if(alpha_rad > M_PI){
                    alpha_rad = abs(alpha_rad - 2.*M_PI);
                }

                alpha_rad = alpha_rad * abs((*iter).second);
            }

            // If two point is same segment that point_angle_reverse_rad has the value around zero;
            point_angle_reverse_rad  = abs(atan((f_longer_range_m -f_shoter_range_m*cos(alpha_rad))/f_shoter_range_m*sin(alpha_rad)));

            if ((*iter).second == 0) // horizontal direction
                range_threshold_m = m_cfg_f_range_threshold_m + 5.0 *  point_angle_reverse_rad / (M_PI/2.0); // 1 + 5.0 * (0~1)
            else
                range_threshold_m = m_cfg_f_range_threshold_m;

            if ((*iter).second == 0) // horizontal direction
                dist_threshold_m = m_cfg_f_dist_threshold_m/2.0 * std::max(1.0, f_longer_range_m/5.0);
            else
                dist_threshold_m = m_cfg_f_dist_threshold_m * std::max(1.0, f_longer_range_m/5.0);


            // Check current pixel is same segment
            if (f_distance_btw_point_m < dist_threshold_m)
            {
                m_v_ui16_queue_idx_x[queueEndInd] = thisIndX;
                m_v_ui16_queue_idx_y[queueEndInd] = thisIndY;
                ++queueSize;
                ++queueEndInd;

                m_v_b_segmented_image[thisIndY][thisIndX] = true;
                PixelCoord cur_pixel = {thisIndY, thisIndX};
                cur_label_pixels.push_back(cur_pixel);

                lineCountFlag[thisIndY] = true;

                m_v_ui16_pushed_idx_x[allPushedIndSize] = thisIndX;
                m_v_ui16_pushed_idx_y[allPushedIndSize] = thisIndY;
                ++allPushedIndSize;
            }
        }
    }
    // check if this segment is valid
    bool feasibleSegment = false;
    if (allPushedIndSize >= m_cfg_i_segment_min_point_num)
        feasibleSegment = true;
    else if (allPushedIndSize >= m_cfg_i_segment_valid_point_num)
    {
        int lineCount = 0;
        for (size_t i = 0; i < m_i_row_size; ++i)
            if (lineCountFlag[i] == true)
                ++lineCount;
        if (lineCount >= m_cfg_i_segment_valid_line_num)
            feasibleSegment = true;
    }

    // segment is valid, mark these points
    if (feasibleSegment == true)
    {
        // Add current segments pixels
        m_v_object_pixels.push_back(cur_label_pixels);
    }
}

void CLabelSeg::Reset()
{
    m_v_ui16_pushed_idx_x.clear();
    m_v_ui16_pushed_idx_x.resize(m_i_col_size*m_i_row_size);
    m_v_ui16_pushed_idx_y.clear();
    m_v_ui16_pushed_idx_y.resize(m_i_col_size*m_i_row_size);

    m_v_ui16_queue_idx_x.clear();
    m_v_ui16_queue_idx_x.resize(m_i_col_size*m_i_row_size);
    m_v_ui16_queue_idx_y.clear();
    m_v_ui16_queue_idx_y.resize(m_i_col_size*m_i_row_size);

    m_v_object_pixels.clear();

    // Label component
    std::vector<std::vector<bool>> tmp_init_vector(m_i_row_size, std::vector<bool>(m_i_col_size, false));
    m_v_b_segmented_image.swap(tmp_init_vector);
}

bool CLabelSeg::ParseIni()
{

    if (m_ini_handler.ParseConfig("LabelSeg" , "m_cfg_b_use_ini_config",                m_cfg_b_use_ini_config ) != true){ return false;}
    if (m_cfg_b_use_ini_config == false)
        return true;

    if (m_ini_handler.ParseConfig("LabelSeg" , "m_cfg_i_neighbor_size",           m_cfg_i_neighbor_size ) != true){ return false;}
    if (m_ini_handler.ParseConfig("LabelSeg" , "m_cfg_f_range_threshold_m",       m_cfg_f_range_threshold_m ) != true){ return false;}
    if (m_ini_handler.ParseConfig("LabelSeg" , "m_cfg_f_dist_threshold_m",        m_cfg_f_dist_threshold_m ) != true){ return false;}
    if (m_ini_handler.ParseConfig("LabelSeg" , "m_cfg_f_segment_theta_deg",       m_cfg_f_segment_theta_deg ) != true){ return false;}
    if (m_ini_handler.ParseConfig("LabelSeg" , "m_cfg_i_segment_min_point_num",   m_cfg_i_segment_min_point_num ) != true){ return false;}
    if (m_ini_handler.ParseConfig("LabelSeg" , "m_cfg_i_segment_valid_point_num", m_cfg_i_segment_valid_point_num ) != true){ return false;}
    if (m_ini_handler.ParseConfig("LabelSeg" , "m_cfg_i_segment_valid_line_num",  m_cfg_i_segment_valid_line_num ) != true){ return false;}
    if (m_ini_handler.ParseConfig("LabelSeg" , "m_cfg_b_print_message",           m_cfg_b_print_message ) != true){ return false;}

    return true;
}