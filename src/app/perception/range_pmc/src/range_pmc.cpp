#include "range_pmc.hpp"

RangePmc::RangePmc(){};
RangePmc::~RangePmc(){};

void RangePmc::Init(range_pmc_params params){
    cout<<"[Range PMC] Init Started"<<endl;

    m_params = params;
    cout<<"f_horizontal_resolution: "   << m_params.f_horizontal_resolution<<endl;
    cout<<"f_vertical_resolution: "     << m_params.f_vertical_resolution<<endl;
    cout<<"f_min_range: "               << m_params.f_min_range<<endl;
    cout<<"f_max_range: "               << m_params.f_max_range<<endl;
    cout<<"f_range_threshold: "         << m_params.f_range_threshold<<endl;
    cout<<"i_max_range_image_num: "     << m_params.i_max_range_image_num<<endl;
    cout<<"f_min_key_frame_time: "      << m_params.f_min_key_frame_time<<endl;
    cout<<"f_min_key_frame_rot: "       << m_params.f_min_key_frame_rot<<endl;
    cout<<"f_fov_up: "                  << m_params.f_fov_up<<endl;
    cout<<"f_fov_down: "                << m_params.f_fov_down<<endl;
    cout<<"f_fov_left: "                << m_params.f_fov_left<<endl;
    cout<<"f_fov_right: "               << m_params.f_fov_right<<endl;
    cout<<"i_min_occluded_num: "        << m_params.i_min_occluded_num<<endl;


    m_pmc_xyzrgb_pcptr.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    m_range_image_xyzrgb_pcptr.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    m_cluster_xyzrgb_pcptr.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

    m_i_pixel_fov_up    = floor((m_params.f_fov_up   / 180.0*M_PI + 0.5 * M_PI)  / m_params.f_vertical_resolution);
    m_i_pixel_fov_down  = floor((m_params.f_fov_down / 180.0*M_PI + 0.5 * M_PI)  / m_params.f_vertical_resolution);
    m_i_pixel_fov_left  = floor((m_params.f_fov_left / 180.0*M_PI +  M_PI)       / m_params.f_horizontal_resolution);
    m_i_pixel_fov_right = floor((m_params.f_fov_right/ 180.0*M_PI +  M_PI)       / m_params.f_horizontal_resolution);

    m_i_row_size        = m_i_pixel_fov_up - m_i_pixel_fov_down;
    m_i_col_size        = m_i_pixel_fov_left - m_i_pixel_fov_right;
    m_point_soph_pointers.reserve(m_params.i_max_range_image_num); 

    cout<<"m_i_pixel_fov_up: "          << m_i_pixel_fov_up<<endl;
    cout<<"m_i_pixel_fov_down: "        << m_i_pixel_fov_down<<endl;
    cout<<"m_i_pixel_fov_left: "        << m_i_pixel_fov_left<<endl;
    cout<<"m_i_pixel_fov_right: "       << m_i_pixel_fov_right<<endl;


    m_b_key_frame = false;

    for (int i = 0; i < m_params.i_max_range_image_num; i++)
    {
        point_soph* p = new point_soph[MAX_POINT]; // 비어있는 point_soph 배열 생성 
        m_point_soph_pointers.push_back(p); // 메모리 할당 
    }

    m_temp_point_soph_pointer = new point_soph[MAX_POINT];

    for (int i = 0; i < m_params.i_max_range_image_num; i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr iter_ptr;
        iter_ptr.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        m_deque_range_image_xyzrgb_pcptr.push_back(iter_ptr);
    }

    NeighborAssign(2, 3);
    
    cout<<"[Range PMC] Init Done"<<endl;
}


void RangePmc::Filter(PointCloudXYZI::Ptr feats_undistort, const M3D & rot_end, const V3D & pos_end, const double & scan_end_time)
{   

    
    double t00 = omp_get_wtime();
    cout<<"[Range PMC] Filter Started"<<endl;

    m_b_key_frame = KeyFrameCheck(scan_end_time, rot_end, pos_end);


    int size = feats_undistort->points.size(); // point num

    cout<<"[Range PMC] size"<< size << endl;

    std::vector<int> index(size); // Point Index 할당 
    for (int i = 0; i < size; i++) {
        index[i] = i;
    }

    m_pmc_xyzrgb_pcptr.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

    vector<point_soph*> vec_point_ptr; // point_soph의 주소들을 저장하기 위한 벡터 
    vec_point_ptr.reserve(size);
    vec_point_ptr.resize(size);


    point_soph* p;  // point_soph 포인트 배열 
    if(m_b_key_frame == true){
        p = m_point_soph_pointers[m_i_cur_point_soph_pointers]; // 현재 range image내  point_soph들 메모리 불러옴. 초기화 할 것임 
    }
    else{
        p = m_temp_point_soph_pointer; // 임시 포인터. point_soph의 array임
    }

    // p 초기화
    for (auto i : index) // feats_undistort 의 크기 
    {   // 한 range image 내부를 iteration
        V3D p_body(feats_undistort->points[i].x, feats_undistort->points[i].y, feats_undistort->points[i].z); // 라이다 기준 point 위치
        V3D p_glob(rot_end * (p_body) + pos_end); // global point 위치 

        p[i].ind = i; // 포인트 고유 인덱스

        p[i].glob = p_glob;        
        p[i].dyn = UNKNOWN;
        p[i].rot = rot_end;
        // p[i].rot = rot_end.transpose();

        p[i].transl = pos_end;
        p[i].time = scan_end_time;
        p[i].local = p_body;
        p[i].intensity = feats_undistort->points[i].intensity;
        p[i].ground = false;
        p[i].cluster_ind = -1;

        p[i].r = 0;
        p[i].g = 0;
        p[i].b = 255;

        vec_point_ptr[i] = &p[i]; // p[i] 의 주소를 저장함
    }

    RangeImage::Ptr new_image_pointer(new RangeImage(rot_end, pos_end, scan_end_time, m_i_map_index)); // 신규 range image pointer


    // 이때 p는 keyframe이면 순환되는 주소 m_point_soph_pointers[m_i_cur_point_soph_pointers]
    // key frame이 아니면 m_temp_point_soph_pointer 
    // 이 함수에선 p의 주소를 new_image_pointer에 push_back 함 
    i_valid_position_num = 0;
    GenerateRangeImage(vec_point_ptr, scan_end_time, rot_end, pos_end, new_image_pointer);
    GroundSegmentation(new_image_pointer);

    if(m_params.b_cluster_level_filtering == true)
        ObjectSegmentation(new_image_pointer);

    std::cout<<"i_valid_position_num: "<<i_valid_position_num<<std::endl;
    std::cout<<"Cluster size: "<<new_image_pointer->cluster_poses_vec.size()<<" idx: "<< m_i_cluster_idx<<std::endl;
    
    // p는 point_soph 객체 배열을 가리키는 포인터
    // PMC 수행 
    double tcase1_start = omp_get_wtime();
    time_cluster_labeling = 0.0;
    time_demster = 0.0;
    time_gaussian = 0.0;

    i_small_incident = 0;
    i_total_incident = 0;
    i_prop_num = 0;
    i_gaussian = 0;
    i_dempster = 0;
    int i_ground = 0;
    int i_skip_cluster = 0;
    int i_skip_ind = 0;
    int p_cluster_ind;
    int p_ind_in_cluster;
    int p_cluster_size;
    int p_min_pos_ind;
    int static_count, dynamic_count, unknown_count, total_count;
    int i_skip_size;
    for (auto i : index)
    {   // 한 range image 내부를 iteration

        // 벗어난 포인트 제외
        if(p[i].position < 0 || p[i].position >= MAX_2D_N) continue;

        // Assume 1: Ground is Static
        if (new_image_pointer->ground_all[p[i].position] == true){
            p[i].dyn = STATIC;
            p[i].r = 255; p[i].g = 0; p[i].b = 0;
            i_ground++;
            continue;
        }
        
        // Clustering에 의한 라벨링이 된 p는 skip
        if (m_params.b_cluster_level_filtering == true){
            p_cluster_ind = new_image_pointer->cluster_idx_all[p[i].position];

            if (p_cluster_ind >= 0 ){

                p_cluster_size = new_image_pointer->cluster_poses_vec[p_cluster_ind].size();
        
                p_min_pos_ind = new_image_pointer->min_range_index_all[p[i].position];
                // p_ind_in_cluster = FindIndexInVector(new_image_pointer->cluster_poses_vec[p_cluster_ind], p[i].position);

                // // std::cout<<"p_ind_in_cluster: "<<p_ind_in_cluster<<std::endl;
                // if(p_ind_in_cluster % max(p_cluster_size/10,2) != 0){
                //     i_skip_ind++;
                //     continue;
                // }


                // Skip point in clustering
                i_skip_size = sqrt(p_cluster_size) / 6;
                if(i_skip_size < 1 )  i_skip_size = 1;
                if(p[i].hor_ind % i_skip_size != 0 || p[i].ver_ind % i_skip_size != 0){
                    i_skip_ind++;
                    continue;
                }
                

                static_count    =  new_image_pointer->range_image[p[i].position][p_min_pos_ind]->r;
                dynamic_count   =  new_image_pointer->range_image[p[i].position][p_min_pos_ind]->g;
                unknown_count   =  new_image_pointer->range_image[p[i].position][p_min_pos_ind]->b;

                if(dynamic_count > static_count && dynamic_count > unknown_count){
                    p[i].dyn = CASE1;
                    // p[i].r = 0; p[i].g = 255; p[i].b = 0;
                    i_skip_cluster++;
                    continue;
                }       
                else if (static_count > dynamic_count && static_count > unknown_count){
                    p[i].dyn = STATIC;
                    i_skip_cluster++;
                    // p[i].r = 255; p[i].g = 0; p[i].b = 0;
                    continue;
                }  
                
            }
        }

        // p[i]는 value
        Case1(p[i], new_image_pointer);
    }

    std::cout<<"[Range PMC] Propagated  : "<< i_prop_num <<std::endl;
    std::cout<<"[Range PMC] Ground  Skip: "<< i_ground <<" / "<< size <<std::endl;
    std::cout<<"[Range PMC] Skip Index: "<< i_skip_ind <<" / "<< size <<std::endl;
    std::cout<<"[Range PMC] Cluster Skip: "<< i_skip_cluster <<" / "<< size <<std::endl;
    std::cout<<"[Range PMC] Small Incident: "<< i_small_incident <<" / "<< i_total_incident<<std::endl;

    double tcase1_end = omp_get_wtime() - tcase1_start;
    std::cout<<"[Range PMC] Case Gussian num  : "<< i_gaussian <<std::endl;
    std::cout<<"[Range PMC] Case Cluster Gaussian: "<< time_gaussian*1000.0 << " ms"<<std::endl;
    std::cout<<"[Range PMC] Case Cluster Gaussian mean: "<< time_gaussian*1000.0 / i_gaussian << " ms"<<std::endl;
    std::cout<<"[Range PMC] Case Cluster Demster: "<< time_demster*1000.0 << " ms"<<std::endl;
    std::cout<<"[Range PMC] Case Cluster Demster mean: "<< time_demster*1000.0 / i_dempster<< " ms"<<std::endl;
    std::cout<<"[Range PMC] Case Cluster Labeling: "<< time_cluster_labeling*1000.0 << " ms"<<std::endl;
    std::cout<<"[Range PMC] Case: "<< tcase1_end*1000.0 << " ms"<<std::endl;


    // Range Image queue에 저장 
    if(m_b_key_frame == true){

        if(m_range_image_ptr_list.size() < m_params.i_max_range_image_num){
            m_i_map_index++;
            m_range_image_ptr_list.push_back(new_image_pointer);
        }
        else{
            m_i_map_index++;
            m_range_image_ptr_list.pop_front();
            m_range_image_ptr_list.push_back(new_image_pointer); // 벡터의 요소가 가리키는 포인터 메모리 교체 
        }
        m_i_cur_point_soph_pointers = (m_i_cur_point_soph_pointers + 1) % m_params.i_max_range_image_num; // 다음 번 range image 의 index를 지정함 
    }


    std::cout<<"[Range PMC] Image num: "<< m_range_image_ptr_list.size() <<std::endl;

    // Output
    OutputPmcPC(vec_point_ptr, new_image_pointer);


    time_total = omp_get_wtime() - t00;
    std::cout<<"[Range PMC] Total Time: "<< time_total*1000.0 << " ms"<<std::endl;
}

void RangePmc::Case1(point_soph & p, RangeImage::Ptr range_image_ptr)
{
    int i_range_image_num = m_range_image_ptr_list.size();
    int i_valid_image = 0;
    float f_gaussian_weight;

    dyn_obj_flg p_dyn;

    for (int i = i_range_image_num - 1; i >= 0; i--) // point 한개에 대해 range image 탐색. 최근 range image 부터 
    {   
        // p에는 glob이 있어야하고, 이 함수를 통해서 vec, hor_ind, ver_ind, position 가 m_range_image_ptr_list[i] 기준으로 바뀜 
        SphericalProjection(p, m_range_image_ptr_list[i]->map_index, m_range_image_ptr_list[i]->project_R, m_range_image_ptr_list[i]->project_T, p);

        if (p.ver_ind > m_i_pixel_fov_up || p.ver_ind < m_i_pixel_fov_down || 
            p.hor_ind > m_i_pixel_fov_left || p.hor_ind < m_i_pixel_fov_right || p.vec(2) < 0.0f \
            || p.position < 0 || p.position >= MAX_2D_N)
        {
            continue; // range image의 경계를 넘은 point는 iter range image와 맞지 않음 skip
        }

        if (Case1Enter(p, *m_range_image_ptr_list[i], p_dyn) == true)
        {   
            i_valid_image++;
        }
        
        if (p_dyn == CASE2){
            // TODO 현재 포인트가 직전 range point보다 먼 경우
            p_dyn = UNKNOWN; // TODO
        }

        // Dempster
        EvidType SrcEvidence[3] = {(EvidType)p.r, (EvidType)p.g, (EvidType)p.b};
        if(p_dyn == CASE1){
            
            EvidType MovingEvidence[3] = {0, (EvidType)(m_params.f_moving_confidence * MAX_EVIDENCE),
                                            (EvidType)((1. - m_params.f_moving_confidence) * MAX_EVIDENCE)};
            
            DempsterCombination(SrcEvidence, MovingEvidence);

            p.r = (uint8_t)SrcEvidence[0];
            p.g = (uint8_t)SrcEvidence[1];
            p.b = (uint8_t)SrcEvidence[2];
        }
        else if (p_dyn == STATIC){
            EvidType MovingEvidence[3] = {(EvidType)(m_params.f_static_confidence * MAX_EVIDENCE), 0,
                                            (EvidType)((1. - m_params.f_static_confidence) * MAX_EVIDENCE)};

            DempsterCombination(SrcEvidence, MovingEvidence);
            
            p.r = (uint8_t)SrcEvidence[0];
            p.g = (uint8_t)SrcEvidence[1];
            p.b = (uint8_t)SrcEvidence[2];
        }


        // Range Image 돌다가 레이블링 끝나면 break
        if((p.r > p.g && p.r > p.b) || (p.g > p.r && p.g > p.b)) break;

    }


    // Dempster
    if(p.r > p.g && p.r > p.b){
        p.dyn = STATIC;
    } 
    else if(p.g > p.r && p.g > p.b){
        p.dyn = CASE1;
    } 
    else{
        p.dyn = UNKNOWN;
        return;
    } 

    // Cluster Labeling. min pose에만 labeling 함
    if (m_params.b_cluster_level_filtering == true && p.cluster_ind >= 0){
        double t_cluster_labeling_start = omp_get_wtime();
        double t_demster_start, t_gaussian_start;

        for (const auto pixel_pos : range_image_ptr->cluster_poses_vec[p.cluster_ind])
        {
            if (pixel_pos < MAX_2D_N && pixel_pos >= 0 ){
                t_gaussian_start = omp_get_wtime();
                int min_pos = range_image_ptr->min_range_index_all[pixel_pos];
                
                // Get Gaussian weight
                if(p.dyn == CASE1)
                    f_gaussian_weight = GaussianWeight((p.local - range_image_ptr->range_image[pixel_pos][min_pos]->local).norm(), m_params.f_dynamic_gaussian_sigma);
                else
                    f_gaussian_weight = GaussianWeight((p.local - range_image_ptr->range_image[pixel_pos][min_pos]->local).norm(), m_params.f_static_gaussian_sigma);

                i_gaussian++;
                time_gaussian += omp_get_wtime() - t_gaussian_start;

                if( f_gaussian_weight < m_params.f_sigma_epsilon ) continue;

                i_prop_num++;

                t_demster_start = omp_get_wtime();
                i_dempster++;
                EvidType SrcEvidence[3] = {(EvidType)range_image_ptr->range_image[pixel_pos][min_pos]->r, 
                                            (EvidType)range_image_ptr->range_image[pixel_pos][min_pos]->g,
                                            (EvidType)range_image_ptr->range_image[pixel_pos][min_pos]->b};
                                            
                // 이미 분류 끝난 position은 dempster 수행 x. 근데 이거 한다고 크게 빨라지지 않음 
                // 이 if문 윗블럭에서의 시간 합이 아래 시간 합의 10배가 넘기 때문...
                // if(SrcEvidence[0] > SrcEvidence[2] || SrcEvidence[1] > SrcEvidence[2])
                //     continue;
                
                if(p.dyn == CASE1){
                    
                    EvidType MovingEvidence[3] = {0, (EvidType)(m_params.f_moving_confidence * f_gaussian_weight * MAX_EVIDENCE),
                                                  (EvidType)((1. - m_params.f_moving_confidence * f_gaussian_weight) * MAX_EVIDENCE)};
                    
                    DempsterCombination(SrcEvidence, MovingEvidence);

                    range_image_ptr->range_image[pixel_pos][min_pos]->r = (uint8_t)SrcEvidence[0];
                    range_image_ptr->range_image[pixel_pos][min_pos]->g = (uint8_t)SrcEvidence[1];
                    range_image_ptr->range_image[pixel_pos][min_pos]->b = (uint8_t)SrcEvidence[2];
                }
                else if (p.dyn == STATIC){
                    EvidType MovingEvidence[3] = {(EvidType)(m_params.f_static_confidence * f_gaussian_weight * MAX_EVIDENCE), 0,
                                                  (EvidType)((1. - m_params.f_static_confidence * f_gaussian_weight) * MAX_EVIDENCE)};

                    DempsterCombination(SrcEvidence, MovingEvidence);
                    
                    range_image_ptr->range_image[pixel_pos][min_pos]->r = (uint8_t)SrcEvidence[0];
                    range_image_ptr->range_image[pixel_pos][min_pos]->g = (uint8_t)SrcEvidence[1];
                    range_image_ptr->range_image[pixel_pos][min_pos]->b = (uint8_t)SrcEvidence[2];
                }
                time_demster += omp_get_wtime() - t_demster_start;
            }
        }

        time_cluster_labeling += omp_get_wtime() - t_cluster_labeling_start;
    }
}

// return true if p is occluding previous range images
bool RangePmc::Case1Enter(const point_soph & p, const RangeImage &image_info, dyn_obj_flg & p_dyn)
{   
    // p는 image_info 기준으로 projection 되어있음
    float f_max_range = 0.0, f_min_range = 0.0;
    float f_cur_max_range = 0.0, f_cur_min_range = 0.0;
    p_dyn = UNKNOWN;

    f_max_range = image_info.max_range_all[p.position];
    f_min_range = image_info.min_range_all[p.position];

    if(p.ver_ind <= m_i_pixel_fov_up    && p.ver_ind >  m_i_pixel_fov_down && \
       p.hor_ind <= m_i_pixel_fov_left  && p.hor_ind >= m_i_pixel_fov_right)
    {
        CheckNeighbor(p, image_info, f_max_range, f_min_range);
    }
 
    if(f_min_range < 10E-5){ // 비교할 포인트를 찾지 못함 
        p_dyn = UNKNOWN;
        return false;
    }
    

    if( f_max_range - f_min_range > 4 * m_params.f_range_threshold) 
        f_max_range = f_min_range + 4 * m_params.f_range_threshold;

    int map_ind_gap = m_i_map_index - image_info.map_index;
    if(map_ind_gap < 0) map_ind_gap += m_params.i_max_range_image_num;
    
    if (p.vec(2) < f_min_range - m_params.f_range_threshold ){ // if point is closer than min range
        p_dyn = CASE1;
        return true;
    }
    else if (p.vec(2) < f_max_range + m_params.f_range_threshold){ // if point is Static
        p_dyn = STATIC;
        return true;
    }
    else if (map_ind_gap <= 1){
        p_dyn = CASE2;
        return true;
    }
    else{
        p_dyn = UNKNOWN;
        return false;
    }

    return false;
}


// points엔 point_soph들의 값이 있음 
void RangePmc::GenerateRangeImage(std::vector<point_soph*> &points, double cur_time, M3D rot, V3D transl, RangeImage::Ptr range_image_ptr)
{   
    double t00 = omp_get_wtime();
    int i_points_len = points.size();
    std::cout<<"[GenerateRangeImage] Started"<<std::endl;
    std::cout<<"[GenerateRangeImage] m_i_cur_point_soph_pointers: "<<m_i_cur_point_soph_pointers<<std::endl;
    std::cout<<"[GenerateRangeImage] map_index: "<<m_i_map_index<<std::endl;

    int i_invalid_num = 0;
    float f_min_ver_angle = FLT_MAX;
    float f_max_ver_angle = -FLT_MAX;
    for (int k = 0; k < i_points_len; k++)
    { 
        points[k]->GetVec(points[k]->local, m_params.f_horizontal_resolution, m_params.f_vertical_resolution);

        if (points[k]->vec(1) < f_min_ver_angle) f_min_ver_angle = points[k]->vec(1);
        if (points[k]->vec(1) > f_max_ver_angle) f_max_ver_angle = points[k]->vec(1);
        
        // FOV를 벗어난 point는 range image에 할당하지 않음
        if(points[k]->ver_ind > m_i_pixel_fov_up || points[k]->ver_ind < m_i_pixel_fov_down || \
           points[k]->hor_ind > m_i_pixel_fov_left || points[k]->hor_ind < m_i_pixel_fov_right || \
           points[k]->position < 0 || points[k]->position >= MAX_2D_N)
        {
            points[k]->dyn = INVALID;
            i_invalid_num++;
            continue;
        }

        if(range_image_ptr->range_image[points[k]->position].size() < m_params.i_max_points_in_pixel) // 해당 픽셀에 여유가 있다면 
        {   
            if(range_image_ptr->range_image[points[k]->position].size() == 0) i_valid_position_num++;

            range_image_ptr->range_image[points[k]->position].push_back(points[k]); // 주소를 뒤에 추가함 

            // 이 포인트의 range가 픽셀 내 모든 포인트보다 크다면 
            if (points[k]->vec(2) > range_image_ptr->max_range_all[points[k]->position])  
            {
                range_image_ptr->max_range_all[points[k]->position] = points[k]->vec(2); // 최대 range 교체
                range_image_ptr->max_range_index_all[points[k]->position] = range_image_ptr->range_image[points[k]->position].size()-1;
            }

            // 이 포인트의 range가 픽셀 내 모든 포인트보다 작다면, 혹은 min_range_all가 0이라면
            if (points[k]->vec(2) < range_image_ptr->min_range_all[points[k]->position] ||\
                range_image_ptr->min_range_all[points[k]->position] < 10E-5)  
            {
                range_image_ptr->min_range_all[points[k]->position] = points[k]->vec(2); // 최소 range 교체
                range_image_ptr->min_range_index_all[points[k]->position] = range_image_ptr->range_image[points[k]->position].size() - 1;
            }

        }
    }

    std::cout<<"[GenerateRangeImage] min ver angle: "<<f_min_ver_angle*180/M_PI<< " max: "<< f_max_ver_angle*180/M_PI << std::endl;
    std::cout<<"[GenerateRangeImage] Invalid Num: "<<i_invalid_num<<" / "<<i_points_len<<std::endl;
    double time_generate_range_image = omp_get_wtime() - t00;
    std::cout<<"[GenerateRangeImage] Total Time: "<< time_generate_range_image*1000.0 << " ms"<<std::endl;
}

// Segment Ground from Range Image
void RangePmc::GroundSegmentation(RangeImage::Ptr range_image_ptr)
{   
    double t00 = omp_get_wtime();

    V3D u_point, l_point;
    float f_ver_angle, f_incident_angle, f_hor_incident_angle, f_ver_incident_angle;
    float f_hor_dist, f_ver_dist;
    float f_hor_dist_ground;
    float f_upper_range, f_lower_range;
    float f_ver_angle_diff, f_hor_angle_diff, f_ver_angle_from_ground;
    float f_ground_margin_angle;

    int i_upper_pos, i_lower_pos, i_upper_point_min_pos, i_lower_point_min_pos;

    for(int i_hor_idx = m_i_pixel_fov_right; i_hor_idx < m_i_pixel_fov_left; i_hor_idx++)
    {
        for(int i_ver_idx = m_i_pixel_fov_down; i_ver_idx < m_i_pixel_fov_up; i_ver_idx++)  // vertical은 하나 적게 올라감
        {   
            i_lower_pos = i_hor_idx * MAX_1D_HALF + i_ver_idx;
            i_lower_point_min_pos = range_image_ptr->min_range_index_all[i_lower_pos];

            if(i_lower_point_min_pos == -1 || range_image_ptr->range_image[i_lower_pos].size() == 0) continue; // Check lower point valid

            l_point = range_image_ptr->range_image[i_lower_pos][i_lower_point_min_pos]->local;
            f_lower_range = range_image_ptr->range_image[i_lower_pos][i_lower_point_min_pos]->vec(2);

            for (int u_ver_idx = i_ver_idx + 1; u_ver_idx < i_ver_idx + 20 && u_ver_idx < m_i_pixel_fov_up ; u_ver_idx++){
                
                i_upper_pos = i_hor_idx * MAX_1D_HALF + u_ver_idx;
                i_upper_point_min_pos = range_image_ptr->min_range_index_all[i_upper_pos];

                u_point = range_image_ptr->range_image[i_upper_pos][i_upper_point_min_pos]->local;
                f_upper_range = range_image_ptr->range_image[i_upper_pos][i_upper_point_min_pos]->vec(2);

                if(i_upper_point_min_pos == -1 || range_image_ptr->range_image[i_upper_pos].size() == 0) continue; // Check upper point valid

                f_ver_angle_diff = range_image_ptr->range_image[i_upper_pos][i_upper_point_min_pos]->vec(1) 
                               - range_image_ptr->range_image[i_lower_pos][i_lower_point_min_pos]->vec(1);

                f_hor_dist = sqrt(pow(u_point.x() - l_point.x(), 2) + pow(u_point.y() - l_point.y(), 2));
                f_ver_dist = u_point.z() - l_point.z();

                f_hor_dist_ground = sqrt(l_point.x() * l_point.x() + l_point.y() * l_point.y());
                f_ver_angle_from_ground = atan2f(l_point.z() + 1.6, f_hor_dist_ground);
                f_ground_margin_angle = atan2f(- l_point.z() - 1.1, f_hor_dist_ground);

                f_ver_angle = atan2f(f_ver_dist,f_hor_dist);
                
                if( f_upper_range*cos(f_ver_angle_diff) - f_lower_range > 0 )
                    f_ver_incident_angle = M_PI/2 - atan2f(f_upper_range*sin(f_ver_angle_diff), f_upper_range*cos(f_ver_angle_diff) - f_lower_range);
                else
                    f_ver_incident_angle = M_PI/2 - atan2f(f_upper_range*sin(f_ver_angle_diff), - f_upper_range*cos(f_ver_angle_diff) + f_lower_range);     

                f_incident_angle = f_ver_incident_angle;

                range_image_ptr->range_image[i_lower_pos][i_lower_point_min_pos]->incident = f_incident_angle;
                range_image_ptr->incident_all[i_lower_pos] = f_incident_angle;

                if(fabs(f_ver_angle) < m_params.f_ground_angle * M_PI/180.0  && f_ver_angle_from_ground < m_params.f_ground_angle * M_PI/180.0){
                    range_image_ptr->range_image[i_upper_pos][i_upper_point_min_pos]->ground = true;
                    range_image_ptr->ground_all[i_upper_pos] = true;
                }
                if( f_ground_margin_angle > m_params.f_ground_angle * M_PI/180.0 ){
                    range_image_ptr->range_image[i_lower_pos][i_lower_point_min_pos]->ground = true;
                    range_image_ptr->ground_all[i_lower_pos] = true;
                }

                break;
            }

        }   
    }

    double ground_total = omp_get_wtime() - t00;
    std::cout<<"[GroundSegmentation] Total Time: "<< ground_total*1000.0 << " ms"<<std::endl;
}

// Segement Object
void RangePmc::ObjectSegmentation(RangeImage::Ptr range_image_ptr)
{
    double t00 = omp_get_wtime();

    ResetClustering();

    range_image_ptr->cluster_poses_vec.clear();

    int i_from_pos, i_from_min_pos;

    m_i_cluster_idx = 0;
    for(int i_ver_idx = m_i_pixel_fov_down ; i_ver_idx < m_i_pixel_fov_up ; ++i_ver_idx)
    {
        for(int i_hor_idx = m_i_pixel_fov_right ; i_hor_idx < m_i_pixel_fov_left ; ++i_hor_idx)
        {   
            i_from_pos = i_hor_idx * MAX_1D_HALF + i_ver_idx;
            i_from_min_pos = range_image_ptr->min_range_index_all[i_from_pos];

            if(range_image_ptr->range_image[i_from_pos].size() > 0 &&
               range_image_ptr->ground_all[i_from_pos] == false &&
               range_image_ptr->range_image[i_from_pos][i_from_min_pos]->cluster_ind == -1)
            {
                LabelComponents(range_image_ptr, i_ver_idx, i_hor_idx);
            }
        }   
    }
    std::cout<<"[ObjectSegmentation] Total segment: "<< m_i_cluster_idx <<std::endl;

    double object_total = omp_get_wtime() - t00;
    std::cout<<"[ObjectSegmentation] Total Time: "<< object_total*1000.0 << " ms"<<std::endl;
}


void RangePmc::NeighborAssign(unsigned int hor_neighbor, unsigned int ver_heighbor)
{
    m_v_neighbor_iter.clear();
    std::pair<int8_t, int8_t> neighbor;
    
    for(int i_hor_ind = 1; i_hor_ind <= hor_neighbor; i_hor_ind++ ){
        for(int i_ver_ind = 1; i_ver_ind <= ver_heighbor; i_ver_ind++ ){
            neighbor.first = -i_hor_ind;  neighbor.second =  0;       m_v_neighbor_iter.push_back(neighbor);
            neighbor.first =  0;        neighbor.second =  i_ver_ind; m_v_neighbor_iter.push_back(neighbor);
            neighbor.first =  0;        neighbor.second = -i_ver_ind; m_v_neighbor_iter.push_back(neighbor);
            neighbor.first =  i_hor_ind;  neighbor.second =  0;       m_v_neighbor_iter.push_back(neighbor);
        }
    }
}

void RangePmc::LabelComponents(RangeImage::Ptr range_image_ptr, uint16_t row, uint16_t col)
{   
    // row = Y = ver_ind = fov up down
    // col = X = hor_ind = fov left right

    float f_longer_range_m, dist_threshold_m;
    int i_from_x_idx, i_from_y_idx, i_this_x_idx, i_this_y_idx;
    std::vector<bool> lineCountFlag(m_i_row_size,false);

    int i_queue_size = 1;
    int i_queue_start_idx = 0;
    int i_queue_end_idx = 1;

    m_v_ui16_queue_idx_x[0] = col;
    m_v_ui16_queue_idx_y[0] = row;

    int i_all_pushed_idx_size = 1;

    float f_distance_btw_point_m;

    int i_this_pos, i_this_min_pos;
    int i_from_pos, i_from_min_pos;

    std::vector<int> vec_cluster_pos;

    std::vector<std::pair<int, int>> vec_pair_pos_min_pos; // (position, min_range_position) vec

    while(i_queue_size > 0)
    {
        // Pop point
        i_from_x_idx = m_v_ui16_queue_idx_x[i_queue_start_idx];
        i_from_y_idx = m_v_ui16_queue_idx_y[i_queue_start_idx];
        --i_queue_size;
        ++i_queue_start_idx;

        i_from_pos = i_from_x_idx * MAX_1D_HALF + i_from_y_idx;
        i_from_min_pos = range_image_ptr->min_range_index_all[i_from_pos];

        // 해당 픽셀에 포인트가 없거나, ground이면 skip
        if (range_image_ptr->range_image[i_from_pos].size() == 0 ||
            range_image_ptr->ground_all[i_from_pos] == true)
            continue;

        range_image_ptr->range_image[i_from_pos][i_from_min_pos]->cluster_ind = -2; // 검색 되었음의 의미로 -2 할당

        vec_pair_pos_min_pos.push_back(make_pair(i_from_pos, i_from_min_pos));

        // Loop through all the neighboring grids of popped grid
        for (auto iter = m_v_neighbor_iter.begin(); iter != m_v_neighbor_iter.end(); ++iter)
        {
            // new index
            i_this_x_idx = i_from_x_idx + (*iter).first; // Horizontal Direction
            i_this_y_idx = i_from_y_idx + (*iter).second; // Vertical Direction

            // Index should be within the boundary.
            if (i_this_y_idx < m_i_pixel_fov_down || i_this_y_idx >= m_i_pixel_fov_up)
                continue;

            // Check continuity of col idx
            if (i_this_x_idx < m_i_pixel_fov_right) i_this_x_idx = i_this_x_idx + m_i_col_size;
            if (i_this_x_idx >= m_i_pixel_fov_left) i_this_x_idx = i_this_x_idx - m_i_col_size;

            i_this_pos = i_this_x_idx * MAX_1D_HALF + i_this_y_idx;
            i_this_min_pos = range_image_ptr->min_range_index_all[i_this_pos];


            // Check current pixel is already segmented or ground
            if (range_image_ptr->range_image[i_this_pos].size() == 0 ||
                range_image_ptr->ground_all[i_this_pos] == true ||
                range_image_ptr->range_image[i_this_pos][i_this_min_pos]->cluster_ind != -1)
                continue;            

            f_distance_btw_point_m = (range_image_ptr->range_image[i_from_pos][i_from_min_pos]->local - 
                                      range_image_ptr->range_image[i_this_pos][i_this_min_pos]->local).norm();


            dist_threshold_m = m_params.f_dist_threshold_m; // Debug

            // Check current pixel is same segment
            if (f_distance_btw_point_m < dist_threshold_m)
            {
                m_v_ui16_queue_idx_x[i_queue_end_idx] = i_this_x_idx;
                m_v_ui16_queue_idx_y[i_queue_end_idx] = i_this_y_idx;
                ++i_queue_size;
                ++i_queue_end_idx;

                range_image_ptr->range_image[i_this_pos][i_this_min_pos]->cluster_ind = -2; // 탐색 되었음 라벨링 

                lineCountFlag[i_this_y_idx - m_i_pixel_fov_down] = true;

                vec_pair_pos_min_pos.push_back(make_pair(i_this_pos, i_this_min_pos));

                ++i_all_pushed_idx_size;
            }
        }
    }
    

    // check if this segment is valid

    bool b_feasible_segment = false;
    if (i_all_pushed_idx_size >= m_params.i_segment_min_point_num)
        b_feasible_segment = true;
    else if (i_all_pushed_idx_size >= m_params.i_segment_valid_point_num)
    {
        int lineCount = 0;
        for (int i = 0; i < m_i_row_size; ++i)
            if (lineCountFlag[i] == true)
                ++lineCount;

        if (lineCount >= m_params.i_segment_valid_line_num)
            b_feasible_segment = true;
    }

    // segment is valid, mark these points
    if (b_feasible_segment == true && vec_pair_pos_min_pos.size() > 0)
    {
        for (auto pos_min_pos : vec_pair_pos_min_pos){
            range_image_ptr->range_image[pos_min_pos.first][pos_min_pos.second]->cluster_ind = m_i_cluster_idx;
            range_image_ptr->cluster_idx_all[pos_min_pos.first] = m_i_cluster_idx;
            vec_cluster_pos.push_back(pos_min_pos.first); // pos 
        }

        range_image_ptr->cluster_poses_vec.push_back(vec_cluster_pos);
        m_i_cluster_idx++;
    }
}


void RangePmc::ResetClustering()
{
    m_v_ui16_queue_idx_x.clear();
    m_v_ui16_queue_idx_x.resize(m_i_col_size*m_i_row_size);
    m_v_ui16_queue_idx_y.clear();
    m_v_ui16_queue_idx_y.resize(m_i_col_size*m_i_row_size);
}

void RangePmc::DempsterCombination(EvidType* EvidSrc, EvidType* EvidOther) {
    // Common denominator
    EvidSqaureType CommDen = (EvidSqaureType)MAX_EVIDENCE * MAX_EVIDENCE -
                             (EvidSqaureType)EvidSrc[0] * (EvidSqaureType)EvidOther[1] -
                             (EvidSqaureType)EvidSrc[1] * (EvidSqaureType)EvidOther[0];

    if (CommDen == 0) {
        EvidSrc[2] = (EvidType)MAX_EVIDENCE;
        EvidSrc[1] = 0;
        EvidSrc[0] = 0;
        return;
    }

    EvidSqaureType tmp_0 = ((EvidSqaureType)EvidSrc[0] * (EvidSqaureType)EvidOther[2] +
                            (EvidSqaureType)EvidSrc[2] * (EvidSqaureType)EvidOther[0] +
                            (EvidSqaureType)EvidSrc[0] * (EvidSqaureType)EvidOther[0]) *
                           (EvidSqaureType)MAX_EVIDENCE / CommDen;

    EvidSqaureType tmp_1 = ((EvidSqaureType)EvidSrc[1] * (EvidSqaureType)EvidOther[2] +
                            (EvidSqaureType)EvidSrc[2] * (EvidSqaureType)EvidOther[1] +
                            (EvidSqaureType)EvidSrc[1] * (EvidSqaureType)EvidOther[1]) *
                           (EvidSqaureType)MAX_EVIDENCE / CommDen;

    EvidSqaureType tmp_2 = (EvidSqaureType)MAX_EVIDENCE - tmp_1 - tmp_0;

    EvidSrc[0] = (EvidType)tmp_0;
    EvidSrc[1] = (EvidType)tmp_1;
    EvidSrc[2] = (EvidType)tmp_2;
}


// Range Image의 rot, transl를 입력받아 point의 world상의 해당 range image 상의 point_soph 값 반환
void RangePmc::SphericalProjection(point_soph &p, int range_index, const M3D &rot, const V3D &transl, point_soph &p_spherical)
{
    V3D p_projected(rot.inverse()*(p.glob - transl)); // 현재 range image 좌표계로 이전됨 p의 위치 

    // GetVec을 통해 vec과 ind, position이 할당됨
    p_spherical.GetVec(p_projected, m_params.f_horizontal_resolution, m_params.f_vertical_resolution);
}

bool RangePmc::KeyFrameCheck(double cur_time, M3D rot, V3D transl)
{
    if(m_range_image_ptr_list.size() == 0) return true;

    Eigen::Matrix3d relative_rotation =  m_range_image_ptr_list.back()->project_R.transpose() * rot;
    Eigen::AngleAxisd angle_axis(relative_rotation);

    std::cout.precision(15);
    std::cout<<"cur_time: "<<cur_time<<" rangetime: "<<m_range_image_ptr_list.back()->time<<std::endl;

    if(cur_time - m_range_image_ptr_list.back()->time > m_params.f_min_key_frame_time ||
       (m_range_image_ptr_list.back()->project_T - transl).norm() > m_params.f_min_key_frame_rot ||
       (angle_axis.angle() > m_params.f_min_key_frame_rot))
    {
        return true;
    }

    return false;
}

inline float RangePmc::GaussianWeight(double value, double sigma)
{   
    // return - value / (sigma * 2.5) + 1.0;

    // return (float)exp(-0.5 * pow((p1-p2).norm()/m_params.f_gaussian_sigma, 2));
    return (float)(exp(-0.5 * pow(value/sigma, 2)) + m_params.f_sigma_epsilon) / (1.0 + m_params.f_sigma_epsilon);
}

int RangePmc::FindIndexInVector(const std::vector<int>& vec, int value) {
    auto it = std::find(vec.begin(), vec.end(), value);
    
    // 값이 벡터 내에 존재한다면, 인덱스 반환
    if (it != vec.end()) {
        // 'it - vec.begin()'를 통해 인덱스 계산
        return std::distance(vec.begin(), it);
    }
    // 값이 벡터 내에 존재하지 않는다면, -1 반환
    return -1;
}

// 해당 Range Image Vertical FOV 안에 있는 포인트 인지 확인
bool RangePmc::CheckVerFoV(const point_soph & p, const RangeImage &image_info)
{
    bool b_ver_up = false, b_ver_down = false;
    for(int i = p.ver_ind; i >= m_i_pixel_fov_down; i--) // 현재 포인트부터 image 하단까지 유효 pixel 있는지 확인
    {   
        int cur_pos = p.hor_ind * MAX_1D_HALF + i;
        if(image_info.range_image[cur_pos].size() > 0)
        {
            b_ver_down = true;
            break;
        }
    } 
    for(int i = p.ver_ind; i <= m_i_pixel_fov_up; i++) // 현재 포인트부터 image 상단까지 유효 pixel 있는지 확인
    {   
        int cur_pos = p.hor_ind * MAX_1D_HALF + i;
        if(image_info.range_image[cur_pos].size() > 0)
        {
            b_ver_up = true;
            break;
        }
    }   

    if(b_ver_up && b_ver_down) return false;
    else return true;
}

bool RangePmc::CheckNeighbor(const point_soph & p, const RangeImage &image_info, float &max_range, float &min_range)
{   
    int neighbor_range = m_params.i_neighbor_pixel_max;

    // TODO Adaptive
    // if(p.incident < 30 * M_PI/180.0){
    //     neighbor_range = 1;
    //     i_small_incident++;
    // }
    // else{
    //     i_total_incident++;
    // }
    if(image_info.incident_all[p.position] < 30 * M_PI/180.0){
        neighbor_range = 2;
        i_small_incident++;
    }
    else{
        i_total_incident++;
    }

    bool is_valid = false;
    for (int i = -neighbor_range; i <= neighbor_range; i++)
    {
        for (int j = -neighbor_range; j <= neighbor_range; j++)
        {
            int cur_pos = (p.hor_ind + i) * MAX_1D_HALF + p.ver_ind + j;
            if(cur_pos < MAX_2D_N && cur_pos >= 0 && image_info.range_image[cur_pos].size() > 0)
            {
                float cur_max_range = image_info.max_range_all[cur_pos];
                float cur_min_range = image_info.min_range_all[cur_pos];

                if(cur_min_range > 10E-5)
                    is_valid = true;

                if(min_range > 10E-5){
                    min_range = std::min(cur_min_range, min_range);
                }
                else{
                    min_range = cur_min_range;
                } 

                if(max_range > 10E-5){
                    max_range = std::max(cur_max_range, max_range);
                }
                else{
                    max_range = cur_max_range;
                } 

            }
        }
    }

    return is_valid;
}

void RangePmc::OutputPmcPC(const std::vector<point_soph*> &points, const RangeImage::Ptr range_image_ptr)
{   
    double tout_start = omp_get_wtime();

    int i_points_size = points.size(); // point num

    int p_r, p_g, p_b;

    // PMC pcptr
    // for(int i = 0; i < i_points_size; i++){
    //     pcl::PointXYZRGB po;
    //     po.x = points[i]->local[0];
    //     po.y = points[i]->local[1];
    //     po.z = points[i]->local[2]; 

    //     // 현재 포인트의 거리가 최소거리보다 크다면 (대표값이 아니라면)
    //     if(m_params.b_output_min_range == true && 
    //        points[i]->vec(2) > range_image_ptr->min_range_all[points[i]->position] + 10E-5)
    //         continue;

    //     if(m_params.b_output_static_point == true && 
    //        points[i]->dyn != STATIC)
    //         continue;
        
    //     switch(points[i]->dyn)
    //     {
    //         case STATIC: // Red
    //             po.r = 255;
    //             po.g = 0;
    //             po.b = 0;
    //             break;
    //         case CASE1: // Green
    //             po.r = 0;
    //             po.g = 255;
    //             po.b = 0;
    //             break;
    //         default: // Blue. UNKNOWN, INVALID
    //             po.r = 0;
    //             po.g = 0;
    //             po.b = 255;
    //     }

    //     m_pmc_xyzrgb_pcptr->push_back(po);
    // }



    for(int i_ver_idx = m_i_pixel_fov_down; i_ver_idx < m_i_pixel_fov_up; i_ver_idx++) {
        for(int i_hor_idx = m_i_pixel_fov_right; i_hor_idx < m_i_pixel_fov_left; i_hor_idx++) {

            int iter_pos = i_hor_idx * MAX_1D_HALF + i_ver_idx;
            if(range_image_ptr->range_image[iter_pos].size() > 0){
                if(m_params.b_output_min_range == true){
                    int min_range_idx = range_image_ptr->min_range_index_all[iter_pos];
                    pcl::PointXYZRGB po;
                    po.x = range_image_ptr->range_image[iter_pos][min_range_idx]->local[0];
                    po.y = range_image_ptr->range_image[iter_pos][min_range_idx]->local[1];
                    po.z = range_image_ptr->range_image[iter_pos][min_range_idx]->local[2];

                    int r,g,b;

                    // switch (range_image_ptr->range_image[iter_pos][min_range_idx]->dyn)
                    // {
                    //     case STATIC: // Red
                    //         po.r = 255;
                    //         po.g = 0;
                    //         po.b = 0;
                    //         break;
                    //     case CASE1: // Green
                    //         po.r = 0;
                    //         po.g = 255;
                    //         po.b = 0;
                    //         break;
                    //     default: // Blue. UNKNOWN, INVALID
                    //         po.r = 0;
                    //         po.g = 0;
                    //         po.b = 255;
                    // }

                    // p_r = range_image_ptr->range_image[iter_pos][min_range_idx]->r;
                    // p_g = range_image_ptr->range_image[iter_pos][min_range_idx]->g;
                    // p_b = range_image_ptr->range_image[iter_pos][min_range_idx]->b;
                    // if(p_b > max(p_r, p_g)){
                    //     po.r = 0;
                    //     po.g = 0;
                    //     po.b = 255;
                    // }
                    // else if (p_g > p_r){
                    //     po.r = 0;
                    //     po.g = 255;
                    //     po.b = 0;               
                    // }
                    // else{
                    //     po.r = 255;
                    //     po.g = 0;
                    //     po.b = 0;
                    // }   

                    po.r = range_image_ptr->range_image[iter_pos][min_range_idx]->r;
                    po.g = range_image_ptr->range_image[iter_pos][min_range_idx]->g;
                    po.b = range_image_ptr->range_image[iter_pos][min_range_idx]->b;

                    // p_r = range_image_ptr->xyzrgb_cloud_pcptr->points[iter_pos].r;
                    // p_g = range_image_ptr->xyzrgb_cloud_pcptr->points[iter_pos].g;
                    // p_b = range_image_ptr->xyzrgb_cloud_pcptr->points[iter_pos].b;
                    // if(p_b > max(p_r, p_g)){
                    //     po.r = 0;
                    //     po.g = 0;
                    //     po.b = 255;
                    // }
                    // else if (p_g > p_r){
                    //     po.r = 0;
                    //     po.g = 255;
                    //     po.b = 0;               
                    // }
                    // else{
                    //     po.r = 255;
                    //     po.g = 0;
                    //     po.b = 0;
                    // }

                    // p_r = range_image_ptr->range_image[iter_pos][min_range_idx]->static_count;
                    // p_g = range_image_ptr->range_image[iter_pos][min_range_idx]->dynamic_count;
                    // p_b = range_image_ptr->range_image[iter_pos][min_range_idx]->unknown_count;
                    // if(p_b > max(p_r, p_g)){
                    //     po.r = 0;
                    //     po.g = 0;
                    //     po.b = 255;
                    // }
                    // else if (p_g > p_r){
                    //     po.r = 0;
                    //     po.g = 255;
                    //     po.b = 0;               
                    // }
                    // else{
                    //     po.r = 255;
                    //     po.g = 0;
                    //     po.b = 0;
                    // }

                    m_pmc_xyzrgb_pcptr->push_back(po);
                    
                }
                else{
                    for(int pixel_idx = 0 ; pixel_idx < range_image_ptr->range_image[iter_pos].size() ; pixel_idx++){
                        pcl::PointXYZRGB po;
                        po.x = range_image_ptr->range_image[iter_pos][pixel_idx]->local[0];
                        po.y = range_image_ptr->range_image[iter_pos][pixel_idx]->local[1];
                        po.z = range_image_ptr->range_image[iter_pos][pixel_idx]->local[2];

                        int r,g,b;
                        switch (range_image_ptr->range_image[iter_pos][pixel_idx]->dyn)
                        {
                            case STATIC: // Red
                                po.r = 255;
                                po.g = 0;
                                po.b = 0;
                                break;
                            case CASE1: // Green
                                po.r = 0;
                                po.g = 255;
                                po.b = 0;
                                break;
                            default: // Blue. UNKNOWN, INVALID
                                po.r = 0;
                                po.g = 0;
                                po.b = 255;
                        }
                        m_pmc_xyzrgb_pcptr->push_back(po);
                    }
                }

            }

        }
    }

    std::cout<<"output pc num: "<<m_pmc_xyzrgb_pcptr->points.size()<<std::endl;

    // Key frame pcptr
    if(m_b_key_frame == true){
        m_range_image_xyzrgb_pcptr->clear();
        
        m_deque_range_image_xyzrgb_pcptr[m_i_cur_point_soph_pointers]->clear();

        for(int i = 0; i < i_points_size; i++){
            pcl::PointXYZRGB po;
            po.x = points[i]->glob[0];
            po.y = points[i]->glob[1];
            po.z = points[i]->glob[2];

            if(m_params.b_output_static_point == true &&
               points[i]->dyn != STATIC)
                continue;

            switch(points[i]->dyn)
            {
                case STATIC: // Red
                    po.r = 255;
                    po.g = 0;
                    po.b = 0;
                    break;
                case CASE1: // Green
                    po.r = 0;
                    po.g = 255;
                    po.b = 0;
                    break;
                default: // Blue. UNKNOWN, INVALID
                    po.r = 0;
                    po.g = 0;
                    po.b = 255;
            }

            m_deque_range_image_xyzrgb_pcptr[m_i_cur_point_soph_pointers]->push_back(po);
        }


        for(auto range_image_xyzrgb_iter : m_deque_range_image_xyzrgb_pcptr){
            *m_range_image_xyzrgb_pcptr += *range_image_xyzrgb_iter;
        }
    }

    // Cluster pcptr

    std::vector<std::vector<int>> sorted_cluster_poses_vec = range_image_ptr->cluster_poses_vec;
    std::sort(sorted_cluster_poses_vec.begin(), sorted_cluster_poses_vec.end(), by_size_decent());
    m_cluster_xyzrgb_pcptr->clear();

    int cluster_num = sorted_cluster_poses_vec.size();
    const int MAX_COLOR_VALUE = 255;
    int i_hor_idx, i_ver_idx;

    for(int cluster_idx = 0; cluster_idx < sorted_cluster_poses_vec.size(); cluster_idx++)
    {
        int r = 0, g = 0, b = 0;

        // 무지개 색상의 비율을 계산합니다.
        float ratio = (float)cluster_idx / (cluster_num - 1);

        if (ratio < 0.2) { // 빨간색에서 주황색으로
            r = MAX_COLOR_VALUE;
            g = static_cast<int>(MAX_COLOR_VALUE * (ratio / 0.2));
            b = 0;
        } else if (ratio < 0.4) { // 주황색에서 노란색으로
            r = MAX_COLOR_VALUE;
            g = MAX_COLOR_VALUE;
            b = 0;
        } else if (ratio < 0.6) { // 노란색에서 초록색으로
            r = static_cast<int>(MAX_COLOR_VALUE * ((0.6 - ratio) / 0.2));
            g = MAX_COLOR_VALUE;
            b = 0;
        } else if (ratio < 0.8) { // 초록색에서 파란색으로
            r = 0;
            g = MAX_COLOR_VALUE;
            b = static_cast<int>(MAX_COLOR_VALUE * ((ratio - 0.6) / 0.2));
        } else { // 파란색에서 보라색으로
            r = static_cast<int>(MAX_COLOR_VALUE * ((ratio - 0.8) / 0.2));
            g = 0;
            b = MAX_COLOR_VALUE;
        }
        

        for (auto pos : sorted_cluster_poses_vec[cluster_idx])
        {   
            if(m_params.b_output_min_range == true){
                int min_range_pos = range_image_ptr->min_range_index_all[pos];
                if(min_range_pos >= 0 ){
                    pcl::PointXYZRGB po;
                    po.x = range_image_ptr->range_image[pos][min_range_pos]->local.x(); 
                    po.y = range_image_ptr->range_image[pos][min_range_pos]->local.y(); 
                    po.z = range_image_ptr->range_image[pos][min_range_pos]->local.z(); 
                    po.r = r;
                    po.g = g;
                    po.b = b;

                    m_cluster_xyzrgb_pcptr->push_back(po);    
                }
            }
            else{
                for (auto point_in_pixel : range_image_ptr->range_image[pos])
                {
                    pcl::PointXYZRGB po;
                    po.x = point_in_pixel->local.x(); 
                    po.y = point_in_pixel->local.y(); 
                    po.z = point_in_pixel->local.z(); 
                    po.r = r;
                    po.g = g;
                    po.b = b;

                    m_cluster_xyzrgb_pcptr->push_back(po);
                }
            }
        }
    }

    double tout_end = omp_get_wtime() - tout_start;
    std::cout<<"[Range PMC] Output Time: "<< tout_end*1000.0 << " ms"<<std::endl;
}

void RangePmc::OutputRangeImagesPc()
{

}

cv::Mat RangePmc::GetRangeImageCv()
{
    cv::Mat mat_range_img((int)(m_i_pixel_fov_up - m_i_pixel_fov_down), (int)(m_i_pixel_fov_left - m_i_pixel_fov_right), CV_32F, cv::Scalar::all(FLT_MAX));

    if(m_range_image_ptr_list.size()>0){
        for(int i_ver_idx = m_i_pixel_fov_down; i_ver_idx < m_i_pixel_fov_up; i_ver_idx++) {
            for(int i_hor_idx = m_i_pixel_fov_right; i_hor_idx < m_i_pixel_fov_left; i_hor_idx++) {

                int iter_pos = i_hor_idx * MAX_1D_HALF + i_ver_idx;
                mat_range_img.at<float>(m_i_pixel_fov_up - i_ver_idx - 1, m_i_pixel_fov_left - i_hor_idx - 1) = m_range_image_ptr_list.back()->min_range_all[iter_pos];
            }
        }
    }

    // 채워진 cv::Mat 객체를 반환합니다.
    return mat_range_img;
}

cv::Mat RangePmc::GetDynamicImageCv()
{
    cv::Mat mat_dynamic_img((int)(m_i_pixel_fov_up - m_i_pixel_fov_down), (int)(m_i_pixel_fov_left - m_i_pixel_fov_right), CV_8UC3, cv::Scalar::all(0));

    if(m_range_image_ptr_list.size() > 0){
        for(int i_ver_idx = m_i_pixel_fov_down; i_ver_idx < m_i_pixel_fov_up; i_ver_idx++) {
            for(int i_hor_idx = m_i_pixel_fov_right; i_hor_idx < m_i_pixel_fov_left; i_hor_idx++) {

                int iter_pos = i_hor_idx * MAX_1D_HALF + i_ver_idx;
                if(m_range_image_ptr_list.back()->range_image[iter_pos].size() > 0){
                    int min_range_idx = m_range_image_ptr_list.back()->min_range_index_all[iter_pos];

                    int r,g,b;
                    switch (m_range_image_ptr_list.back()->range_image[iter_pos][min_range_idx]->dyn)
                    {
                        case STATIC: // Red
                            r = 255;
                            g = 0;
                            b = 0;
                            break;
                        case CASE1: // Green
                            r = 0;
                            g = 255;
                            b = 0;
                            break;
                        default: // Blue
                            r = 0;
                            g = 0;
                            b = 255;
                    }

                    mat_dynamic_img.at<cv::Vec3b>(m_i_pixel_fov_up - i_ver_idx - 1, m_i_pixel_fov_left - i_hor_idx - 1) = cv::Vec3b(r,g,b);
                }
                else{
                    mat_dynamic_img.at<cv::Vec3b>(m_i_pixel_fov_up - i_ver_idx - 1, m_i_pixel_fov_left - i_hor_idx - 1) = cv::Vec3b(0,0,0); // black
                }
            }
        }
    }

    // 채워진 cv::Mat 객체를 반환합니다.
    return mat_dynamic_img;
}

cv::Mat RangePmc::GetIncidentImageCv()
{
    cv::Mat mat_incident_img((int)(m_i_pixel_fov_up - m_i_pixel_fov_down), (int)(m_i_pixel_fov_left - m_i_pixel_fov_right), CV_32F, cv::Scalar::all(-1));

    if(m_range_image_ptr_list.size()>0){
        for(int i_ver_idx = m_i_pixel_fov_down; i_ver_idx < m_i_pixel_fov_up; i_ver_idx++) {
            for(int i_hor_idx = m_i_pixel_fov_right; i_hor_idx < m_i_pixel_fov_left; i_hor_idx++) {
                
                int iter_pos = i_hor_idx * MAX_1D_HALF + i_ver_idx;
                if(m_range_image_ptr_list.back()->range_image[iter_pos].size() > 0){
                    int min_range_idx = m_range_image_ptr_list.back()->min_range_index_all[iter_pos];
                    mat_incident_img.at<float>(m_i_pixel_fov_up - i_ver_idx - 1, m_i_pixel_fov_left - i_hor_idx - 1) = 
                                m_range_image_ptr_list.back()->range_image[iter_pos][min_range_idx]->incident;

                }
            }
        }
    }

    // 채워진 cv::Mat 객체를 반환합니다.
    return mat_incident_img;
}


cv::Mat RangePmc::GetClusterImageCv()
{
    cv::Mat mat_cluster_img((int)(m_i_pixel_fov_up - m_i_pixel_fov_down), (int)(m_i_pixel_fov_left - m_i_pixel_fov_right), CV_8UC3, cv::Scalar::all(0));

    if(m_range_image_ptr_list.size()>0){
        if(m_range_image_ptr_list.back()->cluster_poses_vec.size() == 0 ) return mat_cluster_img;
    
        std::vector<std::vector<int>> sorted_cluster_poses_vec = m_range_image_ptr_list.back()->cluster_poses_vec;
        std::sort(sorted_cluster_poses_vec.begin(), sorted_cluster_poses_vec.end(), by_size_decent());

        int cluster_num = sorted_cluster_poses_vec.size();
        const int MAX_COLOR_VALUE = 255;
        int i_hor_idx, i_ver_idx;

        for(int cluster_idx = 0; cluster_idx < sorted_cluster_poses_vec.size(); cluster_idx++)
        {
            int r = 0, g = 0, b = 0;

            // 무지개 색상의 비율을 계산합니다.
            float ratio = (float)cluster_idx / (cluster_num - 1);

            if (ratio < 0.2) { // 빨간색에서 주황색으로
                r = MAX_COLOR_VALUE;
                g = static_cast<int>(MAX_COLOR_VALUE * (ratio / 0.2));
                b = 0;
            } else if (ratio < 0.4) { // 주황색에서 노란색으로
                r = MAX_COLOR_VALUE;
                g = MAX_COLOR_VALUE;
                b = 0;
            } else if (ratio < 0.6) { // 노란색에서 초록색으로
                r = static_cast<int>(MAX_COLOR_VALUE * ((0.6 - ratio) / 0.2));
                g = MAX_COLOR_VALUE;
                b = 0;
            } else if (ratio < 0.8) { // 초록색에서 파란색으로
                r = 0;
                g = MAX_COLOR_VALUE;
                b = static_cast<int>(MAX_COLOR_VALUE * ((ratio - 0.6) / 0.2));
            } else { // 파란색에서 보라색으로
                r = static_cast<int>(MAX_COLOR_VALUE * ((ratio - 0.8) / 0.2));
                g = 0;
                b = MAX_COLOR_VALUE;
            }
            

            for (auto pos : sorted_cluster_poses_vec[cluster_idx])
            {
                i_hor_idx = pos / MAX_1D_HALF;
                i_ver_idx = pos % MAX_1D_HALF;

                mat_cluster_img.at<cv::Vec3b>(m_i_pixel_fov_up - i_ver_idx - 1, m_i_pixel_fov_left - i_hor_idx - 1) = cv::Vec3b(r,g,b);
            }

            
        }
    }

    return mat_cluster_img;
}

cv::Mat RangePmc::GetGroundImageCv()
{
    cv::Mat mat_ground_img((int)(m_i_pixel_fov_up - m_i_pixel_fov_down), (int)(m_i_pixel_fov_left - m_i_pixel_fov_right), CV_8U, cv::Scalar::all(0));

    int ground_count = 0;
    int non_ground_count = 0;
    if(m_range_image_ptr_list.size()>0){
        for(int i_ver_idx = m_i_pixel_fov_down; i_ver_idx < m_i_pixel_fov_up; i_ver_idx++) {
            for(int i_hor_idx = m_i_pixel_fov_right; i_hor_idx < m_i_pixel_fov_left; i_hor_idx++) {
                
                int iter_pos = i_hor_idx * MAX_1D_HALF + i_ver_idx;
                if(m_range_image_ptr_list.back()->range_image[iter_pos].size() > 0){
                    int min_range_idx = m_range_image_ptr_list.back()->min_range_index_all[iter_pos];

                    if(m_range_image_ptr_list.back()->range_image[iter_pos][min_range_idx]->ground == true){
                        mat_ground_img.at<uint8_t>(m_i_pixel_fov_up - i_ver_idx - 1, m_i_pixel_fov_left - i_hor_idx - 1) = (uint8_t)2;
                        ground_count++;
                    }
                    else{
                        mat_ground_img.at<uint8_t>(m_i_pixel_fov_up - i_ver_idx - 1, m_i_pixel_fov_left - i_hor_idx - 1) = (uint8_t)1;
                        non_ground_count++;
                    }
                        
                }
            }
        }
    }

    std::cout<<"GetGroundImageCv ground: "<<ground_count<<" non: "<<non_ground_count<<std::endl;

    return mat_ground_img;
}


void RangePmc::GetFilteredPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& o_pmc_xyzrgb_pcptr)
{   
    *o_pmc_xyzrgb_pcptr = *m_pmc_xyzrgb_pcptr;
}

void RangePmc::GetKeyFramePoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& o_key_frame_xyzrgb_pcptr)
{   
    *o_key_frame_xyzrgb_pcptr = *m_range_image_xyzrgb_pcptr;
}

void RangePmc::GetClusterPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& o_cluster_xyzrgb_pcptr)
{   
    *o_cluster_xyzrgb_pcptr = *m_cluster_xyzrgb_pcptr;
}


