#include "gaussian_point_motion_classification.hpp"

GaussianPointMotionClassi::GaussianPointMotionClassi(){};
GaussianPointMotionClassi::~GaussianPointMotionClassi(){};

void GaussianPointMotionClassi::Init(gs_pmc_params params){
    cout<<"[Gaussian PMC] Init Started"<<endl;

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

    m_i_pixel_fov_up    = floor((m_params.f_fov_up   / 180.0*M_PI + 0.5 * M_PI)  / m_params.f_vertical_resolution);
    m_i_pixel_fov_down  = floor((m_params.f_fov_down / 180.0*M_PI + 0.5 * M_PI)  / m_params.f_vertical_resolution);
    m_i_pixel_fov_left  = floor((m_params.f_fov_left / 180.0*M_PI +  M_PI)       / m_params.f_horizontal_resolution);
    m_i_pixel_fov_right = floor((m_params.f_fov_right/ 180.0*M_PI +  M_PI)       / m_params.f_horizontal_resolution);

    m_i_row_size        = m_i_pixel_fov_up - m_i_pixel_fov_down;
    m_i_col_size        = m_i_pixel_fov_left - m_i_pixel_fov_right;
    m_point_soph_pointers.reserve(m_params.i_max_range_image_num); 


    // m_i_max_1d = ceil(2 * M_PI / m_params.f_horizontal_resolution);
    m_i_max_1d = (m_i_pixel_fov_left - m_i_pixel_fov_right);
    // m_i_max_1d_half = ceil(M_PI / m_params.f_vertical_resolution);
    m_i_max_1d_half = (m_i_pixel_fov_up - m_i_pixel_fov_down);

    // m_i_max_2d_n = m_i_max_1d * m_i_max_1d_half;    
    m_i_max_2d_n =  (m_i_pixel_fov_up - m_i_pixel_fov_down) * (m_i_pixel_fov_left - m_i_pixel_fov_right);

    m_i_max_point =  (m_i_pixel_fov_up - m_i_pixel_fov_down) * (m_i_pixel_fov_left - m_i_pixel_fov_right) * m_params.i_max_points_in_pixel;

    cout<<"m_i_pixel_fov_up: "          << m_i_pixel_fov_up<<endl;
    cout<<"m_i_pixel_fov_down: "        << m_i_pixel_fov_down<<endl;
    cout<<"m_i_pixel_fov_left: "        << m_i_pixel_fov_left<<endl;
    cout<<"m_i_pixel_fov_right: "       << m_i_pixel_fov_right<<endl;

    cout<<"m_i_max_1d: "                << m_i_max_1d<<endl;
    cout<<"m_i_max_1d_half: "           << m_i_max_1d_half<<endl;
    cout<<"m_i_max_2d_n: "              << m_i_max_2d_n<<endl;
    cout<<"m_i_max_point: "             << m_i_max_point<<endl;


    m_b_key_frame = false;

    for (int i = 0; i < m_params.i_max_range_image_num; i++)
    {
        point_soph* p = new point_soph[m_i_max_point]; // 비어있는 point_soph 배열 생성 
        m_point_soph_pointers.push_back(p); // 메모리 할당 
    }

    m_temp_point_soph_pointer = new point_soph[m_i_max_point];

    for (int i = 0; i < m_params.i_max_range_image_num; i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr iter_ptr;
        iter_ptr.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        m_deque_range_image_xyzrgb_pcptr.push_back(iter_ptr);
    }

    m_pmc_xyzrgb_pcptr.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    m_range_image_xyzrgb_pcptr.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    m_cluster_xyzrgb_pcptr.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

    m_tbb_dynamic_points_pcptr.reset(new pcl::PointCloud<pcl::PointXYZRGBL>());
    m_tbb_dynamic_points_pcptr->points.resize(m_i_max_point + 1);

    m_tbb_points_normal_pcptr.reset(new pcl::PointCloud<PointType>());
    m_tbb_dynamic_points_normal_pcptr.reset(new pcl::PointCloud<PointType>());
    // m_tbb_dynamic_points_normal_pcptr->points.resize(m_i_max_point + 1);

    // hor, ver
    NeighborAssign(2, 3);
    
    cout<<"[Gaussian PMC] Init Done"<<endl;
}


void GaussianPointMotionClassi::Filter(PointCloudXYZI::Ptr feats_undistort, const M3D & rot_end, const V3D & pos_end, const double & scan_end_time)
{   
    double t00 = omp_get_wtime();
    cout<<"[Gaussian PMC] Filter Started"<<endl;

    m_b_key_frame = KeyFrameCheck(scan_end_time, rot_end, pos_end);

    
    int size = feats_undistort->points.size(); // point num
    if(size > m_i_max_point){ // 최대 인풋 포인트 수 제한
        std::cout<<"[Filter] Input Point is limited!!! : "<< size <<" > "<< m_i_max_point<<std::endl;
        size = m_i_max_point;
    } 

    std::vector<int> index(size); // Point Index 할당 
    for (int i = 0; i < size; i++) {
        index[i] = i;
    }

    std::cout<<"[Filter] Input Size: "<<size<<std::endl;


    point_soph* p;  // point_soph 포인트 배열 value
    if(m_b_key_frame == true){
        p = m_point_soph_pointers[m_i_cur_point_soph_pointers]; // 현재 range image내  point_soph들 메모리 불러옴. 초기화 할 것임 
    }
    else{
        p = m_temp_point_soph_pointer; // 임시 포인터. point_soph의 array임
    }

    vector<point_soph*> vec_point_ptr; // point_soph의 주소들을 저장하기 위한 벡터.
    // m_detector code에서 m_point_soph_pointers에 값을 고정적으로 저장하고, 다음 인덱스로 넘어가서 Case를 수행 하는 동안에
    // vec_point_ptr가 이전 인덱스를 여전히 가리키면서 range image generation을 할 수 있기 때문이다.

    vec_point_ptr.reserve(size);
    vec_point_ptr.resize(size);
    
    tbb::parallel_for(0, size, 1, [&](int i) {
       // 한 range image 내부를 iteration
        p[i].reset();     
        V3D p_body(feats_undistort->points[i].x, feats_undistort->points[i].y, feats_undistort->points[i].z); // 라이다 기준 point 위치
        V3D p_glob(rot_end * (p_body) + pos_end); // global point 위치 

        p[i].ind = i; // 포인트 고유 인덱스

        p[i].glob = p_glob;        
        p[i].dyn = UNKNOWN;
        // p[i].sensor_rot = rot_end;

        p[i].sensor_transl = pos_end;
        p[i].time = scan_end_time;
        p[i].local = p_body;
        // p[i].intensity = feats_undistort->points[i].intensity;
        p[i].ground = false;
        p[i].propagated = false;
        p[i].occluded_by_previous = false;
        p[i].valid = false;
        p[i].key_point = true; // TBD
        p[i].fp_dynamic = false;
        p[i].incident = -1.0;
        p[i].cluster_ind = -1;

        p[i].r = 0;
        p[i].g = 0;
        p[i].b = 255;

        vec_point_ptr[i] = &p[i]; // p[i] 의 주소를 저장함
        // vec_point_ptr를 통해 접근한 정보를 수정하면 new_image_pointer의 정보도 수정됨 
        // p[i]의 valuse를 수정해도 new_image_pointer의 정보 수정됨
    });
    

    double time_init = omp_get_wtime() - t00;
    std::cout<<"[Gaussian PMC] Init Time: "<< time_init*1000.0 << " ms"<<std::endl;

    // 1. Pre processing

    double time_new_range_image_start = omp_get_wtime();
    RangeImage::Ptr new_image_pointer;
    // key frame이면 현재 프레임은 바로 m_range_image_ptr_list에 push back 됨. 물론 비어있겠지만
    if(m_b_key_frame == true){
        if(m_range_image_ptr_list.size() < m_params.i_max_range_image_num){
            m_i_map_index++;
            new_image_pointer.reset(new RangeImage(rot_end, pos_end, scan_end_time, m_i_map_index));
            m_range_image_ptr_list.push_back(new_image_pointer);
        }
        else{
            m_i_map_index++;
            m_range_image_ptr_list.front()->Reset(rot_end, pos_end, scan_end_time, m_i_map_index);
            new_image_pointer = m_range_image_ptr_list.front();
            m_range_image_ptr_list.pop_front();
            m_range_image_ptr_list.push_back(new_image_pointer); // 벡터의 요소가 가리키는 포인터 메모리 교체 
        }
        m_i_cur_point_soph_pointers = (m_i_cur_point_soph_pointers + 1) % m_params.i_max_range_image_num; // 다음 번 range image 의 index를 지정함 
    }
    else{
        if (!m_temp_range_image_ptr){
            m_temp_range_image_ptr.reset(new RangeImage(rot_end, pos_end, scan_end_time, m_i_map_index));
        }
        else{
            m_temp_range_image_ptr->Reset(rot_end, pos_end, scan_end_time, m_i_map_index);
        }

        new_image_pointer = m_temp_range_image_ptr;
    }
    
    double time_new_range_image = omp_get_wtime() - time_new_range_image_start;
    std::cout<<"[Gaussian PMC] Range Memory Time: "<< time_new_range_image*1000.0 << " ms"<<std::endl;

    // GenerateRangeImage(vec_point_ptr, scan_end_time, rot_end, pos_end, new_image_pointer);
    GenerateRangeImageParallel(vec_point_ptr, scan_end_time, rot_end, pos_end, new_image_pointer);
    if(m_params.b_ground_filtering == true){

        // 각 column 별로 tbb
        GroundSegmentation(new_image_pointer);

        // FIXME: 4/5 교수님 아이디어. Segmentation을 Propagation에서 수행하기
        // if(m_params.b_run_region_growing)
        //     ObjectSegmentation(new_image_pointer);
    }

    // 2. Point Motion Classification
    // 1차 for문. 키 포인트에 대한 case1 case2 수행
    // 병렬처리 가능
    double time_first_for_start = omp_get_wtime();
    tbb::parallel_for(0, size, 1, [&](int i) {
        // 벗어난 포인트 제외
        if(p[i].position < 0 || p[i].position >= m_i_max_2d_n || p[i].dyn == INVALID) return;

        // Ground point에 대해서 r 확률 미리 부여
        if (p[i].ground == true){
            p[i].r = 200; p[i].g = 0; p[i].b = 55;
        }

        Case1(p[i], new_image_pointer);
        if(m_params.b_run_case_2 && p[i].occluded_by_previous && p[i].dyn == UNKNOWN){
            Case2(p[i], new_image_pointer);
        }
    });

    StaticRegionGrowing(vec_point_ptr, new_image_pointer);

    double time_first_for = omp_get_wtime() - time_first_for_start;
    std::cout<<"[Gaussian PMC] Motion Classi Time: "<< time_first_for*1000.0 << " ms"<<std::endl;


    // 3. Post Processing

    // Small False positive 제거
    if(m_params.b_run_small_dynamic_filtering)
        SmallDynamicFiltering(vec_point_ptr, new_image_pointer);

    // Dynamic 근처의 Ground(static) point --> Dynamic
    if(m_params.b_run_region_growing){
        // ImageRegionGrowing(vec_point_ptr, new_image_pointer);
        RegionGrowingWithCluster(vec_point_ptr, new_image_pointer);
    }


    // Output
    OutputPmcPC(vec_point_ptr, new_image_pointer);

    time_total = omp_get_wtime() - t00;
    std::cout<<"[Gaussian PMC] Total Time: "<< time_total*1000.0 << " ms"<<std::endl;
}

void GaussianPointMotionClassi::Case1(point_soph & p, RangeImage::Ptr range_image_ptr)
{
    int i_range_image_num = m_range_image_ptr_list.size();
    int i_valid_image = 0;
    int i_last_image_ind;
    float f_gaussian_weight;

    uint8_t p_r, p_g, p_b;

    i_last_image_ind = i_range_image_num - 2;

    dyn_obj_flg p_dyn = UNKNOWN;

    for (int i = i_last_image_ind; i >= 0; i--) // point 한개에 대해 range image 탐색. 최근-1 range image 부터 
    {   
        // p에는 glob이 있어야하고, 이 함수를 통해서 vec, hor_ind, ver_ind, position 가 m_range_image_ptr_list[i] 기준으로 바뀜 
        SphericalProjection(p, m_range_image_ptr_list[i]->map_index, m_range_image_ptr_list[i]->project_R, m_range_image_ptr_list[i]->project_T, p);

        if (p.ver_ind > m_i_max_1d_half || p.ver_ind < 0 || 
            p.hor_ind > m_i_max_1d      || p.hor_ind < 0 || p.vec(2) < 0.0f \
            || p.position < 0 || p.position >= m_i_max_2d_n)
        {
            continue; // range image의 경계를 넘은 point는 iter range image와 맞지 않음 skip
        }

        Case1Gaussian(p, *m_range_image_ptr_list[i], p_dyn, p_r, p_g, p_b);

        if( i == i_last_image_ind && p_dyn == OCCLUDED){

            p.occluded_by_previous = true;

            p.occu_index[0] = m_range_image_ptr_list[i]->map_index; // 현재 range image 인덱스 
            p.occu_index[1] = p.position; // 현재 range image에서의 position
        }

        // Dempster
        EvidType SrcEvidence[3] = {(EvidType)p.r, (EvidType)p.g, (EvidType)p.b};
        EvidType MovingEvidence[3] = {(EvidType)p_r, (EvidType)p_g, (EvidType)p_b};
        
        DempsterCombination(SrcEvidence, MovingEvidence);

        p.r = (uint8_t)SrcEvidence[0];
        p.g = (uint8_t)SrcEvidence[1];
        p.b = (uint8_t)SrcEvidence[2];
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

}

// return true if p is occluding previous range images
bool GaussianPointMotionClassi::Case1Enter(const point_soph & p, const RangeImage &image_info, dyn_obj_flg & p_dyn)
{   
    // p는 image_info 기준으로 projection 되어있음
    float f_max_range = 0.0, f_min_range = 0.0;
    float f_cur_max_range = 0.0, f_cur_min_range = 0.0;
    float f_occluded_max_range = 0.0; // neighbor +- 1칸 까지만 서치한 결과. 그래도 못찾으면 f_cur_max_range와 같게 
    p_dyn = UNKNOWN;

    f_occluded_max_range = f_max_range = image_info.max_range_all[p.position];
    f_min_range = image_info.min_range_all[p.position];

    if(p.ver_ind <= m_i_max_1d_half && p.ver_ind >=  0 && \
       p.hor_ind <= m_i_max_1d      && p.hor_ind >= 0 && \
       (m_params.i_vertical_neighbor_pixel > 0 || m_params.i_horizontal_neighbor_pixel > 0 ))
    {
        CheckNeighbor(p, image_info, f_max_range, f_min_range, f_occluded_max_range);
    }
 
    if(f_min_range < 10E-5){ // 비교할 포인트를 찾지 못함 
        p_dyn = UNKNOWN;
        return false;
    }

    float range_threshold_multiply = p.local.norm() / 10.0;
    if(range_threshold_multiply < 1.0) range_threshold_multiply = 1.0;
    range_threshold_multiply = 1.0;
    

    if( f_occluded_max_range < 10E-5 || m_params.b_run_case_2 == false) f_occluded_max_range = f_max_range; // 못찾았거나 case2 안할꺼면 f_cur_max_range와 같게 

    if (p.vec(2) < f_min_range - m_params.f_range_threshold * range_threshold_multiply ){ // if point is closer than min range
        p_dyn = CASE1;
        return true;
    }
    else if (p.vec(2) < f_min_range + m_params.f_range_threshold * range_threshold_multiply){ // if point is Static
        p_dyn = STATIC;
        return true;
    }
    else if (p.vec(2) >= f_min_range + m_params.f_range_threshold * range_threshold_multiply){ 
        p_dyn = OCCLUDED;
        return true;
    } 


    return false;
}

// neighbor에 여러 픽셀이 겹칠 것을 고려, 한 range image에 대해서 p에 대한 dynamic, static, occluded의 확률을 가우시안 합을 통해 계산
bool GaussianPointMotionClassi::Case1Gaussian(const point_soph & p, const RangeImage &image_info, dyn_obj_flg & p_dyn, uint8_t & p_r, uint8_t & p_g, uint8_t & p_b)
{   
    p_r = 0; p_g = 0; p_b = MAX_EVIDENCE;

    float f_r = 0.0f, f_g = 0.0f, f_b = 1.0f;

    int i_point_in_neighbor = 0;
    float f_iter_point_distance;
    float f_cur_point_distance = p.vec(2);
    float f_gaussian_weight, f_dynamic_weight, f_static_weight, f_occluded_weight;

    float f_max_range = 0.0, f_min_range = 0.0;
    float f_occluded_max_range = 0.0; // neighbor +- 1칸 까지만 서치한 결과. 그래도 못찾으면 f_cur_max_range와 같게 

    f_occluded_max_range = f_max_range = image_info.max_range_all[p.position];
    f_min_range = image_info.min_range_all[p.position];

    if(m_params.i_vertical_neighbor_pixel > 0 || m_params.i_horizontal_neighbor_pixel > 0 )
    {
        CheckNeighbor(p, image_info, f_max_range, f_min_range, f_occluded_max_range);
    }

    if(f_min_range < 10E-5){ // 비교할 포인트를 찾지 못함 
        p_dyn = UNKNOWN;
        return false;
    }
    
    f_gaussian_weight = GaussianWeight(p.vec(2) - f_min_range, 0.4);

    if(p.vec(2) > f_min_range){
        // occluded
        f_g = 0.0f;
        f_r = f_gaussian_weight;
        f_b = 1.0f - f_gaussian_weight;
    }
    else{
        // occluding
        f_g = 1.0f - f_gaussian_weight;
        f_r = f_gaussian_weight;
        f_b = 0.0f;
    }

    // f_gaussian_weight = GaussianWeight2(p.vec(2) - f_min_range, m_params.f_gaussian_sigma);

    // if(p.vec(2) > f_min_range){
    //     // occluded
    //     f_r = f_gaussian_weight;
    //     f_g = 0.0f;
    //     f_b = 1.0f - f_r;
    // }
    // else{
    //     // occluding
    //     f_r = f_gaussian_weight;
    //     f_g = 1.0f/(m_params.f_gaussian_sigma*2.5066) - f_gaussian_weight;
    //     f_b = 1.0f - f_r - f_g;
    // }

    p_r = (uint8_t)(f_r * MAX_EVIDENCE);
    p_g = (uint8_t)(f_g * MAX_EVIDENCE);
    p_b = (uint8_t)MAX_EVIDENCE - (p_r + p_g);

    if(p_r > p_g && p_r > p_b) p_dyn = STATIC;
    else if(p_g > p_r && p_g > p_b) p_dyn = CASE1;
    else p_dyn = OCCLUDED;

    return true;
}

void GaussianPointMotionClassi::Case2(point_soph & p, RangeImage::Ptr range_image_ptr)
{   
    // 이전 range image에 의해 가려진것이 아니라면 스킵
    if(p.occluded_by_previous == false) return;
    if(m_range_image_ptr_list.size() <= 2 ) return;

    int iter_map_index; 
    int min_pos, max_pos;
    int i_valid_range_num = 0;
    float f_cur_moving_evidence; // 연속적으로 이어진 range image의 비율. 0~1

    float f_cur_dist, f_iter_dist, f_point_vel, f_point_acc, f_last_point_vel, f_dt;
    float f_object_ds;
    float f_dist_sum = 0.0;
    float f_vel_sum = 0.0;
    float f_vel_last = 0.0;
    float f_last_dist = 0.0;

    iter_map_index = m_range_image_ptr_list.size() - 2;

    // 이 포인트의 직전 인덱스부터 시작. m_range_image_ptr_list는 back이 최신 
    point_soph cur_p = p;
    while (iter_map_index >= 0){
        
        // 유효 포지션인지 확인
        if(cur_p.occu_index[1] < 0 || cur_p.occu_index[1] >= m_i_max_2d_n) break;
        
        // 이 position이 해당 range에서 유효한지 확인
        if(m_range_image_ptr_list[iter_map_index]->range_image[cur_p.occu_index[1]].size() < 1) break;
        
        min_pos = m_range_image_ptr_list[iter_map_index]->min_range_index_all[cur_p.occu_index[1]];
        max_pos = m_range_image_ptr_list[iter_map_index]->max_range_index_all[cur_p.occu_index[1]];

        // 이 포지션을 가리게 한 min range pixel도 가려졌는지 확인
        if(m_range_image_ptr_list[iter_map_index]->range_image[cur_p.occu_index[1]][min_pos]->occluded_by_previous == false) break;

        // 현재 point의 현재 위치에서의 거리가, 과거 포인트의 현재 위치에서의 거리보다 가까우면 가려진게 아님. 
        f_cur_dist = cur_p.local.norm();
        f_iter_dist = (m_range_image_ptr_list[iter_map_index]->range_image[cur_p.occu_index[1]][min_pos]->glob - cur_p.sensor_transl).norm();
        if(f_cur_dist < f_iter_dist +  m_params.f_range_threshold) break;

        f_dt = cur_p.time - m_range_image_ptr_list[iter_map_index]->range_image[cur_p.occu_index[1]][min_pos]->time;

        f_dist_sum += f_cur_dist - f_iter_dist;
        f_last_dist = f_cur_dist - f_iter_dist;

        f_vel_sum += (f_cur_dist - f_iter_dist) / f_dt;
        f_vel_last = (f_cur_dist - f_iter_dist) / f_dt;

        // f_dt = cur_p.time - m_range_image_ptr_list[iter_map_index]->range_image[cur_p.occu_index[1]][min_pos]->time;
        // // f_point_vel = (f_cur_dist - f_iter_dist) / f_dt;
        // // f_point_acc = (f_point_vel - f_last_point_vel)/f_dt;

        // // FIXME: 속도 이상해.. f_dt, iter_map_index 문제 없음..
        // f_object_ds = (m_range_image_ptr_list[iter_map_index]->range_image[cur_p.occu_index[1]][min_pos]->glob - cur_p.glob).norm(); 
        // f_point_vel = f_object_ds / f_dt;
        // f_point_acc = (f_point_vel - f_last_point_vel)/f_dt;

        // // if(fabs(f_point_vel) > 30) std::cout<<"f_point_vel: "<<  f_point_vel <<" f_dt: "<<f_dt<<std::endl;
        // // 멀어지는 가속도가 5m/s^2 이상이면 FP로 간주
        // if(i_valid_range_num >= 1 && abs(f_point_acc) > 10){
        //     // std::cout<<"f_point_acc: "<<f_point_acc<<" dt: "<<f_dt<<std::endl;
        //     break;
        // }else if (i_valid_range_num >= 1){
        //     // std::cout<<"f_point_acc: "<<f_point_acc<<" dt: "<<f_dt<<std::endl;
        // }
        // f_last_point_vel = f_point_vel;


        // 다음 iter를 위해 저장
        cur_p = *m_range_image_ptr_list[iter_map_index]->range_image[cur_p.occu_index[1]][min_pos];

        i_valid_range_num++;
        iter_map_index--;
    }


    // f_cur_moving_evidence =  (float) i_valid_range_num / 5;
    f_cur_moving_evidence = (float) i_valid_range_num / (m_range_image_ptr_list.size() - 1);
    if(f_cur_moving_evidence > 1.0) f_cur_moving_evidence = 1.0;    

    p.r = (EvidType) 0;
    p.g = (EvidType) (f_cur_moving_evidence * MAX_EVIDENCE);
    p.b = (EvidType) ((1. - f_cur_moving_evidence )  * MAX_EVIDENCE);

    if(p.g > p.r && p.g > p.b){
        p.dyn = CASE2;

        // std::cout<< f_vel_sum / i_valid_range_num<<" "<<f_vel_last<<std::endl;
    } 

    return;
}

// points엔 point_soph들의 값이 있음  // TBB
void GaussianPointMotionClassi::GenerateRangeImage(std::vector<point_soph*> &points, double cur_time, M3D rot, V3D transl, RangeImage::Ptr range_image_ptr)
{   
    double t00 = omp_get_wtime();
    int i_points_len = points.size();
    std::cout<<"[GenerateRangeImage] Started"<<std::endl;
    std::cout<<"[GenerateRangeImage] m_i_cur_point_soph_pointers: "<<m_i_cur_point_soph_pointers<<std::endl;
    std::cout<<"[GenerateRangeImage] map_index: "<<m_i_map_index<<std::endl;

    int i_invalid_num = 0;
    float f_min_ver_angle = FLT_MAX;
    float f_max_ver_angle = -FLT_MAX;
    int i_min_ver_ind = INT_MAX;
    float f_max_hor_angle = -FLT_MAX;
    for (int k = 0; k < i_points_len; k++)
    { 
    // tbb::parallel_for(0, i_points_len, 1, [&](int k) {
        points[k]->GetVec(points[k]->local, m_params.f_horizontal_resolution, m_params.f_vertical_resolution);
        // if(points[k]->ver_ind < i_min_ver_ind) i_min_ver_ind = points[k]->ver_ind;

        // if (points[k]->vec(1) < f_min_ver_angle) f_min_ver_angle = points[k]->vec(1);
        // if (points[k]->vec(1) > f_max_ver_angle) f_max_ver_angle = points[k]->vec(1);
        // if (points[k]->vec(0) > f_max_hor_angle) f_max_hor_angle = points[k]->vec(0);
        
        // FOV를 벗어난 point는 range image에 할당하지 않음
        if(points[k]->ver_ind > m_i_max_1d_half || points[k]->ver_ind < 0 || \
           points[k]->hor_ind > m_i_max_1d      || points[k]->hor_ind < 0 || \
           points[k]->position < 0              || points[k]->position >= m_i_max_2d_n || \
           points[k]->local.norm() < m_params.f_min_range || points[k]->local.norm() > m_params.f_max_range)
        {
            points[k]->dyn = INVALID;
            i_invalid_num++;
            continue;
            // return;
        }


        if(range_image_ptr->range_image[points[k]->position].size() < m_params.i_max_points_in_pixel) // 해당 픽셀에 여유가 있다면 
        {   
            // if(range_image_ptr->range_image[points[k]->position].size() == 0) i_valid_position_num++;

            range_image_ptr->range_image[points[k]->position].push_back(points[k]); // 주소를 뒤에 추가함 

            // 이 포인트의 range가 픽셀 내 모든 포인트보다 크다면 
            if (points[k]->vec(2) > range_image_ptr->max_range_all[points[k]->position])  
            {
                range_image_ptr->max_range_all[points[k]->position] = points[k]->vec(2); // 최대 range 교체
                range_image_ptr->max_range_index_all[points[k]->position] = range_image_ptr->range_image[points[k]->position].size() - 1;
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
    // });

    std::cout<<"[GenerateRangeImage] Invalid Num: "<<i_invalid_num<<" / "<<i_points_len<<std::endl;
    double time_generate_range_image = omp_get_wtime() - t00;
    std::cout<<"[GenerateRangeImage] Total Time: "<< time_generate_range_image*1000.0 << " ms"<<std::endl;
}

void GaussianPointMotionClassi::GenerateRangeImageParallel(std::vector<point_soph*> &points, double cur_time, M3D rot, V3D transl, RangeImage::Ptr range_image_ptr)
{
    double t00 = omp_get_wtime();
    int i_points_len = points.size();
    std::cout<<"[GenerateRangeImageParallel] Started"<<std::endl;
    std::cout<<"[GenerateRangeImageParallel] m_i_cur_point_soph_pointers: "<<m_i_cur_point_soph_pointers<<std::endl;
    std::cout<<"[GenerateRangeImageParallel] map_index: "<<m_i_map_index<<std::endl;
    

    tbb::parallel_for(0, i_points_len, 1, [&](int k) {
        points[k]->GetVec(points[k]->local, m_params.f_horizontal_resolution, m_params.f_vertical_resolution);

        // FOV를 벗어난 point는 range image에 할당하지 않음
        if(points[k]->ver_ind > m_i_max_1d_half || points[k]->ver_ind < 0 || \
           points[k]->hor_ind > m_i_max_1d      || points[k]->hor_ind < 0 || \
           points[k]->position < 0              || points[k]->position > m_i_max_2d_n || \
           points[k]->local.norm() < m_params.f_min_range || points[k]->local.norm() > m_params.f_max_range)
        {
            points[k]->dyn = INVALID;
            return;
        }
    });

    double time_position = omp_get_wtime() - t00;
    std::cout<<"[GenerateRangeImageParallel] time_position: "<< time_position*1000.0 << " ms"<<std::endl;

    for (int k = 0; k < i_points_len; k++){
        if(points[k]->dyn == INVALID) continue;

        if(range_image_ptr->range_image[points[k]->position].size() < m_params.i_max_points_in_pixel) // 해당 픽셀에 여유가 있다면 
        {   
            range_image_ptr->range_image[points[k]->position].push_back(points[k]); // 주소를 뒤에 추가함 

            // 이 포인트의 range가 픽셀 내 모든 포인트보다 크다면 
            if (points[k]->vec(2) > range_image_ptr->max_range_all[points[k]->position])  
            {
                range_image_ptr->max_range_all[points[k]->position] = points[k]->vec(2); // 최대 range 교체
                range_image_ptr->max_range_index_all[points[k]->position] = range_image_ptr->range_image[points[k]->position].size() - 1;
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

    double time_generate_range_image = omp_get_wtime() - t00;
    std::cout<<"[GenerateRangeImageParallel] Total Time: "<< time_generate_range_image*1000.0 << " ms"<<std::endl;
}


// Segment Ground from Range Image // TBB
void GaussianPointMotionClassi::GroundSegmentation(RangeImage::Ptr range_image_ptr)
{   
    double t00 = omp_get_wtime();

    tbb::parallel_for(tbb::blocked_range<int>(0, m_i_pixel_fov_left - m_i_pixel_fov_right),
    [&](const tbb::blocked_range<int>& r) {
        for(int i_hor_idx = r.begin(); i_hor_idx != r.end(); ++i_hor_idx) {

            V3D u_point, l_point;
            float f_ver_angle, f_incident_angle, f_hor_incident_angle, f_ver_incident_angle;
            float f_hor_dist, f_ver_dist, f_l2_dist;
            float f_hor_dist_ground_l, f_hor_dist_ground_u;
            float f_upper_range, f_lower_range;
            float f_ver_angle_diff, f_hor_angle_diff, f_ver_angle_from_ground;

            int i_upper_pos, i_lower_pos, i_upper_point_min_pos, i_lower_point_min_pos;

            for(int i_ver_idx = 0; i_ver_idx < m_i_max_1d_half; i_ver_idx++) {
                i_lower_pos = i_hor_idx * m_i_max_1d_half + i_ver_idx;
                i_lower_point_min_pos = range_image_ptr->min_range_index_all[i_lower_pos];

                if(i_lower_point_min_pos == -1 || range_image_ptr->range_image[i_lower_pos].size() == 0) continue; // Check lower point valid

                l_point = range_image_ptr->range_image[i_lower_pos][i_lower_point_min_pos]->local;
                f_lower_range = range_image_ptr->range_image[i_lower_pos][i_lower_point_min_pos]->vec(2);

                for (int u_ver_idx = i_ver_idx + 1; u_ver_idx < i_ver_idx + 40 && u_ver_idx < m_i_max_1d_half ; u_ver_idx++){
                    
                    i_upper_pos = i_hor_idx * m_i_max_1d_half + u_ver_idx;
                    i_upper_point_min_pos = range_image_ptr->min_range_index_all[i_upper_pos];

                    u_point = range_image_ptr->range_image[i_upper_pos][i_upper_point_min_pos]->local;
                    f_upper_range = range_image_ptr->range_image[i_upper_pos][i_upper_point_min_pos]->vec(2);

                    if(i_upper_point_min_pos == -1 || range_image_ptr->range_image[i_upper_pos].size() == 0) continue; // Check upper point valid

                    f_ver_angle_diff = range_image_ptr->range_image[i_upper_pos][i_upper_point_min_pos]->vec(1) 
                                - range_image_ptr->range_image[i_lower_pos][i_lower_point_min_pos]->vec(1);

                    f_hor_dist = sqrt(pow(u_point.x() - l_point.x(), 2) + pow(u_point.y() - l_point.y(), 2));
                    if(f_hor_dist > 20) continue;

                    f_ver_dist = u_point.z() - l_point.z();
                    f_l2_dist = sqrt(f_ver_dist*f_ver_dist + f_hor_dist*f_hor_dist);

                    f_hor_dist_ground_u = sqrt(u_point.x() * u_point.x() + u_point.y() * u_point.y());
                    f_hor_dist_ground_l = sqrt(l_point.x() * l_point.x() + l_point.y() * l_point.y());
                    f_ver_angle_from_ground = atan2f(l_point.z() + m_params.vec_f_ego_to_lidar[2] , f_hor_dist_ground_l);

                    f_ver_angle = atan2f(f_ver_dist,f_hor_dist);
                    
                    if( f_upper_range*cos(f_ver_angle_diff) - f_lower_range > 0 )
                        f_ver_incident_angle = M_PI/2 - atan2f(f_upper_range*sin(f_ver_angle_diff), f_upper_range*cos(f_ver_angle_diff) - f_lower_range);
                    else
                        f_ver_incident_angle = M_PI/2 - atan2f(f_upper_range*sin(f_ver_angle_diff), - f_upper_range*cos(f_ver_angle_diff) + f_lower_range);     

                    f_incident_angle = f_ver_incident_angle;

                    range_image_ptr->range_image[i_lower_pos][i_lower_point_min_pos]->incident = f_incident_angle;
                    range_image_ptr->incident_all[i_lower_pos] = f_incident_angle;

                    if(fabs(f_ver_angle) < m_params.f_ground_angle * M_PI/180.0  /* && f_ver_angle_from_ground < m_params.f_ground_angle * M_PI/180.0 */ &&
                        f_hor_dist_ground_u > f_hor_dist_ground_l){

                        range_image_ptr->range_image[i_upper_pos][i_upper_point_min_pos]->ground = true;
                        range_image_ptr->ground_all[i_upper_pos] = true;

                        if(i_ver_idx == 0){
                            range_image_ptr->range_image[i_lower_pos][i_lower_point_min_pos]->ground = true;
                            range_image_ptr->ground_all[i_lower_pos] = true;          
                        }
                    }
                    else if (range_image_ptr->range_image[i_lower_pos][i_lower_point_min_pos]->ground == true && f_l2_dist < 0.3 && f_hor_dist < 0.1){
                        // Upper is nonground and lower is ground and both are closed,
                            range_image_ptr->range_image[i_lower_pos][i_lower_point_min_pos]->ground = false;
                            range_image_ptr->ground_all[i_lower_pos] = false;   
                    }

                    break;
                }
            }
        }
    });

    // TODO: min pos가 아닌 pixel에 대해 Ground FN 처리하기

    double ground_total = omp_get_wtime() - t00;
    std::cout<<"[GroundSegmentation] Total Time: "<< ground_total*1000.0 << " ms"<<std::endl;
}

// Segement Object
void GaussianPointMotionClassi::ObjectSegmentation(RangeImage::Ptr range_image_ptr)
{
    double t00 = omp_get_wtime();

    ResetClustering();

    range_image_ptr->cluster_poses_vec.clear();

    int i_from_pos, i_from_min_pos;

    m_i_cluster_idx = 0;
    for(int i_ver_idx = 0 ; i_ver_idx < m_i_max_1d_half ; ++i_ver_idx)
    {
        for(int i_hor_idx = 0 ; i_hor_idx < m_i_max_1d ; ++i_hor_idx)
        {   
            i_from_pos = i_hor_idx * m_i_max_1d_half + i_ver_idx;
            i_from_min_pos = range_image_ptr->min_range_index_all[i_from_pos];

            if(range_image_ptr->range_image[i_from_pos].size() > 0 &&
               range_image_ptr->range_image[i_from_pos][i_from_min_pos]->ground == false &&
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


void GaussianPointMotionClassi::NeighborAssign(unsigned int hor_neighbor, unsigned int ver_neighbor)
{
    m_v_neighbor_iter.clear();
    std::pair<int8_t, int8_t> neighbor;
    
    for(int i_hor_ind = 1; i_hor_ind <= hor_neighbor; i_hor_ind++ ){
        for(int i_ver_ind = 1; i_ver_ind <= ver_neighbor; i_ver_ind++ ){
            neighbor.first = -i_hor_ind;  neighbor.second =  0;       m_v_neighbor_iter.push_back(neighbor);
            neighbor.first =  0;        neighbor.second =  i_ver_ind; m_v_neighbor_iter.push_back(neighbor);
            neighbor.first =  0;        neighbor.second = -i_ver_ind; m_v_neighbor_iter.push_back(neighbor);
            neighbor.first =  i_hor_ind;  neighbor.second =  0;       m_v_neighbor_iter.push_back(neighbor);
        }
    }
}

void GaussianPointMotionClassi::LabelComponents(RangeImage::Ptr range_image_ptr, uint16_t row, uint16_t col)
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

        i_from_pos = i_from_x_idx * m_i_max_1d_half + i_from_y_idx;
        i_from_min_pos = range_image_ptr->min_range_index_all[i_from_pos];

        // 해당 픽셀에 포인트가 없거나, ground이면 skip
        if (range_image_ptr->range_image[i_from_pos].size() == 0 ||
            range_image_ptr->range_image[i_from_pos][i_from_min_pos]->ground == true)
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
            if (i_this_y_idx < 0 || i_this_y_idx >= m_i_max_1d_half)
                continue;

            // Check continuity of col idx
            if (i_this_x_idx < 0) i_this_x_idx = i_this_x_idx + m_i_col_size;
            if (i_this_x_idx >= m_i_col_size) i_this_x_idx = i_this_x_idx - m_i_col_size;

            i_this_pos = i_this_x_idx * m_i_max_1d_half + i_this_y_idx;
            i_this_min_pos = range_image_ptr->min_range_index_all[i_this_pos];


            // Check current pixel is already segmented or ground
            if (range_image_ptr->range_image[i_this_pos].size() == 0 ||
                range_image_ptr->range_image[i_this_pos][i_this_min_pos]->ground == true ||
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

                lineCountFlag[i_this_y_idx - 0] = true;

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
            // range_image_ptr->cluster_idx_all[pos_min_pos.first] = m_i_cluster_idx;
            vec_cluster_pos.push_back(pos_min_pos.first); // pos 
        }

        range_image_ptr->cluster_poses_vec.push_back(vec_cluster_pos);
        m_i_cluster_idx++;
    }
}


void GaussianPointMotionClassi::ResetClustering()
{
    m_v_ui16_queue_idx_x.clear();
    m_v_ui16_queue_idx_x.resize(m_i_col_size*m_i_row_size);
    m_v_ui16_queue_idx_y.clear();
    m_v_ui16_queue_idx_y.resize(m_i_col_size*m_i_row_size);
}


// TODO: 다이나믹 포인트 주변을 range image 상에서 탐색
// point가 static 사이에 둘러 쌓여 있으면 FP로 간주
void GaussianPointMotionClassi::StaticRegionGrowing(std::vector<point_soph*> &points, RangeImage::Ptr range_image_ptr)
{
    double time_static_growing_start = omp_get_wtime();

    size_t i_point_num = points.size();
    int i_vertical_search_range = 2;
    int i_horizontal_search_range = 2;
    int i_total_window_point = (2*i_horizontal_search_range + 1) * (2*i_vertical_search_range + 1) - 1;


    tbb::parallel_for(tbb::blocked_range<size_t>(0, i_point_num),
        [&](const tbb::blocked_range<size_t>& r) {
            for(size_t p_ind = r.begin(); p_ind != r.end(); ++p_ind) {

                // 원래의 position, vec, hor ver ind 되돌려놓음
                points[p_ind]->GetVec(points[p_ind]->local, m_params.f_horizontal_resolution, m_params.f_vertical_resolution);

                if(points[p_ind]->dyn != CASE1 && points[p_ind]->dyn != CASE2) continue; // 현재 포인트가 다이나믹인것만 시작 

                bool b_distance_near = false;
                int i_near_static_point = 0;
                int i_valid_near_point = 0;
                float f_nearest_distance = FLT_MAX;
                
                point_soph* tmp_point_ptr;

                // 포인트 주변 탐색
                for (int i = -i_horizontal_search_range; i <= i_horizontal_search_range; i++) {
                    for (int j = -i_vertical_search_range; j <= i_vertical_search_range; j++) {
                        int iter_pos = (points[p_ind]->hor_ind + i) * m_i_max_1d_half + points[p_ind]->ver_ind + j;
                        if(iter_pos < m_i_max_2d_n && iter_pos >= 0 && range_image_ptr->range_image[iter_pos].size() > 0) {
                            if(iter_pos == points[p_ind]->position) continue;

                            i_valid_near_point++;

                            int min_pos = range_image_ptr->min_range_index_all[iter_pos];
                            if(range_image_ptr->range_image[iter_pos][min_pos]->dyn != STATIC) continue; // 스태틱 의한 것만 전파 받겠다

                            float f_point_distance = (points[p_ind]->local - range_image_ptr->range_image[iter_pos][min_pos]->local).norm();
                            // float f_gaussian_weight = GaussianWeight(f_point_distance, m_params.f_dynamic_gaussian_sigma);

                            if(f_point_distance < 1.0){
                                
                                i_near_static_point++;

                                if(f_point_distance < f_nearest_distance){
                                    f_nearest_distance = f_point_distance;
                                    tmp_point_ptr = range_image_ptr->range_image[iter_pos][min_pos];
                                }

                            }
                        }
                    }
                }

                if(i_near_static_point > 0 && i_near_static_point > 2){
                    points[p_ind]->r = tmp_point_ptr->r;
                    points[p_ind]->g = tmp_point_ptr->g;
                    points[p_ind]->b = tmp_point_ptr->b;
                    points[p_ind]->fp_dynamic = true;
                }
            }
        }
    );


    tbb::parallel_for(tbb::blocked_range<size_t>(0, i_point_num),
        [&](const tbb::blocked_range<size_t>& r) {
            for(size_t p_ind = r.begin(); p_ind != r.end(); ++p_ind) {

                if(points[p_ind]->fp_dynamic == true){
                    points[p_ind]->dyn = STATIC;
                } 

            }
        }
    );
    double time_static_growing = omp_get_wtime() - time_static_growing_start;
    std::cout<<"[StaticRegionGrowing] Static growing Total: "<< time_static_growing*1000.0 << " ms"<<std::endl;

}
// Unknown, Static에 대해서 range image에서 아래가 지면, 위가 dynamic이면 그것도 dynamic
// Cluster 내 Static, Unknown에 대해서 주변 6x6픽셀을 탐색하며, 확률을 가져옴
void GaussianPointMotionClassi::ImageRegionGrowing(std::vector<point_soph*> &points, RangeImage::Ptr range_image_ptr)
{   
    double time_region_growing_start = omp_get_wtime();

    size_t i_point_num = points.size();
    int i_vertical_search_range = 20;
    int i_horizontal_search_range = 40;
    int i_skip_size = 4;
    double d_max_dist = 5.0;

    tbb::parallel_for(tbb::blocked_range<size_t>(0, i_point_num),
        [&](const tbb::blocked_range<size_t>& r) {
            for(size_t p_ind = r.begin(); p_ind != r.end(); ++p_ind) {

                if(points[p_ind]->cluster_ind < 0 ) continue; // Cluster 된 것만 보겠다
                if(points[p_ind]->dyn != UNKNOWN && points[p_ind]->dyn != STATIC) continue; // 현재 포인트가 언노운 이나 스태틱인것만 시작 

                // 원래의 position, vec, hor ver ind 되돌려놓음
                points[p_ind]->GetVec(points[p_ind]->local, m_params.f_horizontal_resolution, m_params.f_vertical_resolution);

                bool b_find_near = false;

                // 먼 거리의 포인트는 포인트 간격이 커서 window를 작게 걸어도 멀리 탐색됨
                double d_hor_res_dist = points[p_ind]->vec(2) * sin(m_params.f_horizontal_resolution);
                double d_ver_res_dist = points[p_ind]->vec(2) * sin(m_params.f_vertical_resolution);

                int iter_hor_search_index = std::min(i_horizontal_search_range, (int)floor(d_max_dist / d_hor_res_dist));
                int iter_ver_search_index = std::min(i_vertical_search_range,   (int)floor(d_max_dist / d_ver_res_dist));

                // Dynamic 확률 가져올 포인트 찾음 
                for (int i = -iter_hor_search_index; i <= iter_hor_search_index && !b_find_near; i += i_skip_size) {
                    for (int j = -iter_ver_search_index; j <= iter_ver_search_index && !b_find_near; j += i_skip_size) {
                        int iter_pos = (points[p_ind]->hor_ind + i) * m_i_max_1d_half + points[p_ind]->ver_ind + j;
                        if(iter_pos < m_i_max_2d_n && iter_pos >= 0 && range_image_ptr->range_image[iter_pos].size() > 0) {
                            if(iter_pos == points[p_ind]->position) continue; // 같은 point 넘기기 
                            
                            int min_pos = range_image_ptr->min_range_index_all[iter_pos];
                            if(range_image_ptr->range_image[iter_pos][min_pos]->cluster_ind != points[p_ind]->cluster_ind) continue; // 같은 클러스터인지 확인
                            if(range_image_ptr->range_image[iter_pos][min_pos]->propagated == true) continue; // 전파에 의한건 안받겠다
                            if(range_image_ptr->range_image[iter_pos][min_pos]->dyn != CASE1 &&
                                range_image_ptr->range_image[iter_pos][min_pos]->dyn != CASE2) continue;
                               // 다이나믹에 의한 것만 전파 받겠다

                            float f_point_dist_btw = (points[p_ind]->local - range_image_ptr->range_image[iter_pos][min_pos]->local).norm();
                            float f_gaussian_weight = GaussianWeight(f_point_dist_btw, m_params.f_dynamic_gaussian_sigma);

                            if(f_point_dist_btw < d_max_dist){
  
                                points[p_ind]->dyn = range_image_ptr->range_image[iter_pos][min_pos]->dyn;
                                points[p_ind]->propagated = true;

                                EvidType SrcEvidence[3] = {(EvidType)(points[p_ind]->r),
                                                                (EvidType)(points[p_ind]->g),
                                                                (EvidType)(points[p_ind]->b)};      

                                EvidType MovingEvidence[3] = {(EvidType)(range_image_ptr->range_image[iter_pos][min_pos]->r * f_gaussian_weight), 
                                (EvidType)(range_image_ptr->range_image[iter_pos][min_pos]->g * f_gaussian_weight),
                                (EvidType)(MAX_EVIDENCE - 
                                        range_image_ptr->range_image[iter_pos][min_pos]->r * f_gaussian_weight - 
                                        range_image_ptr->range_image[iter_pos][min_pos]->g * f_gaussian_weight)};

                                DempsterCombination(SrcEvidence, MovingEvidence);

                                points[p_ind]->r = (uint8_t)SrcEvidence[0];
                                points[p_ind]->g = (uint8_t)SrcEvidence[1];
                                points[p_ind]->b = (uint8_t)SrcEvidence[2];

                                // Dynamic 확률이 괜찮은 포인트를 만났다면, find near 판정. For문 탈출하도록.
                                if(points[p_ind]->g > points[p_ind]->r && points[p_ind]->g > points[p_ind]->b){
                                    b_find_near = true;
                                }
                            }
                        }
                    }
                }
            }
        }
    );

    double time_region_growing = omp_get_wtime() - time_region_growing_start;
    std::cout<<"[ImageRegionGrowing] Region growing Total: "<< time_region_growing*1000.0 << " ms"<<std::endl;
}


// TODO:
void GaussianPointMotionClassi::RegionGrowingWithCluster(std::vector<point_soph*> &points, RangeImage::Ptr range_image_ptr)
{   
    double time_region_growing_start = omp_get_wtime();

    size_t i_point_num = points.size();
    int i_vertical_search_range = 20;
    int i_horizontal_search_range = 40;
    double d_max_dist = 5.0;

    tbb::parallel_for(tbb::blocked_range<size_t>(0, i_point_num),
        [&](const tbb::blocked_range<size_t>& r) {
            for(size_t p_ind = r.begin(); p_ind != r.end(); ++p_ind) {

                if(points[p_ind]->dyn != UNKNOWN && points[p_ind]->dyn != STATIC) continue; // 현재 포인트가 언노운 이나 스태틱인것만 시작 
                if(points[p_ind]->ground == true) continue; // ground는 안함


                // 원래의 position, vec, hor ver ind 되돌려놓음
                points[p_ind]->GetVec(points[p_ind]->local, m_params.f_horizontal_resolution, m_params.f_vertical_resolution);

                if(range_image_ptr->ground_all[points[p_ind]->position] == true) continue;


                bool b_find_near = false;
                int i_jump_count = 0;

                // 먼 거리의 포인트는 포인트 간격이 커서 window를 작게 걸어도 멀리 탐색됨
                double d_hor_res_dist = points[p_ind]->vec(2) * sin(m_params.f_horizontal_resolution);
                double d_ver_res_dist = points[p_ind]->vec(2) * sin(m_params.f_vertical_resolution);

                int iter_hor_search_index = std::min(i_horizontal_search_range, (int)floor(d_max_dist / d_hor_res_dist));
                int iter_ver_search_index = std::min(i_vertical_search_range,   (int)floor(d_max_dist / d_ver_res_dist));

                if(!b_find_near)
                    b_find_near = ClusterDemster(range_image_ptr, points[p_ind], iter_ver_search_index, 0); // 위로 찾기

                if(!b_find_near)
                    b_find_near = ClusterDemster(range_image_ptr, points[p_ind], iter_ver_search_index, iter_hor_search_index); // 위 왼쪽 찾기

                if(!b_find_near)
                    b_find_near = ClusterDemster(range_image_ptr, points[p_ind], -iter_ver_search_index, -iter_hor_search_index); // 위 오른쪽 찾기

                if(!b_find_near)
                    b_find_near = ClusterDemster(range_image_ptr, points[p_ind], -iter_ver_search_index, 0); // 아래 찾기

                if(!b_find_near)
                    b_find_near = ClusterDemster(range_image_ptr, points[p_ind], 0, iter_hor_search_index); // 왼쪽 찾기

                if(!b_find_near)
                    b_find_near = ClusterDemster(range_image_ptr, points[p_ind], 0, -iter_hor_search_index); // 오른쪽 찾기

            }
        }
    );

    // tbb::parallel_for(tbb::blocked_range<size_t>(0, i_point_num),
    //     [&](const tbb::blocked_range<size_t>& r) {
    //         for(size_t p_ind = r.begin(); p_ind != r.end(); ++p_ind) {

    //             if(points[p_ind]->dyn != UNKNOWN && points[p_ind]->dyn != STATIC) continue; // 현재 포인트가 언노운 이나 스태틱인것만 시작 
    //             if(points[p_ind]->ground == true) continue; // ground는 안함


    //             // 원래의 position, vec, hor ver ind 되돌려놓음
    //             points[p_ind]->GetVec(points[p_ind]->local, m_params.f_horizontal_resolution, m_params.f_vertical_resolution);

    //             if(range_image_ptr->ground_all[points[p_ind]->position] == true) continue;


    //             bool b_find_near = false;
    //             int i_jump_count = 0;

    //             // 먼 거리의 포인트는 포인트 간격이 커서 window를 작게 걸어도 멀리 탐색됨
    //             double d_hor_res_dist = points[p_ind]->vec(2) * sin(m_params.f_horizontal_resolution);
    //             double d_ver_res_dist = points[p_ind]->vec(2) * sin(m_params.f_vertical_resolution);

    //             int iter_hor_search_index = std::min(i_horizontal_search_range, (int)floor(d_max_dist / d_hor_res_dist));
    //             int iter_ver_search_index = std::min(i_vertical_search_range,   (int)floor(d_max_dist / d_ver_res_dist));

    //             if(!b_find_near)
    //                 b_find_near = ClusterDemsterWithPropagation(range_image_ptr, points[p_ind], iter_ver_search_index, 0); // 위로 찾기

    //             if(!b_find_near)
    //                 b_find_near = ClusterDemsterWithPropagation(range_image_ptr, points[p_ind], -iter_ver_search_index, 0); // 아래 찾기

    //             if(!b_find_near)
    //                 b_find_near = ClusterDemsterWithPropagation(range_image_ptr, points[p_ind], 0, iter_hor_search_index); // 왼쪽 찾기

    //             if(!b_find_near)
    //                 b_find_near = ClusterDemsterWithPropagation(range_image_ptr, points[p_ind], 0, -iter_hor_search_index); // 오른쪽 찾기

    //         }
    //     }
    // );

    double time_region_growing = omp_get_wtime() - time_region_growing_start;
    std::cout<<"[RegionGrowingWithCluster] Region growing Total: "<< time_region_growing*1000.0 << " ms"<<std::endl;
}

bool GaussianPointMotionClassi::ClusterDemster(RangeImage::Ptr range_image_ptr, point_soph* point, int iter_ver_search_index, int iter_hor_search_index)
{
    int i_jump_count = 0;
    int i_max_jump_count = 3;

    int iter_pos;
    V3D last_local = point->local;
    
    float f_point_dist_btw_local, f_point_dist_btw_neighbor;

    // 멀리 있는 포인트 끼리는 전파율이 낮도록 함
    float f_cur_point_dist_coeff = GaussianWeight(point->local.norm(),30);
    f_cur_point_dist_coeff = 1.0;

    int i_hor_add, i_ver_add;
    int i_max_search_index = std::max(abs(iter_ver_search_index), abs(iter_hor_search_index));

    if(iter_hor_search_index == 0 ) i_hor_add = 0;
    else if (iter_hor_search_index > 0) i_hor_add = 1;
    else i_hor_add = -1;

    if(iter_ver_search_index == 0 ) i_ver_add = 0;
    else if (iter_ver_search_index > 0) i_ver_add = 1;
    else i_ver_add = -1;

    // Ver 아래왼쪽 시작
    for (int j = 1; j < i_max_search_index; j ++){

        iter_pos = (point->hor_ind - j*i_hor_add) * m_i_max_1d_half + point->ver_ind + j*i_ver_add;

        if(point->hor_ind + j*i_hor_add >= m_i_max_1d       || point->hor_ind + j*i_hor_add < 0) break;
        if(point->ver_ind + j*i_ver_add >= m_i_max_1d_half  || point->ver_ind + j*i_ver_add < 0) break;
        
        // Range image 내 유효 좌표 확인
        if(iter_pos < m_i_max_2d_n && iter_pos >= 0 ) {

            // 포인트 없으면 점프
            if(range_image_ptr->range_image[iter_pos].size() == 0){
                i_jump_count++;

                if(i_jump_count > i_max_jump_count){
                    return false;
                }       
                continue;
            }

            int min_pos = range_image_ptr->min_range_index_all[iter_pos];
            f_point_dist_btw_neighbor = (last_local - range_image_ptr->range_image[iter_pos][min_pos]->local).norm();
            f_point_dist_btw_local = (point->local - range_image_ptr->range_image[iter_pos][min_pos]->local).norm();       

            // 거리 멀면 점프
            if(f_point_dist_btw_neighbor > m_params.f_dist_threshold_m ){
                i_jump_count++;

                if(i_jump_count > i_max_jump_count){
                    return false;
                }
                continue;
            }
            else{ // 포인트 있고 거리 가까우면 뎀스터 계산 시작
                i_jump_count = 0;

                // // 지난 local 변경
                if(range_image_ptr->range_image[iter_pos][min_pos]->ground == false)
                    last_local = range_image_ptr->range_image[iter_pos][min_pos]->local;

                if(range_image_ptr->range_image[iter_pos][min_pos]->propagated == true) continue; // 전파에 의한건 안받겠다
                if(range_image_ptr->range_image[iter_pos][min_pos]->dyn != CASE1 &&
                range_image_ptr->range_image[iter_pos][min_pos]->dyn != CASE2) continue;
                // 다이나믹에 의한 것만 전파 받겠다

                float f_gaussian_weight = GaussianWeight(f_point_dist_btw_local, m_params.f_dynamic_gaussian_sigma) * f_cur_point_dist_coeff;
                point->dyn = range_image_ptr->range_image[iter_pos][min_pos]->dyn;
                point->propagated = true;

                EvidType SrcEvidence[3] = {(EvidType)(point->r),
                                                (EvidType)(point->g),
                                                (EvidType)(point->b)};      

                EvidType MovingEvidence[3] = {(EvidType)(range_image_ptr->range_image[iter_pos][min_pos]->r * f_gaussian_weight), 
                (EvidType)(range_image_ptr->range_image[iter_pos][min_pos]->g * f_gaussian_weight),
                (EvidType)(MAX_EVIDENCE - 
                        range_image_ptr->range_image[iter_pos][min_pos]->r * f_gaussian_weight - 
                        range_image_ptr->range_image[iter_pos][min_pos]->g * f_gaussian_weight)};

                DempsterCombination(SrcEvidence, MovingEvidence);

                point->r = (uint8_t)SrcEvidence[0];
                point->g = (uint8_t)SrcEvidence[1];
                point->b = (uint8_t)SrcEvidence[2];

                // Dynamic 확률이 괜찮은 포인트를 만났다면, find near 판정. For문 탈출하도록.
                if(point->g > point->r && point->g > point->b){
                    return true;
                }
            }
        }
    }

    return false;
}

bool GaussianPointMotionClassi::ClusterDemsterWithPropagation(RangeImage::Ptr range_image_ptr, point_soph* point, int iter_ver_search_index, int iter_hor_search_index)
{
    int i_jump_count = 0;
    int i_max_jump_count = 3;

    int iter_pos;
    V3D last_local = point->local;
    
    float f_point_dist_btw_local, f_point_dist_btw_neighbor;

    // 멀리 있는 포인트 끼리는 전파율이 낮도록 함
    float f_cur_point_dist_coeff = GaussianWeight(point->local.norm(),20);

    int i_hor_add, i_ver_add;
    int i_max_search_index = std::max(abs(iter_ver_search_index), abs(iter_hor_search_index));

    if(iter_hor_search_index == 0 ) i_hor_add = 0;
    else if (iter_hor_search_index > 0) i_hor_add = 1;
    else i_hor_add = -1;

    if(iter_ver_search_index == 0 ) i_ver_add = 0;
    else if (iter_ver_search_index > 0) i_ver_add = 1;
    else i_ver_add = -1;

    // Ver 아래왼쪽 시작
    for (int j = 1; j < i_max_search_index; j ++){

        iter_pos = (point->hor_ind - j*i_hor_add) * m_i_max_1d_half + point->ver_ind + j*i_ver_add;

        if(point->hor_ind + j*i_hor_add >= m_i_max_1d       || point->hor_ind + j*i_hor_add < 0) break;
        if(point->ver_ind + j*i_ver_add >= m_i_max_1d_half  || point->ver_ind + j*i_ver_add < 0) break;
        
        // Range image 내 유효 좌표 확인
        if(iter_pos < m_i_max_2d_n && iter_pos >= 0 ) {

            // 포인트 없으면 점프
            if(range_image_ptr->range_image[iter_pos].size() == 0){
                i_jump_count++;

                if(i_jump_count > i_max_jump_count){
                    return false;
                }       
                continue;
            }

            int min_pos = range_image_ptr->min_range_index_all[iter_pos];
            f_point_dist_btw_neighbor = (last_local - range_image_ptr->range_image[iter_pos][min_pos]->local).norm();
            f_point_dist_btw_local = (point->local - range_image_ptr->range_image[iter_pos][min_pos]->local).norm();       

            // 거리 멀면 점프
            if(f_point_dist_btw_neighbor > m_params.f_dist_threshold_m ){
                i_jump_count++;

                if(i_jump_count > i_max_jump_count){
                    return false;
                }
                continue;
            }
            else{ // 포인트 있고 거리 가까우면 뎀스터 계산 시작
                i_jump_count = 0;

                // // 지난 local 변경
                if(range_image_ptr->range_image[iter_pos][min_pos]->ground == false)
                    last_local = range_image_ptr->range_image[iter_pos][min_pos]->local;

                if(range_image_ptr->range_image[iter_pos][min_pos]->propagated == false) continue; // 전파에 의한것만 받겠다
                if(range_image_ptr->range_image[iter_pos][min_pos]->dyn != CASE1 &&
                range_image_ptr->range_image[iter_pos][min_pos]->dyn != CASE2) continue;
                // 다이나믹에 의한 것만 전파 받겠다

                float f_gaussian_weight = GaussianWeight(f_point_dist_btw_local, m_params.f_dynamic_gaussian_sigma) * f_cur_point_dist_coeff;
                point->dyn = range_image_ptr->range_image[iter_pos][min_pos]->dyn;
                point->propagated = true;

                EvidType SrcEvidence[3] = {(EvidType)(point->r),
                                                (EvidType)(point->g),
                                                (EvidType)(point->b)};      

                EvidType MovingEvidence[3] = {(EvidType)(range_image_ptr->range_image[iter_pos][min_pos]->r * f_gaussian_weight), 
                (EvidType)(range_image_ptr->range_image[iter_pos][min_pos]->g * f_gaussian_weight),
                (EvidType)(MAX_EVIDENCE - 
                        range_image_ptr->range_image[iter_pos][min_pos]->r * f_gaussian_weight - 
                        range_image_ptr->range_image[iter_pos][min_pos]->g * f_gaussian_weight)};

                DempsterCombination(SrcEvidence, MovingEvidence);

                point->r = (uint8_t)SrcEvidence[0];
                point->g = (uint8_t)SrcEvidence[1];
                point->b = (uint8_t)SrcEvidence[2];

                // Dynamic 확률이 괜찮은 포인트를 만났다면, find near 판정. For문 탈출하도록.
                if(point->g > point->r && point->g > point->b){
                    return true;
                }
            }
        }
    }

    return false;
}

// Voxel Grid 기반 Clustering 및 필터링
void GaussianPointMotionClassi::SmallDynamicFiltering(std::vector<point_soph*> &points, RangeImage::Ptr range_image_ptr)
{   
    double time_small_point_filtering_start = omp_get_wtime();

    float Voxel_revolusion = 0.5;
    float GridMapedgesize_xy = m_params.f_max_range * 2 / Voxel_revolusion;
    float GridMapedgesize_z = m_params.f_max_range / Voxel_revolusion;
    Eigen::Vector3f xyz_origin;
    xyz_origin << -m_params.f_max_range, -m_params.f_max_range, -m_params.f_max_range / 2.0f;
    int cluster_extend_pixel = 1; // 주변 탐색 거리
    int cluster_min_pixel_number = m_params.i_min_voxel_cluster_num; // 최소 유효 복셀 개수 

    std::unordered_map<int, Point_Cloud::Ptr> umap_cluster; // 포인트 인덱스에 대한 point cloud ptr
    std::vector<std::vector<int>> voxel_cluster; // cluster index --> point index
    
    int i_point_num = points.size();

    m_tbb_points_normal_pcptr->clear();
    m_tbb_points_normal_pcptr->resize(i_point_num);

    m_tbb_dynamic_points_normal_pcptr->clear();

    // tbb
    std::vector<int> vec_refined_dynamic_point_ind(i_point_num, -1);
    std::vector<int> valid_indices;
    
    tbb::parallel_for(0, i_point_num, 1, [&](int i) {
        if (points[i]->dyn == CASE1 || points[i]->dyn == CASE2 ||
            (points[i]->g > max(points[i]->r, points[i]->b))) {
            // p->ind 값을 인덱스로 사용하여 포인트 클라우드의 해당 위치를 직접 수정합니다.
            PointType& point = m_tbb_points_normal_pcptr->points[points[i]->ind];

            point.x = static_cast<float>(points[i]->local.x());
            point.y = static_cast<float>(points[i]->local.y());
            point.z = static_cast<float>(points[i]->local.z());
            point.intensity = static_cast<uint32_t>(points[i]->ind); // 인덱스를 라벨로 사용합니다.

            // vec_refined_dynamic_point_ind.push_back(p->ind);
            vec_refined_dynamic_point_ind[points[i]->ind] = points[i]->ind;
        }
    });

    // 유효한 인덱스만 선택
    for (const auto& index : vec_refined_dynamic_point_ind) {
        if (index >= 0) {
            valid_indices.push_back(index); // Dynamic 포인트에 대한 points[i]->ind 저장 
        }
    }

    std::cout<<"Total Points: "<<points.size()<<std::endl;
    std::cout<<"Before: "<<m_tbb_points_normal_pcptr->points.size()<<std::endl;
    pcl::copyPointCloud(*m_tbb_points_normal_pcptr, valid_indices, *m_tbb_dynamic_points_normal_pcptr);
    std::cout<<"After : "<<m_tbb_dynamic_points_normal_pcptr->points.size()<<std::endl;

    std::cout<<"[SmallDynamicFiltering] Dynamic Total: "<<m_tbb_dynamic_points_normal_pcptr->points.size()<<std::endl;

    VOXEL_CLUSTER cluster;
    cluster.setInputCloud(*m_tbb_dynamic_points_normal_pcptr);
    cluster.setVoxelResolution(Voxel_revolusion, GridMapedgesize_xy, GridMapedgesize_z, xyz_origin);
    cluster.setExtendRange(cluster_extend_pixel);
    cluster.setMinClusterSize(cluster_min_pixel_number);

    cluster.createVoxelMap(umap_cluster); // umap_cluster에는 voxel 고유 position이 인덱스로
    cluster.extractReverse(voxel_cluster); // cluster ind --> voxel ind
    // cluster.extractReverseHeight(voxel_cluster, 0.6); // cluster ind --> voxel ind

    // FIXME: 제거가 안됨!! 총 Range Image 수가 10개가 되기 전에만 오류 발생함..

    std::cout<<"[SmallDynamicFiltering] Erasing Cluster Num "<<voxel_cluster.size()<<std::endl;
    int i_erased_point = 0;

    // 클러스터 순회
    for (int cluster_ind = 0; cluster_ind < voxel_cluster.size(); cluster_ind++)
    {   
        int i_voxel_num = voxel_cluster[cluster_ind].size();
        int i_total_point_num = 0;

        float max_height = -FLT_MAX;
        float min_height = FLT_MAX;
        // 클러스터 내 복셀 순회
        for(auto voxel_ind : voxel_cluster[cluster_ind])
        {   
            // 복셀 내 포인트 순회
            i_total_point_num += umap_cluster[voxel_ind]->cloud->size();
            
            for (int j = 0; j < umap_cluster[voxel_ind]->cloud->size(); j++)
            {
                if(umap_cluster[voxel_ind]->cloud->points[j].z > max_height) max_height = umap_cluster[voxel_ind]->cloud->points[j].z;
                if(umap_cluster[voxel_ind]->cloud->points[j].z < min_height) min_height = umap_cluster[voxel_ind]->cloud->points[j].z;
            }
        }

        if(max_height - min_height > 0.5 && i_voxel_num >= 3) continue;
        
        // 클러스터 내 복셀 순회
        for(auto voxel_ind : voxel_cluster[cluster_ind])
        {   
            // 복셀 내 포인트 순회
            for (int j = 0; j < umap_cluster[voxel_ind]->cloud->size(); j++)
            {
                int pcl_ind = umap_cluster[voxel_ind]->cloud_index->at(j); // m_tbb_dynamic_points_normal_pcptr 의 points 인덱스임
                int range_point_ind = (int)(m_tbb_dynamic_points_normal_pcptr->points[pcl_ind].intensity); // points[i]->ind 와 같음
                

                points[range_point_ind]->r = 255; // FIXME:
                points[range_point_ind]->g = 0;
                points[range_point_ind]->b = 0;

                points[range_point_ind]->dyn = STATIC;

                i_erased_point++;
            }
        }
    }


    double time_small_point_filtering = omp_get_wtime() - time_small_point_filtering_start;

    std::cout<<"[SmallDynamicFiltering] Erased Dyn point: "<<i_erased_point<<std::endl;
    std::cout<<"[SmallDynamicFiltering] SD Total: "<< time_small_point_filtering*1000.0 << " ms"<<std::endl;
}


// Range Image의 rot, transl를 입력받아 point의 world상의 해당 range image 상의 point_soph 값 반환
void GaussianPointMotionClassi::SphericalProjection(point_soph &p, int range_index, const M3D &rot, const V3D &transl, point_soph &p_spherical)
{
    V3D p_projected(rot.inverse()*(p.glob - transl)); // 현재 range image 좌표계로 이전됨 p의 위치 

    // GetVec을 통해 vec과 ind, position이 할당됨
    p_spherical.GetVec(p_projected, m_params.f_horizontal_resolution, m_params.f_vertical_resolution);
}

bool GaussianPointMotionClassi::KeyFrameCheck(double cur_time, M3D rot, V3D transl)
{
    if(m_range_image_ptr_list.size() == 0) return true;

    Eigen::Matrix3d relative_rotation =  m_range_image_ptr_list.back()->project_R.transpose() * rot;
    Eigen::AngleAxisd angle_axis(relative_rotation);



    if(cur_time - m_range_image_ptr_list.back()->time > m_params.f_min_key_frame_time ||
       (m_range_image_ptr_list.back()->project_T - transl).norm() > m_params.f_min_key_frame_rot ||
       (angle_axis.angle() > m_params.f_min_key_frame_rot))
    {
        std::cout.precision(15);
        std::cout<<"cur_time: "<<cur_time<<" rangetime: "<<m_range_image_ptr_list.back()->time<<std::endl;
        return true;
    }

    return false;
}

// Max always 1
inline float GaussianPointMotionClassi::GaussianWeight(double value, double sigma)
{   
    return (float)(exp(-0.5 * pow(value/sigma, 2)) + m_params.f_sigma_epsilon) / (1.0 + m_params.f_sigma_epsilon);
}

// Original Gaussian kernel
inline float GaussianPointMotionClassi::GaussianWeight2(double value, double sigma)
{   
    return (float)(1.0/(sigma * 2.5066) * exp(-0.5 * pow(value/sigma, 2)));
}

void GaussianPointMotionClassi::DempsterCombination(EvidType* EvidSrc, EvidType* EvidOther) {
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

void GaussianPointMotionClassi::CheckNeighbor(const point_soph & p, const RangeImage &image_info, float &max_range, float &min_range, float &occluded_max_range)
{   

    for (int i = -m_params.i_horizontal_neighbor_pixel; i <= m_params.i_horizontal_neighbor_pixel; i++)
    {
        for (int j = -m_params.i_vertical_neighbor_pixel; j <= m_params.i_vertical_neighbor_pixel; j++)
        {
            int cur_pos = (p.hor_ind + i) * m_i_max_1d_half + p.ver_ind + j;
            if( p.hor_ind + i >= 0 && p.hor_ind + i < m_i_max_1d && p.ver_ind + j >= 0 && p.ver_ind + j < m_i_max_1d_half && 
                cur_pos < m_i_max_2d_n && cur_pos >= 0 && image_info.range_image[cur_pos].size() > 0)
            {
                float cur_max_range = image_info.max_range_all[cur_pos];
                float cur_min_range = image_info.min_range_all[cur_pos];

                if(min_range > 10E-5) min_range = std::min(cur_min_range, min_range);
                else min_range = cur_min_range; 

                if(max_range > 10E-5){
                    max_range = std::max(cur_max_range, max_range);
                    if( abs(i) <= 1 && abs(j) <= 1) occluded_max_range = max_range;
                } 
                else{
                    max_range = cur_max_range;
                    if( abs(i) <= 1 && abs(j) <= 1) occluded_max_range = max_range;
                }
            }
        }
    }
}

void GaussianPointMotionClassi::OutputPmcPC(const std::vector<point_soph*> &points, const RangeImage::Ptr range_image_ptr)
{   
    double tout_start = omp_get_wtime();
    m_pmc_xyzrgb_pcptr.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

    // int i_points_size = points.size(); // point num

    int i_static_num = 0, i_dynamic_num = 0, i_unknown_num = 0;
    int p_r, p_g, p_b;
    
    int i_point_num = points.size();

    for(int i = 0; i < i_point_num; i++){
        pcl::PointXYZRGB po;
        po.x = points[i]->local[0];
        po.y = points[i]->local[1];
        po.z = points[i]->local[2];

        p_r = points[i]->r;
        p_g = points[i]->g;
        p_b = points[i]->b;

        if(m_params.b_output_discrete_label == true){
            if(p_b > max(p_r, p_g)){ // Unknown
                po.r = 0;
                po.g = 0;
                po.b = 255;
            }
            else if (p_g > p_r){ // Dynamic
                po.r = 0;
                po.g = 255;
                po.b = 0;    

                // if(points[i]->propagated){
                //     po.r = 255;
                //     po.g = 177;
                //     po.b = 190;
                // }           
            }
            else{ // Static
                po.r = 255;
                po.g = 0;
                po.b = 0;
            }   

            // if(points[i]->dyn == CASE2){
            //     po.r = 255;
            //     po.g = 255;
            //     po.b = 0;
            // }



        }
        else{
            po.r = p_r;
            po.g = p_g;
            po.b = p_b;
        }

        if(m_params.b_output_static_point == true && !(p_r > p_g && p_r > p_b)) continue;

        m_pmc_xyzrgb_pcptr->push_back(po);

    }

    // PMC pcptr
    // for(int i_ver_idx = 0; i_ver_idx < m_i_max_1d_half; i_ver_idx++) {
    //     for(int i_hor_idx = 0; i_hor_idx < m_i_max_1d; i_hor_idx++) {

    //         int iter_pos = i_hor_idx * m_i_max_1d_half + i_ver_idx;
    //         if(range_image_ptr->range_image[iter_pos].size() > 0){
    //             if(m_params.b_output_min_range == true){
    //                 int min_range_idx = range_image_ptr->min_range_index_all[iter_pos];
    //                 pcl::PointXYZRGB po;
    //                 po.x = range_image_ptr->range_image[iter_pos][min_range_idx]->local[0];
    //                 po.y = range_image_ptr->range_image[iter_pos][min_range_idx]->local[1];
    //                 po.z = range_image_ptr->range_image[iter_pos][min_range_idx]->local[2];

    //                 int r,g,b;


    //                 if(m_params.b_output_discrete_label == true){
    //                     p_r = range_image_ptr->range_image[iter_pos][min_range_idx]->r;
    //                     p_g = range_image_ptr->range_image[iter_pos][min_range_idx]->g;
    //                     p_b = range_image_ptr->range_image[iter_pos][min_range_idx]->b;
    //                     if(p_b > max(p_r, p_g)){
    //                         po.r = 0;
    //                         po.g = 0;
    //                         po.b = 255;
    //                     }
    //                     else if (p_g > p_r){
    //                         po.r = 0;
    //                         po.g = 255;
    //                         po.b = 0;               
    //                     }
    //                     else{
    //                         po.r = 255;
    //                         po.g = 0;
    //                         po.b = 0;
    //                     }   

    //                     if(range_image_ptr->range_image[iter_pos][min_range_idx]->dyn == CASE2){
    //                         po.r = 255;
    //                         po.g = 255;
    //                         po.b = 0;
    //                     }


    //                     if(range_image_ptr->range_image[iter_pos][min_range_idx]->propagated){
    //                         po.r = 255;
    //                         po.g = 177;
    //                         po.b = 190;
    //                     }
    //                 }
    //                 else{
    //                     po.r = range_image_ptr->range_image[iter_pos][min_range_idx]->r;
    //                     po.g = range_image_ptr->range_image[iter_pos][min_range_idx]->g;
    //                     po.b = range_image_ptr->range_image[iter_pos][min_range_idx]->b;

    //                     if(po.b > max(po.r, po.g)) i_unknown_num++;
    //                     else if (po.g > po.r) i_dynamic_num++;
    //                     else i_static_num++;
    //                 }


    //                 if(m_params.b_output_static_point == true && po.r < max(po.g, po.b)) continue;

    //                 m_pmc_xyzrgb_pcptr->push_back(po);
                    
    //             }
    //             else{
    //                 for(int pixel_idx = 0 ; pixel_idx < range_image_ptr->range_image[iter_pos].size() ; pixel_idx++){
    //                     pcl::PointXYZRGB po;
    //                     po.x = range_image_ptr->range_image[iter_pos][pixel_idx]->local[0];
    //                     po.y = range_image_ptr->range_image[iter_pos][pixel_idx]->local[1];
    //                     po.z = range_image_ptr->range_image[iter_pos][pixel_idx]->local[2];

    //                     int r,g,b;


    //                     if(m_params.b_output_discrete_label == true){
    //                         p_r = range_image_ptr->range_image[iter_pos][pixel_idx]->r;
    //                         p_g = range_image_ptr->range_image[iter_pos][pixel_idx]->g;
    //                         p_b = range_image_ptr->range_image[iter_pos][pixel_idx]->b;
    //                         if(p_b > max(p_r, p_g)){
    //                             po.r = 0;
    //                             po.g = 0;
    //                             po.b = 255;
    //                         }
    //                         else if (p_g > p_r){
    //                             po.r = 0;
    //                             po.g = 255;
    //                             po.b = 0;               
    //                         }
    //                         else{
    //                             po.r = 255;
    //                             po.g = 0;
    //                             po.b = 0;
    //                         }   
    //                     }
    //                     else{
    //                         po.r = range_image_ptr->range_image[iter_pos][pixel_idx]->r;
    //                         po.g = range_image_ptr->range_image[iter_pos][pixel_idx]->g;
    //                         po.b = range_image_ptr->range_image[iter_pos][pixel_idx]->b;

    //                         if(po.b > max(po.r, po.g)) i_unknown_num++;
    //                         else if (po.g > po.r) i_dynamic_num++;
    //                         else i_static_num++;
    //                     }


    //                     if(m_params.b_output_static_point == true && po.r < max(po.g, po.b)) continue;

    //                     m_pmc_xyzrgb_pcptr->push_back(po);
    //                 }
    //             }
    //         }
    //     }
    // }

    std::cout<<"Static: "<<i_static_num<<" Dynamic: "<<i_dynamic_num<<" Unknown: "<<i_unknown_num<<std::endl;
    std::cout<<"output pc num: "<<m_pmc_xyzrgb_pcptr->points.size()<<std::endl;

    // Key frame pcptr
    
    if(m_b_key_frame == true && m_params.b_debug_image){
        m_range_image_xyzrgb_pcptr->clear();
        
        m_deque_range_image_xyzrgb_pcptr[m_i_cur_point_soph_pointers]->clear();

        for(int i_ver_idx = 0; i_ver_idx < m_i_max_1d_half; i_ver_idx++) {
            for(int i_hor_idx = 0; i_hor_idx < m_i_max_1d; i_hor_idx++) {

                    int iter_pos = i_hor_idx * m_i_max_1d_half + i_ver_idx;
                    if(range_image_ptr->range_image[iter_pos].size() <= 0) continue;

                    int min_range_idx = range_image_ptr->min_range_index_all[iter_pos];
                    pcl::PointXYZRGB po;
                    po.x = range_image_ptr->range_image[iter_pos][min_range_idx]->glob[0];
                    po.y = range_image_ptr->range_image[iter_pos][min_range_idx]->glob[1];
                    po.z = range_image_ptr->range_image[iter_pos][min_range_idx]->glob[2];

                    int r,g,b;


                    if(m_params.b_output_discrete_label == true){
                        p_r = range_image_ptr->range_image[iter_pos][min_range_idx]->r;
                        p_g = range_image_ptr->range_image[iter_pos][min_range_idx]->g;
                        p_b = range_image_ptr->range_image[iter_pos][min_range_idx]->b;
                        if(p_b > max(p_r, p_g)){
                            po.r = 0;
                            po.g = 0;
                            po.b = 255;
                        }
                        else if (p_g > p_r){
                            po.r = 0;
                            po.g = 255;
                            po.b = 0;               
                        }
                        else{
                            po.r = 255;
                            po.g = 0;
                            po.b = 0;
                        }   
                    }
                    else{
                        po.r = range_image_ptr->range_image[iter_pos][min_range_idx]->r;
                        po.g = range_image_ptr->range_image[iter_pos][min_range_idx]->g;
                        po.b = range_image_ptr->range_image[iter_pos][min_range_idx]->b;
                    }


                    if(m_params.b_output_static_point == true && po.r < max(po.g, po.b)) continue;

                    m_deque_range_image_xyzrgb_pcptr[m_i_cur_point_soph_pointers]->push_back(po);
            }
        }

        for(auto range_image_xyzrgb_iter : m_deque_range_image_xyzrgb_pcptr){
            *m_range_image_xyzrgb_pcptr += *range_image_xyzrgb_iter;
        }
    }
    

    // Cluster pcptr

    // std::vector<std::vector<int>> sorted_cluster_poses_vec = range_image_ptr->cluster_poses_vec;
    // std::sort(sorted_cluster_poses_vec.begin(), sorted_cluster_poses_vec.end(), by_size_decent());
    // m_cluster_xyzrgb_pcptr->clear();

    // int cluster_num = sorted_cluster_poses_vec.size();
    // const int MAX_COLOR_VALUE = 255;
    // int i_hor_idx, i_ver_idx;

    // for(int cluster_idx = 0; cluster_idx < sorted_cluster_poses_vec.size(); cluster_idx++)
    // {
    //     int r = 0, g = 0, b = 0;

    //     // 무지개 색상의 비율을 계산합니다.
    //     float ratio = (float)cluster_idx / (cluster_num - 1);

    //     if (ratio < 0.2) { // 빨간색에서 주황색으로
    //         r = MAX_COLOR_VALUE;
    //         g = static_cast<int>(MAX_COLOR_VALUE * (ratio / 0.2));
    //         b = 0;
    //     } else if (ratio < 0.4) { // 주황색에서 노란색으로
    //         r = MAX_COLOR_VALUE;
    //         g = MAX_COLOR_VALUE;
    //         b = 0;
    //     } else if (ratio < 0.6) { // 노란색에서 초록색으로
    //         r = static_cast<int>(MAX_COLOR_VALUE * ((0.6 - ratio) / 0.2));
    //         g = MAX_COLOR_VALUE;
    //         b = 0;
    //     } else if (ratio < 0.8) { // 초록색에서 파란색으로
    //         r = 0;
    //         g = MAX_COLOR_VALUE;
    //         b = static_cast<int>(MAX_COLOR_VALUE * ((ratio - 0.6) / 0.2));
    //     } else { // 파란색에서 보라색으로
    //         r = static_cast<int>(MAX_COLOR_VALUE * ((ratio - 0.8) / 0.2));
    //         g = 0;
    //         b = MAX_COLOR_VALUE;
    //     }

    //     for (auto pos : sorted_cluster_poses_vec[cluster_idx])
    //     {   
    //         if(m_params.b_output_min_range == true){
    //             int min_range_pos = range_image_ptr->min_range_index_all[pos];
    //             if(min_range_pos >= 0 ){
    //                 pcl::PointXYZRGB po;
    //                 po.x = range_image_ptr->range_image[pos][min_range_pos]->local.x(); 
    //                 po.y = range_image_ptr->range_image[pos][min_range_pos]->local.y(); 
    //                 po.z = range_image_ptr->range_image[pos][min_range_pos]->local.z(); 
    //                 po.r = r;
    //                 po.g = g;
    //                 po.b = b;

    //                 m_cluster_xyzrgb_pcptr->push_back(po);    
    //             }
    //         }
    //         else{
    //             for (auto point_in_pixel : range_image_ptr->range_image[pos])
    //             {
    //                 pcl::PointXYZRGB po;
    //                 po.x = point_in_pixel->local.x(); 
    //                 po.y = point_in_pixel->local.y(); 
    //                 po.z = point_in_pixel->local.z(); 
    //                 po.r = r;
    //                 po.g = g;
    //                 po.b = b;

    //                 m_cluster_xyzrgb_pcptr->push_back(po);
    //             }
    //         }
    //     }
    // }

    double tout_end = omp_get_wtime() - tout_start;
    std::cout<<"[Gaussian PMC] Output Time: "<< tout_end*1000.0 << " ms"<<std::endl;
}

cv::Mat GaussianPointMotionClassi::GetRangeImageCv()
{
    cv::Mat mat_range_img((int)(m_i_pixel_fov_up - m_i_pixel_fov_down), (int)(m_i_pixel_fov_left - m_i_pixel_fov_right), CV_32F, cv::Scalar::all(FLT_MAX));

    if(m_range_image_ptr_list.size()>0){
        for(int i_ver_idx = 0; i_ver_idx < m_i_max_1d_half; i_ver_idx++) {
            for(int i_hor_idx = 0; i_hor_idx < m_i_max_1d; i_hor_idx++) {

                int iter_pos = i_hor_idx * m_i_max_1d_half + i_ver_idx;
                mat_range_img.at<float>(m_i_max_1d_half - i_ver_idx - 1, m_i_max_1d - i_hor_idx - 1) = m_range_image_ptr_list.back()->min_range_all[iter_pos];
            }
        }
    }

    // 채워진 cv::Mat 객체를 반환합니다.
    return mat_range_img;
}

cv::Mat GaussianPointMotionClassi::GetDynamicImageCv()
{
    cv::Mat mat_dynamic_img((int)(m_i_pixel_fov_up - m_i_pixel_fov_down), (int)(m_i_pixel_fov_left - m_i_pixel_fov_right), CV_8UC3, cv::Scalar::all(0));

    if(m_range_image_ptr_list.size() > 0){
        for(int i_ver_idx = 0; i_ver_idx < m_i_max_1d_half; i_ver_idx++) {
            for(int i_hor_idx = 0; i_hor_idx < m_i_max_1d; i_hor_idx++) {

                int iter_pos = i_hor_idx * m_i_max_1d_half + i_ver_idx;
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
                        case CASE2: // Yellow
                            r = 255;
                            g = 255;
                            b = 0;
                            break;
                        default: // Blue
                            r = 0;
                            g = 0;
                            b = 255;
                    }

                    if(m_range_image_ptr_list.back()->range_image[iter_pos][min_range_idx]->propagated){
                        r = 255;
                        g = 177;
                        b = 190;
                    }

                    mat_dynamic_img.at<cv::Vec3b>(m_i_max_1d_half - i_ver_idx - 1, m_i_max_1d - i_hor_idx - 1) = cv::Vec3b(r,g,b);
                }
                else{
                    mat_dynamic_img.at<cv::Vec3b>(m_i_max_1d_half - i_ver_idx - 1, m_i_max_1d - i_hor_idx - 1) = cv::Vec3b(0,0,0); // black
                }
            }
        }
    }

    // 채워진 cv::Mat 객체를 반환합니다.
    return mat_dynamic_img;
}

cv::Mat GaussianPointMotionClassi::GetIncidentImageCv()
{
    cv::Mat mat_incident_img((int)(m_i_pixel_fov_up - m_i_pixel_fov_down), (int)(m_i_pixel_fov_left - m_i_pixel_fov_right), CV_32F, cv::Scalar::all(-1));

    if(m_range_image_ptr_list.size()>0){
        for(int i_ver_idx = 0; i_ver_idx < m_i_max_1d_half; i_ver_idx++) {
            for(int i_hor_idx = 0; i_hor_idx < m_i_max_1d; i_hor_idx++) {
                
                int iter_pos = i_hor_idx * m_i_max_1d_half + i_ver_idx;
                if(m_range_image_ptr_list.back()->range_image[iter_pos].size() > 0){
                    int min_range_idx = m_range_image_ptr_list.back()->min_range_index_all[iter_pos];
                    mat_incident_img.at<float>(m_i_max_1d_half - i_ver_idx - 1, m_i_max_1d - i_hor_idx - 1) = 
                                m_range_image_ptr_list.back()->range_image[iter_pos][min_range_idx]->incident;

                }
            }
        }
    }

    // 채워진 cv::Mat 객체를 반환합니다.
    return mat_incident_img;
}


cv::Mat GaussianPointMotionClassi::GetClusterImageCv()
{
    cv::Mat mat_cluster_img((int)(m_i_pixel_fov_up - m_i_pixel_fov_down), (int)(m_i_pixel_fov_left - m_i_pixel_fov_right), CV_8UC3, cv::Scalar::all(0));

    if(m_range_image_ptr_list.size()>0){
        if(m_range_image_ptr_list.back()->cluster_poses_vec.size() == 0 ) return mat_cluster_img;
    
        std::vector<std::vector<int>> sorted_cluster_poses_vec = m_range_image_ptr_list.back()->cluster_poses_vec;
        // std::sort(sorted_cluster_poses_vec.begin(), sorted_cluster_poses_vec.end(), by_size_decent());

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
                i_hor_idx = pos / m_i_max_1d_half;
                i_ver_idx = pos % m_i_max_1d_half;

                mat_cluster_img.at<cv::Vec3b>(m_i_max_1d_half - i_ver_idx - 1, m_i_max_1d - i_hor_idx - 1) = cv::Vec3b(r,g,b);
            }
            
        }
    }

    return mat_cluster_img;
}

cv::Mat GaussianPointMotionClassi::GetGroundImageCv()
{
    cv::Mat mat_ground_img((int)(m_i_pixel_fov_up - m_i_pixel_fov_down), (int)(m_i_pixel_fov_left - m_i_pixel_fov_right), CV_8U, cv::Scalar::all(0));

    int ground_count = 0;
    int non_ground_count = 0;
    if(m_range_image_ptr_list.size()>0){
        for(int i_ver_idx = 0; i_ver_idx < m_i_max_1d_half; i_ver_idx++) {
            for(int i_hor_idx = 0; i_hor_idx < m_i_max_1d; i_hor_idx++) {
                
                int iter_pos = i_hor_idx * m_i_max_1d_half + i_ver_idx;
                if(m_range_image_ptr_list.back()->range_image[iter_pos].size() > 0){
                    int min_range_idx = m_range_image_ptr_list.back()->min_range_index_all[iter_pos];

                    if(m_range_image_ptr_list.back()->range_image[iter_pos][min_range_idx]->ground == true){
                        mat_ground_img.at<uint8_t>(m_i_max_1d_half - i_ver_idx - 1, m_i_max_1d - i_hor_idx - 1) = (uint8_t)2;
                        ground_count++;
                    }
                    else{
                        mat_ground_img.at<uint8_t>(m_i_max_1d_half - i_ver_idx - 1, m_i_max_1d - i_hor_idx - 1) = (uint8_t)1;
                        non_ground_count++;
                    }
                        
                }
            }
        }
    }

    std::cout<<"GetGroundImageCv ground: "<<ground_count<<" non: "<<non_ground_count<<std::endl;

    return mat_ground_img;
}


void GaussianPointMotionClassi::GetFilteredPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& o_pmc_xyzrgb_pcptr)
{   
    *o_pmc_xyzrgb_pcptr = *m_pmc_xyzrgb_pcptr;
}

void GaussianPointMotionClassi::GetKeyFramePoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& o_key_frame_xyzrgb_pcptr)
{   
    *o_key_frame_xyzrgb_pcptr = *m_range_image_xyzrgb_pcptr;
}

void GaussianPointMotionClassi::GetClusterPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& o_cluster_xyzrgb_pcptr)
{   
    *o_cluster_xyzrgb_pcptr = *m_cluster_xyzrgb_pcptr;
}


