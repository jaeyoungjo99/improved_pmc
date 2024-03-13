#include "range_pmc.hpp"

RangePmc::RangePmc(){};
RangePmc::~RangePmc(){};

void RangePmc::Init(range_pmc_params params){
    cout<<"[Range PMC] Init Started"<<endl;

    params_ = params;
    cout<<"f_horizontal_resolution: "<<params_.f_horizontal_resolution<<endl;
    cout<<"f_vertical_resolution: "<<params_.f_vertical_resolution<<endl;
    cout<<"f_min_range: "<<params_.f_min_range<<endl;
    cout<<"f_max_range: "<<params_.f_max_range<<endl;
    cout<<"f_range_threshold: "<<params_.f_range_threshold<<endl;
    cout<<"i_max_range_image_num: "<<params_.i_max_range_image_num<<endl;
    cout<<"f_min_key_frame_time: "<<params_.f_min_key_frame_time<<endl;
    cout<<"f_min_key_frame_rot: "<<params_.f_min_key_frame_rot<<endl;
    cout<<"f_fov_up: "<<params_.f_fov_up<<endl;
    cout<<"f_fov_down: "<<params_.f_fov_down<<endl;
    cout<<"f_fov_left: "<<params_.f_fov_left<<endl;
    cout<<"f_fov_right: "<<params_.f_fov_right<<endl;
    cout<<"i_min_occluded_num: "<<params_.i_min_occluded_num<<endl;


    pmc_xyzrgb_pcptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    range_image_xyzrgb_pcptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    cluster_xyzrgb_pcptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    temp_image_pointer.reset(new RangeImage());

    

    pixel_fov_up    = floor((params_.f_fov_up   / 180.0*M_PI + 0.5 * M_PI)  /params_.f_vertical_resolution);
    pixel_fov_down  = floor((params_.f_fov_down / 180.0*M_PI + 0.5 * M_PI)  /params_.f_vertical_resolution);
    pixel_fov_left  = floor((params_.f_fov_left / 180.0*M_PI +  M_PI)       /params_.f_horizontal_resolution);
    pixel_fov_right = floor((params_.f_fov_right/ 180.0*M_PI +  M_PI)       /params_.f_horizontal_resolution);

    m_i_row_size        = pixel_fov_up - pixel_fov_down;
    m_i_col_size        = pixel_fov_left - pixel_fov_right;
    max_pointers_num    = params_.i_max_range_image_num;
    point_soph_pointers.reserve(max_pointers_num); 

    cout<<"pixel_fov_up: "<<pixel_fov_up<<endl;
    cout<<"pixel_fov_down: "<<pixel_fov_down<<endl;
    cout<<"pixel_fov_left: "<<pixel_fov_left<<endl;
    cout<<"pixel_fov_right: "<<pixel_fov_right<<endl;


    is_key_frame = false;

    for (int i = 0; i < max_pointers_num; i++)
    {
        point_soph* p = new point_soph[MAX_POINT]; // 비어있는 point_soph 배열 생성 
        point_soph_pointers.push_back(p); // 메모리 할당 
    }

    temp_point_soph_pointer = new point_soph[MAX_POINT];

    for (int i = 0; i < max_pointers_num; i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr iter_ptr;
        iter_ptr.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        range_image_xyzrgb_pcptr_deque_.push_back(iter_ptr);
    }


    NeighborAssign(2, 3);

    
    cout<<"[Range PMC] Init Done"<<endl;

}


void RangePmc::Filter(PointCloudXYZI::Ptr feats_undistort, const M3D & rot_end, const V3D & pos_end, const double & scan_end_time)
{   

    
    double t00 = omp_get_wtime();
    cout<<"[Range PMC] Filter Started"<<endl;

    is_key_frame = KeyFrameCheck(scan_end_time, rot_end, pos_end);


    int size = feats_undistort->points.size(); // point num

    cout<<"[Range PMC] size"<< size << endl;

    std::vector<int> index(size); // Point Index 할당 
    for (int i = 0; i < size; i++) {
        index[i] = i;
    }

    pmc_xyzrgb_pcptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

    vector<point_soph*> points; // point_soph의 주소들을 저장하기 위한 벡터 
    points.reserve(size);
    points.resize(size);


    point_soph* p;  // point_soph 포인트 배열 
    if(is_key_frame == true){
        p = point_soph_pointers[cur_point_soph_pointers]; // 현재 range image내  point_soph들 메모리 불러옴. 초기화 할 것임 
    }
    else{
        p = temp_point_soph_pointer; // 임시 포인터. point_soph의 array임
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

        points[i] = &p[i]; // p[i] 의 주소를 저장함
    }

    temp_image_pointer->Reset(rot_end, pos_end, scan_end_time, map_index);

    RangeImage::Ptr new_image_pointer(new RangeImage(rot_end, pos_end, scan_end_time, map_index)); // 신규 range image pointer


    // 이때 p는 keyframe이면 순환되는 주소 point_soph_pointers[cur_point_soph_pointers]
    // key frame이 아니면 temp_point_soph_pointer 
    // 이 함수에선 p의 주소를 new_image_pointer에 push_back 함 
    i_valid_position_num = 0;
    GenerateRangeImage(points, scan_end_time, rot_end, pos_end, new_image_pointer);
    GroundSegmentation(new_image_pointer);

    if(params_.b_cluster_level_filtering == true)
        ObjectSegmentation(new_image_pointer);

    std::cout<<"i_valid_position_num: "<<i_valid_position_num<<std::endl;
    std::cout<<"Cluster size: "<<new_image_pointer->cluster_poses_vec.size()<<" idx: "<< i_cluster_idx<<std::endl;
    
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
    bool is_dyn;
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
        if (params_.b_cluster_level_filtering == true){
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
                i_skip_size = sqrt(p_cluster_size) / 4;
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
    if(is_key_frame == true){

        if(range_image_list.size() < params_.i_max_range_image_num){
            map_index++;
            range_image_list.push_back(new_image_pointer);
        }
        else{
            map_index++;
            range_image_list.pop_front();
            range_image_list.push_back(new_image_pointer); // 벡터의 요소가 가리키는 포인터 메모리 교체 
        }
        cur_point_soph_pointers = (cur_point_soph_pointers + 1) % max_pointers_num; // 다음 번 range image 의 index를 지정함 
    }


    std::cout<<"[Range PMC] Image num: "<< range_image_list.size() <<std::endl;

    // Output
    OutputPmcPC(points, new_image_pointer);


    time_total = omp_get_wtime() - t00;
    std::cout<<"[Range PMC] Total Time: "<< time_total*1000.0 << " ms"<<std::endl;
}

void RangePmc::Case1(point_soph & p, RangeImage::Ptr range_image_ptr)
{
    int range_image_num = range_image_list.size();
    int valid_image = 0;
    float gaussian_weight;

    dyn_obj_flg p_dyn;

    for (int i = range_image_num- 1; i >= 0; i--) // point 한개에 대해 range image 탐색. 최근 range image 부터 
    {   
        // p에는 glob이 있어야하고, 이 함수를 통해서 vec, hor_ind, ver_ind, position 가 range_image_list[i] 기준으로 바뀜 
        SphericalProjection(p, range_image_list[i]->map_index, range_image_list[i]->project_R, range_image_list[i]->project_T, p);

        if (p.ver_ind > pixel_fov_up || p.ver_ind < pixel_fov_down || 
            p.hor_ind > pixel_fov_left || p.hor_ind < pixel_fov_right || p.vec(2) < 0.0f \
            || p.position < 0 || p.position >= MAX_2D_N)
        {
            continue; // range image의 경계를 넘은 point는 iter range image와 맞지 않음 skip
        }

        if (Case1Enter(p, *range_image_list[i], p_dyn) == true)
        {   
            valid_image++;
        }
        
        if (p_dyn == CASE2){
            // TODO 현재 포인트가 직전 range point보다 먼 경우
            p_dyn = UNKNOWN; // TODO
        }

        // Dempster
        EvidType SrcEvidence[3] = {(EvidType)p.r, (EvidType)p.g, (EvidType)p.b};
        if(p_dyn == CASE1){
            
            EvidType MovingEvidence[3] = {0, (EvidType)(params_.f_moving_confidence * MAX_EVIDENCE),
                                            (EvidType)((1. - params_.f_moving_confidence) * MAX_EVIDENCE)};
            
            DempsterCombination(SrcEvidence, MovingEvidence);

            p.r = (uint8_t)SrcEvidence[0];
            p.g = (uint8_t)SrcEvidence[1];
            p.b = (uint8_t)SrcEvidence[2];
        }
        else if (p_dyn == STATIC){
            EvidType MovingEvidence[3] = {(EvidType)(params_.f_static_confidence * MAX_EVIDENCE), 0,
                                            (EvidType)((1. - params_.f_static_confidence) * MAX_EVIDENCE)};

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
    if (params_.b_cluster_level_filtering == true && p.cluster_ind >= 0){
        double t_cluster_labeling_start = omp_get_wtime();
        double t_demster_start, t_gaussian_start;

        for (const auto pixel_pos : range_image_ptr->cluster_poses_vec[p.cluster_ind])
        {
            if (pixel_pos < MAX_2D_N && pixel_pos >= 0 ){
                t_gaussian_start = omp_get_wtime();
                int min_pos = range_image_ptr->min_range_index_all[pixel_pos];
                
                // Get Gaussian weight
                if(p.dyn == CASE1)
                    gaussian_weight = GaussianWeight((p.local - range_image_ptr->range_image[pixel_pos][min_pos]->local).norm(), params_.f_dynamic_gaussian_sigma);
                else
                    gaussian_weight = GaussianWeight((p.local - range_image_ptr->range_image[pixel_pos][min_pos]->local).norm(), params_.f_static_gaussian_sigma);
                // gaussian_weight = 1;
                i_gaussian++;
                time_gaussian += omp_get_wtime() - t_gaussian_start;

                if(gaussian_weight < 0.1) continue;

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
                    
                    EvidType MovingEvidence[3] = {0, (EvidType)(params_.f_moving_confidence * gaussian_weight * MAX_EVIDENCE),
                                                  (EvidType)((1. - params_.f_moving_confidence * gaussian_weight) * MAX_EVIDENCE)};
                    
                    DempsterCombination(SrcEvidence, MovingEvidence);

                    range_image_ptr->range_image[pixel_pos][min_pos]->r = (uint8_t)SrcEvidence[0];
                    range_image_ptr->range_image[pixel_pos][min_pos]->g = (uint8_t)SrcEvidence[1];
                    range_image_ptr->range_image[pixel_pos][min_pos]->b = (uint8_t)SrcEvidence[2];
                }
                else if (p.dyn == STATIC){
                    EvidType MovingEvidence[3] = {(EvidType)(params_.f_static_confidence * gaussian_weight * MAX_EVIDENCE), 0,
                                                  (EvidType)((1. - params_.f_static_confidence * gaussian_weight) * MAX_EVIDENCE)};

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
    float max_range = 0, min_range = 0;
    p_dyn = UNKNOWN;

    max_range = image_info.max_range_all[p.position];
    min_range = image_info.min_range_all[p.position];

    if(p.ver_ind <= pixel_fov_up && p.ver_ind > pixel_fov_down && \
        p.hor_ind <= pixel_fov_left && p.hor_ind >= pixel_fov_right
        )
    {
        CheckNeighbor(p, image_info, max_range, min_range);
    }
 
    if(min_range < 10E-5){ // 비교할 포인트를 찾지 못함 
        return false;
    }

    int map_ind_gap = map_index - image_info.map_index;
    if(map_ind_gap < 0) map_ind_gap += params_.i_max_range_image_num;
    
    if (p.vec(2) < min_range - params_.f_range_threshold ){ // if point is closer than min range
        p_dyn = CASE1;
        return true;
    }
    else if (p.vec(2) < max_range + params_.f_range_threshold){ // if point is Static
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
    int len = points.size();
    std::cout<<"[GenerateRangeImage] Started"<<std::endl;
    std::cout<<"[GenerateRangeImage] cur_point_soph_pointers: "<<cur_point_soph_pointers<<std::endl;
    std::cout<<"[GenerateRangeImage] map_index: "<<map_index<<std::endl;

    int i_invalid_num = 0;
    float min_ver_angle = FLT_MAX;
    float max_ver_angle = -FLT_MAX;
    for (int k = 0; k < len; k++)
    { 
        points[k]->GetVec(points[k]->local, params_.f_horizontal_resolution, params_.f_vertical_resolution);

        if (points[k]->vec(1) < min_ver_angle) min_ver_angle = points[k]->vec(1);
        if (points[k]->vec(1) > max_ver_angle) max_ver_angle = points[k]->vec(1);
        
        // FOV를 벗어난 point는 range image에 할당하지 않음
        if(points[k]->ver_ind > pixel_fov_up || points[k]->ver_ind < pixel_fov_down || \
           points[k]->hor_ind > pixel_fov_left || points[k]->hor_ind < pixel_fov_right || \
           points[k]->position < 0 || points[k]->position >= MAX_2D_N)
        {
            points[k]->dyn = INVALID;
            i_invalid_num++;
            continue;
        }

        if(range_image_ptr->range_image[points[k]->position].size() < max_pixel_points) // 해당 픽셀에 여유가 있다면 
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

    std::cout<<"[GenerateRangeImage] min ver angle: "<<min_ver_angle*180/M_PI<< " max: "<< max_ver_angle*180/M_PI << std::endl;
    std::cout<<"[GenerateRangeImage] Invalid Num: "<<i_invalid_num<<" / "<<len<<std::endl;
    double time_generate_range_image = omp_get_wtime() - t00;
    std::cout<<"[GenerateRangeImage] Total Time: "<< time_generate_range_image*1000.0 << " ms"<<std::endl;
}

// Segment Ground from Range Image
void RangePmc::GroundSegmentation(RangeImage::Ptr range_image_ptr)
{   
    double t00 = omp_get_wtime();

    V3D u_point, l_point;
    float v_angle, i_angle, hor_i_angle, ver_i_angle;
    float hor_dist, ver_dist;
    float hor_dist_ground;
    float u_range, l_range;
    float ver_angle_diff, hor_angle_diff, ver_angle_from_ground;
    float ground_margin_angle;

    int u_pos, l_pos, u_point_min_pos, l_point_min_pos;

    for(int hor_idx = pixel_fov_right; hor_idx < pixel_fov_left; hor_idx++)
    {
        for(int ver_idx = pixel_fov_down; ver_idx < pixel_fov_up; ver_idx++)  // vertical은 하나 적게 올라감
        {   
            l_pos = hor_idx * MAX_1D_HALF + ver_idx;
            l_point_min_pos = range_image_ptr->min_range_index_all[l_pos];

            if(l_point_min_pos == -1 || range_image_ptr->range_image[l_pos].size() == 0) continue; // Check lower point valid

            l_point = range_image_ptr->range_image[l_pos][l_point_min_pos]->local;
            l_range = range_image_ptr->range_image[l_pos][l_point_min_pos]->vec(2);

            for (int u_ver_idx = ver_idx + 1; u_ver_idx < ver_idx + 20 && u_ver_idx < pixel_fov_up ; u_ver_idx++){
                
                u_pos = hor_idx * MAX_1D_HALF + u_ver_idx;
                u_point_min_pos = range_image_ptr->min_range_index_all[u_pos];

                u_point = range_image_ptr->range_image[u_pos][u_point_min_pos]->local;
                u_range = range_image_ptr->range_image[u_pos][u_point_min_pos]->vec(2);

                if(u_point_min_pos == -1 || range_image_ptr->range_image[u_pos].size() == 0) continue; // Check upper point valid

                ver_angle_diff = range_image_ptr->range_image[u_pos][u_point_min_pos]->vec(1) 
                               - range_image_ptr->range_image[l_pos][l_point_min_pos]->vec(1);

                hor_dist = sqrt(pow(u_point.x() - l_point.x(), 2) + pow(u_point.y() - l_point.y(), 2));
                ver_dist = u_point.z() - l_point.z();

                hor_dist_ground = sqrt(l_point.x() * l_point.x() + l_point.y() * l_point.y());
                ver_angle_from_ground = atan2f(l_point.z() + 1.6, hor_dist_ground);
                ground_margin_angle = atan2f(- l_point.z() - 1.1, hor_dist_ground);

                v_angle = atan2f(ver_dist,hor_dist);
                
                if( u_range*cos(ver_angle_diff) - l_range > 0 )
                    ver_i_angle = M_PI/2 - atan2f(u_range*sin(ver_angle_diff), u_range*cos(ver_angle_diff) - l_range);
                else
                    ver_i_angle = M_PI/2 - atan2f(u_range*sin(ver_angle_diff), - u_range*cos(ver_angle_diff) + l_range);     

                i_angle = ver_i_angle;

                range_image_ptr->range_image[l_pos][l_point_min_pos]->incident = i_angle;
                range_image_ptr->incident_all[l_pos] = i_angle;

                if(fabs(v_angle) < params_.f_ground_angle * M_PI/180.0  && ver_angle_from_ground < params_.f_ground_angle * M_PI/180.0){
                    range_image_ptr->range_image[u_pos][u_point_min_pos]->ground = true;
                    range_image_ptr->ground_all[u_pos] = true;
                }
                if( ground_margin_angle > params_.f_ground_angle * M_PI/180.0 ){
                    range_image_ptr->range_image[l_pos][l_point_min_pos]->ground = true;
                    range_image_ptr->ground_all[l_pos] = true;
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

    int from_pos, from_min_pos;

    i_cluster_idx = 0;
    for(int idx_row = pixel_fov_down ; idx_row < pixel_fov_up ; ++idx_row)
    {
        for(int idx_col = pixel_fov_right ; idx_col < pixel_fov_left ; ++idx_col)
        {   
            from_pos = idx_col * MAX_1D_HALF + idx_row;
            from_min_pos = range_image_ptr->min_range_index_all[from_pos];

            if(range_image_ptr->range_image[from_pos].size() > 0 &&
               range_image_ptr->ground_all[from_pos] == false &&
               range_image_ptr->range_image[from_pos][from_min_pos]->cluster_ind == -1)
            {
                LabelComponents(range_image_ptr, idx_row, idx_col);
            }
        }   
    }
    std::cout<<"[ObjectSegmentation] Total segment: "<< i_cluster_idx <<std::endl;

    double object_total = omp_get_wtime() - t00;
    std::cout<<"[ObjectSegmentation] Total Time: "<< object_total*1000.0 << " ms"<<std::endl;
}


void RangePmc::NeighborAssign(unsigned int hor_neighbor, unsigned int ver_heighbor)
{
    m_v_neighbor_iter.clear();
    std::pair<int8_t, int8_t> neighbor;
    
    for(int hor_ind = 1; hor_ind <= hor_neighbor; hor_ind++ ){
        for(int ver_ind = 1; ver_ind <= ver_heighbor; ver_ind++ ){
            neighbor.first = -hor_ind;  neighbor.second =  0;       m_v_neighbor_iter.push_back(neighbor);
            neighbor.first =  0;        neighbor.second =  ver_ind; m_v_neighbor_iter.push_back(neighbor);
            neighbor.first =  0;        neighbor.second = -ver_ind; m_v_neighbor_iter.push_back(neighbor);
            neighbor.first =  hor_ind;  neighbor.second =  0;       m_v_neighbor_iter.push_back(neighbor);
        }
    }
}

void RangePmc::LabelComponents(RangeImage::Ptr range_image_ptr, uint16_t row, uint16_t col)
{   
    // row = Y = ver_ind = fov up down
    // col = X = hor_ind = fov left right

    float f_longer_range_m, dist_threshold_m;
    int fromIndX, fromIndY, thisIndX, thisIndY;
    std::vector<bool> lineCountFlag(m_i_row_size,false);
    Pixels cur_label_pixels;

    int queueSize = 1;
    int queueStartInd = 0;
    int queueEndInd = 1;

    m_v_ui16_queue_idx_x[0] = col;
    m_v_ui16_queue_idx_y[0] = row;

    int allPushedIndSize = 1;

    float f_distance_btw_point_m;

    int this_pos, this_min_pos;
    int from_pos, from_min_pos;

    std::vector<int> cluster_pos_vec;

    std::vector<std::pair<int, int>> pos_min_pos_vec; // (position, min_range_position) vec

    while(queueSize > 0)
    {
        // Pop point
        fromIndX = m_v_ui16_queue_idx_x[queueStartInd];
        fromIndY = m_v_ui16_queue_idx_y[queueStartInd];
        --queueSize;
        ++queueStartInd;

        from_pos = fromIndX * MAX_1D_HALF + fromIndY;
        from_min_pos = range_image_ptr->min_range_index_all[from_pos];

        // 해당 픽셀에 포인트가 없거나, ground이면 skip
        if (range_image_ptr->range_image[from_pos].size() == 0 ||
            range_image_ptr->ground_all[from_pos] == true)
            continue;

        range_image_ptr->range_image[from_pos][from_min_pos]->cluster_ind = -2; // 검색 되었음의 의미로 -2 할당

        pos_min_pos_vec.push_back(make_pair(from_pos, from_min_pos));

        // Loop through all the neighboring grids of popped grid
        for (auto iter = m_v_neighbor_iter.begin(); iter != m_v_neighbor_iter.end(); ++iter)
        {
            // new index
            thisIndX = fromIndX + (*iter).first; // Horizontal Direction
            thisIndY = fromIndY + (*iter).second; // Vertical Direction

            // Index should be within the boundary.
            if (thisIndY < pixel_fov_down || thisIndY >= pixel_fov_up)
                continue;

            // Check continuity of col idx
            if (thisIndX < pixel_fov_right) thisIndX = thisIndX + m_i_col_size;
            if (thisIndX >= pixel_fov_left) thisIndX = thisIndX - m_i_col_size;

            this_pos = thisIndX * MAX_1D_HALF + thisIndY;
            this_min_pos = range_image_ptr->min_range_index_all[this_pos];


            // Check current pixel is already segmented or ground
            if (range_image_ptr->range_image[this_pos].size() == 0 ||
                range_image_ptr->ground_all[this_pos] == true ||
                range_image_ptr->range_image[this_pos][this_min_pos]->cluster_ind != -1)
                continue;            

            f_distance_btw_point_m = (range_image_ptr->range_image[from_pos][from_min_pos]->local - 
                                      range_image_ptr->range_image[this_pos][this_min_pos]->local).norm();


            dist_threshold_m = params_.f_dist_threshold_m; // Debug


            // Check current pixel is same segment
            if (f_distance_btw_point_m < dist_threshold_m)
            {
                m_v_ui16_queue_idx_x[queueEndInd] = thisIndX;
                m_v_ui16_queue_idx_y[queueEndInd] = thisIndY;
                ++queueSize;
                ++queueEndInd;

                range_image_ptr->range_image[this_pos][this_min_pos]->cluster_ind = -2; // 탐색 되었음 라벨링 

                lineCountFlag[thisIndY - pixel_fov_down] = true;

                pos_min_pos_vec.push_back(make_pair(this_pos, this_min_pos));

                ++allPushedIndSize;
            }
        }
    }
    

    // check if this segment is valid

    bool feasibleSegment = false;
    if (allPushedIndSize >= params_.i_segment_min_point_num)
        feasibleSegment = true;
    else if (allPushedIndSize >= params_.i_segment_valid_point_num)
    {
        int lineCount = 0;
        for (int i = 0; i < m_i_row_size; ++i)
            if (lineCountFlag[i] == true)
                ++lineCount;

        if (lineCount >= params_.i_segment_valid_line_num)
            feasibleSegment = true;
    }

    // segment is valid, mark these points
    if (feasibleSegment == true && pos_min_pos_vec.size() > 0)
    {
        for (auto pos_min_pos : pos_min_pos_vec){
            range_image_ptr->range_image[pos_min_pos.first][pos_min_pos.second]->cluster_ind = i_cluster_idx;
            range_image_ptr->cluster_idx_all[pos_min_pos.first] = i_cluster_idx;
            cluster_pos_vec.push_back(pos_min_pos.first); // pos 
        }

        range_image_ptr->cluster_poses_vec.push_back(cluster_pos_vec);
        i_cluster_idx++;
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
    V3D p_proj(rot.inverse()*(p.glob - transl)); // 현재 range image 좌표계로 이전됨 p의 위치 

    // GetVec을 통해 vec과 ind, position이 할당됨
    p_spherical.GetVec(p_proj, params_.f_horizontal_resolution, params_.f_vertical_resolution);
}

bool RangePmc::KeyFrameCheck(double cur_time, M3D rot, V3D transl)
{
    if(range_image_list.size() == 0) return true;

    Eigen::Matrix3d relative_rotation =  range_image_list.back()->project_R.transpose() * rot;
    Eigen::AngleAxisd angle_axis(relative_rotation);

    std::cout.precision(15);
    std::cout<<"cur_time: "<<cur_time<<" rangetime: "<<range_image_list.back()->time<<std::endl;

    if(cur_time - range_image_list.back()->time > params_.f_min_key_frame_time ||
       (range_image_list.back()->project_T - transl).norm() > params_.f_min_key_frame_rot ||
       (angle_axis.angle() > params_.f_min_key_frame_rot))
    {
        return true;
    }

    return false;
}

inline float RangePmc::GaussianWeight(double value, double sigma)
{   
    // return - value / (sigma * 2.5) + 1.0;

    // return (float)exp(-0.5 * pow((p1-p2).norm()/params_.f_gaussian_sigma, 2));
    return (float)(exp(-0.5 * pow(value/sigma, 2)) + 0.1) / 1.1;
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
    bool ver_up = false, ver_down = false;
    for(int i = p.ver_ind; i >= pixel_fov_down; i--) // 현재 포인트부터 image 하단까지 유효 pixel 있는지 확인
    {   
        int cur_pos = p.hor_ind * MAX_1D_HALF + i;
        if(image_info.range_image[cur_pos].size() > 0)
        {
            ver_down = true;
            break;
        }
    } 
    for(int i = p.ver_ind; i <= pixel_fov_up; i++) // 현재 포인트부터 image 상단까지 유효 pixel 있는지 확인
    {   
        int cur_pos = p.hor_ind * MAX_1D_HALF + i;
        if(image_info.range_image[cur_pos].size() > 0)
        {
            ver_up = true;
            break;
        }
    }   
    if(ver_up && ver_down)
    {
        return false;
    }
    else
    {
        return true;
    }
}

bool RangePmc::CheckNeighbor(const point_soph & p, const RangeImage &image_info, float &max_range, float &min_range)
{   
    int n = params_.i_neighbor_pixel_max;

    // TODO Adaptive
    // if(p.incident < 30 * M_PI/180.0){
    //     n = 1;
    //     i_small_incident++;
    // }
    // else{
    //     i_total_incident++;
    // }
    if(image_info.incident_all[p.position] < 30 * M_PI/180.0){
        n = 2;
        i_small_incident++;
    }
    else{
        i_total_incident++;
    }

    bool is_valid = false;
    for (int i = -n; i <= n; i++)
    {
        for (int j = -n; j <= n; j++)
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

    int size = points.size(); // point num

    int p_r, p_g, p_b;

    // PMC pcptr
    // for(int i = 0; i < size; i++){
    //     pcl::PointXYZRGB po;
    //     po.x = points[i]->local[0];
    //     po.y = points[i]->local[1];
    //     po.z = points[i]->local[2]; 

    //     // 현재 포인트의 거리가 최소거리보다 크다면 (대표값이 아니라면)
    //     if(params_.b_output_min_range == true && 
    //        points[i]->vec(2) > range_image_ptr->min_range_all[points[i]->position] + 10E-5)
    //         continue;

    //     if(params_.b_output_static_point == true && 
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

    //     pmc_xyzrgb_pcptr_->push_back(po);
    // }



    for(int ver_idx = pixel_fov_down; ver_idx < pixel_fov_up; ver_idx++) {
        for(int hor_idx = pixel_fov_right; hor_idx < pixel_fov_left; hor_idx++) {

            int iter_pos = hor_idx * MAX_1D_HALF + ver_idx;
            if(range_image_ptr->range_image[iter_pos].size() > 0){
                if(params_.b_output_min_range == true){
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

                    pmc_xyzrgb_pcptr_->push_back(po);
                    
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
                        pmc_xyzrgb_pcptr_->push_back(po);
                    }
                }

            }

        }
    }

    std::cout<<"output pc num: "<<pmc_xyzrgb_pcptr_->points.size()<<std::endl;

    // Key frame pcptr
    if(is_key_frame == true){
        range_image_xyzrgb_pcptr_->clear();
        
        range_image_xyzrgb_pcptr_deque_[cur_point_soph_pointers]->clear();

        for(int i = 0; i < size; i++){
            pcl::PointXYZRGB po;
            po.x = points[i]->glob[0];
            po.y = points[i]->glob[1];
            po.z = points[i]->glob[2];

            if(params_.b_output_static_point == true &&
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

            range_image_xyzrgb_pcptr_deque_[cur_point_soph_pointers]->push_back(po);
        }


        for(auto range_image_xyzrgb_iter : range_image_xyzrgb_pcptr_deque_){
            *range_image_xyzrgb_pcptr_ += *range_image_xyzrgb_iter;
        }
    }

    // Cluster pcptr

    std::vector<std::vector<int>> sorted_cluster_poses_vec = range_image_ptr->cluster_poses_vec;
    std::sort(sorted_cluster_poses_vec.begin(), sorted_cluster_poses_vec.end(), by_size_decent());
    cluster_xyzrgb_pcptr_->clear();

    int cluster_num = sorted_cluster_poses_vec.size();
    const int MAX_COLOR_VALUE = 255;
    int hor_idx, ver_idx;

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
            if(params_.b_output_min_range == true){
                int min_range_pos = range_image_ptr->min_range_index_all[pos];
                if(min_range_pos >= 0 ){
                    pcl::PointXYZRGB po;
                    po.x = range_image_ptr->range_image[pos][min_range_pos]->local.x(); 
                    po.y = range_image_ptr->range_image[pos][min_range_pos]->local.y(); 
                    po.z = range_image_ptr->range_image[pos][min_range_pos]->local.z(); 
                    po.r = r;
                    po.g = g;
                    po.b = b;

                    cluster_xyzrgb_pcptr_->push_back(po);    
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

                    cluster_xyzrgb_pcptr_->push_back(po);
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
    cv::Mat mat_range_img((int)(pixel_fov_up - pixel_fov_down), (int)(pixel_fov_left - pixel_fov_right), CV_32F, cv::Scalar::all(FLT_MAX));

    if(range_image_list.size()>0){
        for(int ver_idx = pixel_fov_down; ver_idx < pixel_fov_up; ver_idx++) {
            for(int hor_idx = pixel_fov_right; hor_idx < pixel_fov_left; hor_idx++) {

                int iter_pos = hor_idx * MAX_1D_HALF + ver_idx;
                mat_range_img.at<float>(pixel_fov_up - ver_idx - 1, pixel_fov_left - hor_idx - 1) = range_image_list.back()->min_range_all[iter_pos];
            }
        }
    }

    // 채워진 cv::Mat 객체를 반환합니다.
    return mat_range_img;
}

cv::Mat RangePmc::GetDynamicImageCv()
{
    cv::Mat mat_dynamic_img((int)(pixel_fov_up - pixel_fov_down), (int)(pixel_fov_left - pixel_fov_right), CV_8UC3, cv::Scalar::all(0));

    if(range_image_list.size() > 0){
        for(int ver_idx = pixel_fov_down; ver_idx < pixel_fov_up; ver_idx++) {
            for(int hor_idx = pixel_fov_right; hor_idx < pixel_fov_left; hor_idx++) {

                int iter_pos = hor_idx * MAX_1D_HALF + ver_idx;
                if(range_image_list.back()->range_image[iter_pos].size() > 0){
                    int min_range_idx = range_image_list.back()->min_range_index_all[iter_pos];

                    int r,g,b;
                    switch (range_image_list.back()->range_image[iter_pos][min_range_idx]->dyn)
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

                    mat_dynamic_img.at<cv::Vec3b>(pixel_fov_up - ver_idx - 1, pixel_fov_left - hor_idx - 1) = cv::Vec3b(r,g,b);
                }
                else{
                    mat_dynamic_img.at<cv::Vec3b>(pixel_fov_up - ver_idx - 1, pixel_fov_left - hor_idx - 1) = cv::Vec3b(0,0,0); // black
                }
            }
        }
    }

    // 채워진 cv::Mat 객체를 반환합니다.
    return mat_dynamic_img;
}

cv::Mat RangePmc::GetIncidentImageCv()
{
    cv::Mat mat_incident_img((int)(pixel_fov_up - pixel_fov_down), (int)(pixel_fov_left - pixel_fov_right), CV_32F, cv::Scalar::all(-1));

    if(range_image_list.size()>0){
        for(int ver_idx = pixel_fov_down; ver_idx < pixel_fov_up; ver_idx++) {
            for(int hor_idx = pixel_fov_right; hor_idx < pixel_fov_left; hor_idx++) {
                
                int iter_pos = hor_idx * MAX_1D_HALF + ver_idx;
                if(range_image_list.back()->range_image[iter_pos].size() > 0){
                    int min_range_idx = range_image_list.back()->min_range_index_all[iter_pos];
                    mat_incident_img.at<float>(pixel_fov_up - ver_idx - 1, pixel_fov_left - hor_idx - 1) = 
                                range_image_list.back()->range_image[iter_pos][min_range_idx]->incident;

                }
            }
        }
    }

    // 채워진 cv::Mat 객체를 반환합니다.
    return mat_incident_img;
}


cv::Mat RangePmc::GetClusterImageCv()
{
    cv::Mat mat_cluster_img((int)(pixel_fov_up - pixel_fov_down), (int)(pixel_fov_left - pixel_fov_right), CV_8UC3, cv::Scalar::all(0));

    if(range_image_list.size()>0){
        if(range_image_list.back()->cluster_poses_vec.size() == 0 ) return mat_cluster_img;
    
        std::vector<std::vector<int>> sorted_cluster_poses_vec = range_image_list.back()->cluster_poses_vec;
        std::sort(sorted_cluster_poses_vec.begin(), sorted_cluster_poses_vec.end(), by_size_decent());

        int cluster_num = sorted_cluster_poses_vec.size();
        const int MAX_COLOR_VALUE = 255;
        int hor_idx, ver_idx;

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
                hor_idx = pos / MAX_1D_HALF;
                ver_idx = pos % MAX_1D_HALF;

                mat_cluster_img.at<cv::Vec3b>(pixel_fov_up - ver_idx - 1, pixel_fov_left - hor_idx - 1) = cv::Vec3b(r,g,b);
            }

            
        }
    }

    return mat_cluster_img;
}

cv::Mat RangePmc::GetGroundImageCv()
{
    cv::Mat mat_ground_img((int)(pixel_fov_up - pixel_fov_down), (int)(pixel_fov_left - pixel_fov_right), CV_8U, cv::Scalar::all(0));

    int ground_count = 0;
    int non_ground_count = 0;
    if(range_image_list.size()>0){
        for(int ver_idx = pixel_fov_down; ver_idx < pixel_fov_up; ver_idx++) {
            for(int hor_idx = pixel_fov_right; hor_idx < pixel_fov_left; hor_idx++) {
                
                int iter_pos = hor_idx * MAX_1D_HALF + ver_idx;
                if(range_image_list.back()->range_image[iter_pos].size() > 0){
                    int min_range_idx = range_image_list.back()->min_range_index_all[iter_pos];

                    if(range_image_list.back()->range_image[iter_pos][min_range_idx]->ground == true){
                        mat_ground_img.at<uint8_t>(pixel_fov_up - ver_idx - 1, pixel_fov_left - hor_idx - 1) = (uint8_t)2;
                        ground_count++;
                    }
                    else{
                        mat_ground_img.at<uint8_t>(pixel_fov_up - ver_idx - 1, pixel_fov_left - hor_idx - 1) = (uint8_t)1;
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
    *o_pmc_xyzrgb_pcptr = *pmc_xyzrgb_pcptr_;
}

void RangePmc::GetKeyFramePoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& o_key_frame_xyzrgb_pcptr)
{   
    *o_key_frame_xyzrgb_pcptr = *range_image_xyzrgb_pcptr_;
}

void RangePmc::GetClusterPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& o_cluster_xyzrgb_pcptr)
{   
    *o_cluster_xyzrgb_pcptr = *cluster_xyzrgb_pcptr_;
}


