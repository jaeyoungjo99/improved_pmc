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
    temp_image_pointer.reset(new RangeImage());
    

    pixel_fov_up    = floor((params_.f_fov_up   / 180.0*M_PI + 0.5 * M_PI)  /params_.f_vertical_resolution);
    pixel_fov_down  = floor((params_.f_fov_down / 180.0*M_PI + 0.5 * M_PI)  /params_.f_vertical_resolution);
    pixel_fov_left  = floor((params_.f_fov_left / 180.0*M_PI +  M_PI)       /params_.f_horizontal_resolution);
    pixel_fov_right = floor((params_.f_fov_right/ 180.0*M_PI +  M_PI)       /params_.f_horizontal_resolution);
    // max_pointers_num = round((params_.i_max_range_image_num * range_map_dur + buffer_delay)/frame_dur) + 1;
    max_pointers_num = params_.i_max_range_image_num;
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

    
    cout<<"[Range PMC] Init Done"<<endl;

}


void RangePmc::Filter(PointCloudXYZI::Ptr feats_undistort, const M3D & rot_end, const V3D & pos_end, const double & scan_end_time)
{   
    double t00 = omp_get_wtime();
    cout<<"[Range PMC] Filter Started"<<endl;

    is_key_frame = KeyFrameCheck(scan_end_time, rot_end, pos_end);


    int size = feats_undistort->points.size(); // point num

    std::vector<int> index(size); // Point Index 할당 
    for (int i = 0; i < size; i++) {
        index[i] = i;
    }

    pmc_xyzrgb_pcptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    pmc_xyzrgb_pcptr_->reserve(size);

    vector<point_soph*> points; // point_soph의 주소들을 저장하기 위한 벡터 
    points.reserve(size);
    points.resize(size);

    point_soph* p;  // point_soph 포인트 배열 
    if(is_key_frame == true){
        p = point_soph_pointers[cur_point_soph_pointers]; // 현재 range image내  point_soph들 메모리 불러옴. 초기화 할 것임 
    }
    else{
        p = temp_point_soph_pointer; // 임시 포인터 
    }

    // 0304
    for (auto i : index)
    {   // 한 range image 내부를 iteration
        V3D p_body(feats_undistort->points[i].x, feats_undistort->points[i].y, feats_undistort->points[i].z); // 라이다 기준 point 위치
        V3D p_glob(rot_end * (p_body) + pos_end); // global point 위치 

        p[i].glob = p_glob;        
        p[i].dyn = STATIC;
        p[i].rot = rot_end.transpose();
        p[i].transl = pos_end;
        p[i].time = scan_end_time;
        p[i].local = p_body;
        p[i].intensity = feats_undistort->points[i].intensity;
    }

    temp_image_pointer->Reset(rot_end, pos_end, scan_end_time, map_index);

    RangeImage::Ptr new_image_pointer(new RangeImage(rot_end, pos_end, scan_end_time, map_index));

    GenerateRangeImage(p, scan_end_time, rot_end, pos_end, size, new_image_pointer);

    double tcase1_start = omp_get_wtime();
    for (auto i : index)
    {   // 한 range image 내부를 iteration


        int iter_pos = p[i].position;
        int nearest_idx = new_image_pointer->max_range_index_all[iter_pos];
        
        V3D p_body(feats_undistort->points[i].x, feats_undistort->points[i].y, feats_undistort->points[i].z); // 라이다 기준 point 위치
        V3D p_glob(rot_end * (p_body) + pos_end); // global point 위치 

        if (p[i].local.z() < -1.5){
            p[i].dyn = STATIC;
            points[i] = &p[i]; // p[i] 객체의 주소 를 points[i]에 저장 
            // new_image_pointer->range_image[iter_pos][nearest_idx]->dyn = p[i].dyn;
            continue;
        }

        if (Case1(p[i]) == true){ // p의 vec, hor_ind, ver_ind, position 가 range_image_list[i] 기준으로 바뀜
            p[i].dyn = CASE1;
        }   


        points[i] = &p[i]; // p[i] 객체의 주소 를 points[i]에 저장 
    }

    double tcase1_end = omp_get_wtime() - tcase1_start;
    std::cout<<"[Range PMC] Case: "<< tcase1_end*1000.0 << " ms"<<std::endl;

    // 위에서 p의 값을 변경하였고 p의 주소가 new_image_pointer->range_image에 있음
    // 
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

        cur_point_soph_pointers = (cur_point_soph_pointers + 1)%max_pointers_num; // 다음 번 range image 의 index를 지정함 
    }


    std::cout<<"[Range PMC] Image num: "<< range_image_list.size() <<std::endl;

    // Output
    OutputPmcPC(points);

    time_total = omp_get_wtime() - t00;
    std::cout<<"[Range PMC] Total Time: "<< time_total*1000.0 << " ms"<<std::endl;
}

bool RangePmc::Case1(point_soph & p)
{
    int range_image_num = range_image_list.size();
    int occluded_image = range_image_num;
    int valid_image = 0;
    bool is_valid;
    bool p_valid = false;
    for (int i = range_image_num- 1; i >= 0; i--) // point 한개에 대해 range image 탐색. 최근 range image 부터 
    {   
        SphericalProjection(p, range_image_list[i]->map_index, range_image_list[i]->project_R, range_image_list[i]->project_T, p);
        if (fabs(p.hor_ind) > MAX_1D || fabs(p.ver_ind) > MAX_1D_HALF || p.vec(2) < 0.0f \
            || p.position < 0 || p.position >= MAX_2D_N)
        {
            p.dyn = INVALID;
            continue; // range image의 경계를 넘은 point는 iter range image와 맞지 않음 
        }

        if (Case1Enter(p, *range_image_list[i], is_valid) == true)
        {   


            p_valid = true;
            valid_image++;
        }
        else{ // p is not occluding range image. static or moving away
            occluded_image -= 1;
            if(is_valid == true){
                p_valid = true;
                valid_image++;
            }
        }

    }

    if(p_valid == false || valid_image <  params_.i_min_occluded_num / 2){
        p.dyn = INVALID;
        return false;
    }

    if(occluded_image >= params_.i_min_occluded_num)
    {
        return true;
    }
    return false;  

}

// return true if p is occluding previous range images
bool RangePmc::Case1Enter(const point_soph & p, const RangeImage &image_info, bool & is_valid)
{   
    // p는 image_info 기준으로 projection 되어있음
    
    float max_range = 0, min_range = 0;
    is_valid = true;

    max_range = image_info.max_range_all[p.position];
    min_range = image_info.min_range_all[p.position];

    if(p.ver_ind <= pixel_fov_up && p.ver_ind > pixel_fov_down && \
        p.hor_ind <= pixel_fov_left && p.hor_ind >= pixel_fov_right
        )
    {
        is_valid = CheckNeighbor(p, image_info, max_range, min_range);
    }
    else{
        // fov에 없거나 neighbor 포인트를 못찾은경우 is_valid = false
        is_valid = false;
        return false;
    }
 
    
    if(min_range < 10E-5){
        is_valid = false;
        return false;
    }
    if (p.vec(2) < min_range - params_.f_range_threshold){ // if point is closer than min range
        return true;
    }

    return false;
}


void RangePmc::GenerateRangeImage(point_soph* &points, double cur_time, M3D rot, V3D transl, int len, RangeImage::Ptr range_image_pointer)
{   
    double t00 = omp_get_wtime();

    for (int k = 0; k < len; k++)
    { 
        points[k].GetVec(points[k].local, params_.f_horizontal_resolution, params_.f_vertical_resolution);
        
        // FOV를 벗어난 point는 range image에 할당하지 않음
        if(points[k].ver_ind > pixel_fov_up || points[k].ver_ind < pixel_fov_down || \
           points[k].hor_ind > pixel_fov_left || points[k].hor_ind < pixel_fov_right ){
            continue;
        }

        if(range_image_pointer->range_image[points[k].position].size() < max_pixel_points) // 해당 픽셀에 여유가 있다면 
        {   
            range_image_pointer->range_image[points[k].position].push_back(&points[k]); // 주소를 추가함 
            i_push_back++;

            // 이 포인트의 range가 픽셀 내 모든 포인트보다 크다면 
            if (points[k].vec(2) > range_image_pointer->max_range_all[points[k].position])  
            {
                range_image_pointer->max_range_all[points[k].position] = points[k].vec(2); // 최대 range 교체
                range_image_pointer->max_range_index_all[points[k].position] = range_image_pointer->range_image[points[k].position].size()-1;
            }

            // 이 포인트의 range가 픽셀 내 모든 포인트보다 작다면, 혹은 min_range_all가 0이라면
            if (points[k].vec(2) < range_image_pointer->min_range_all[points[k].position] ||\
                range_image_pointer->min_range_all[points[k].position] < 10E-5)  
            {
                range_image_pointer->min_range_all[points[k].position] = points[k].vec(2); // 최소 range 교체
                range_image_pointer->min_range_index_all[points[k].position] = range_image_pointer->range_image[points[k].position].size()-1;
            }

        }
    }

    double time_generate_range_image = omp_get_wtime() - t00;
    std::cout<<"[GenerateRangeImage] Total Time: "<< time_generate_range_image*1000.0 << " ms"<<std::endl;
}


// Segment Ground from Range Image
void RangePmc::GroundSegmentation(RangeImage::Ptr range_image_ptr)
{   
    double t00 = omp_get_wtime();

    V3D u_point, l_point;
    double v_angle, i_angle, p_angle; // vertical, incident angle
    double hor_dist, ver_dist;
    int u_pos, l_pos, u_point_min_pos, l_point_min_pos;

    for(int hor_idx = pixel_fov_right; hor_idx <= pixel_fov_left; hor_idx++)
    {
        for(int ver_idx = pixel_fov_down; ver_idx < pixel_fov_up; ver_idx++)  // vertical은 하나 적게 올라감
        {   

            u_pos = hor_idx * MAX_1D_HALF + ver_idx + 1;
            l_pos = hor_idx * MAX_1D_HALF + ver_idx;

            u_point_min_pos = range_image_ptr->min_range_index_all[u_pos];
            l_point_min_pos = range_image_ptr->min_range_index_all[l_pos];

            if(u_point_min_pos == -1 || l_point_min_pos == -1 ||
              range_image_ptr->range_image[u_pos].size() == 0 || 
              range_image_ptr->range_image[l_pos].size() == 0) continue; // Check upper lower point valid

            std::cout<<"3"<<std::endl;

            std::cout<<"hor_idx: "<<hor_idx<<" ver_idx: "<<ver_idx<<std::endl;
            std::cout<<"ul_pos: "<<u_pos<<" "<<l_pos<<std::endl;
            std::cout<<"ul_min_pos: "<<u_point_min_pos<<" "<<l_point_min_pos<<std::endl;
            std::cout<<"size: "<<range_image_ptr->range_image[u_pos].size() << " " << range_image_ptr->range_image[l_pos].size()<<std::endl;

            // if(range_image_ptr->range_image[u_pos].size() > 10000 || range_image_ptr->range_image[l_pos].size() > 10000) continue;

            u_point = range_image_ptr->range_image[u_pos][u_point_min_pos]->local;
            l_point = range_image_ptr->range_image[l_pos][l_point_min_pos]->local;


            std::cout<<"4"<<std::endl;

            hor_dist = sqrt(pow(u_point.x() - l_point.x(), 2) + pow(u_point.y() - l_point.y(), 2));
            ver_dist = u_point.z() - l_point.z();

            v_angle = atan2(ver_dist,hor_dist);
            p_angle = atan2(-l_point.z(), sqrt(l_point.x()*l_point.x() + l_point.y() + l_point.y()));
            i_angle = M_PI/2.0 - p_angle - v_angle;

            range_image_ptr->range_image[l_pos][l_point_min_pos]->incident = i_angle;

            if(fabs(v_angle) < params_.f_ground_angle * M_PI/180.0 ){
                range_image_ptr->range_image[l_pos][l_point_min_pos]->ground = true;
                // std::cout<<"Ground: "<<v_angle*180/M_PI<<" p: " <<p_angle* 180/M_PI<<" i: "<< i_angle * 180/M_PI <<std::endl;
            }

        }   
    }

    double ground_total = omp_get_wtime() - t00;
    std::cout<<"[GroundSegmentation] Total Time: "<< ground_total*1000.0 << " ms"<<std::endl;
}

// Segement Object
void RangePmc::ObjectSegmentation(RangeImage::Ptr range_image_ptr)
{

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

void RangePmc::OutputPmcPC(std::vector<point_soph*> &points)
{   
    double tout_start = omp_get_wtime();

    int size = points.size(); // point num

    for(int i = 0; i < size; i++){
        pcl::PointXYZRGB po;
        po.x = points[i]->local[0];
        po.y = points[i]->local[1];
        po.z = points[i]->local[2];

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
            default: // Blue
                po.r = 0;
                po.g = 0;
                po.b = 255;
        }

        pmc_xyzrgb_pcptr_->push_back(po);
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

    if(range_image_list.size()>0){
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
                    mat_dynamic_img.at<cv::Vec3b>(pixel_fov_up - ver_idx - 1, pixel_fov_left - hor_idx - 1) = cv::Vec3b(0,0,0);
                }
            }
        }
    }


    // 채워진 cv::Mat 객체를 반환합니다.
    return mat_dynamic_img;
}

cv::Mat RangePmc::GetIncidentImageCv()
{
    cv::Mat mat_incident_img((int)(pixel_fov_up - pixel_fov_down), (int)(pixel_fov_left - pixel_fov_right), CV_32F, cv::Scalar::all(FLT_MAX));

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


void RangePmc::GetFilteredPoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& o_pmc_xyzrgb_pcptr)
{   
    *o_pmc_xyzrgb_pcptr = *pmc_xyzrgb_pcptr_;
}

