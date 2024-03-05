# LiDAR_preprocessing

## Functions
각 함수 설명 작성할것
간단한 설명, input, output, return, configuration(필요시)

추후 함수이름들 함수명으로 대체

---
### Deskewing
Brief
- 라이다 frame의 motion정보(속도, 각속도)를 기반으로 point cloud의 각 포인트를를 scan 시작 지점으로 transformation하는 함수
- CV 모델을 기반으로 각 포인트들의 pose를 추정하여 transformation 수행
- motion 정보의 time과 pointcloud의 time이 같은 시계열? 로 구성되어 있야 함
- pointcloud에 찍히는 시간은 scan 종료 시간 기준이지만, 본 deskewing은 scan 시작점의 frame으로 일치 시킴

Input
- pointcloud
- lidar_time_sec
- motion_dt (vel_x_ms,vel_y_ms, vel_z_ms, ang_vel_x_rads, ang_vel_y_rads, ang_vel_z_rads, time) 

Output
- deskewed pointcloud (pcl object)

Return
- ?

Configuration

---
### Extrinsic_calibration


---
### Multiple_LiDARs_merging

---

### Downsampling

---

### 극좌표 변환 / 소팅

---

### Range_image_generation

---

### Ground_extraction

---

### Clustering/Segmentation

---

### Small_object_filtering

---

### Object_fitting
