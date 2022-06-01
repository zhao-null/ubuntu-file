/*
 * @Description: 数据预处理模块，包括时间同步、点云去畸变等
 * @Author: Zhao Chengyue
 * @Date: 2021-09-07 16:09:00
 */
#include "localization_mapping/data_pretreat/data_pretreat_flow.hpp"
//#include "glog/logging.h"
//#include "localization_mapping/global_defination/global_defination.h"
namespace localization_mapping {
DataPretreatFlow::DataPretreatFlow(ros::NodeHandle& nh): cloud_filtered_ptr_(new pcl::PointCloud<pcl::PointXYZ>),
                                                         cv_range_image_ptr_(new sensor_msgs::Image),
                                                         cv_range_image_ground_ptr_(new sensor_msgs::Image),
                                                         point_with_image_ptr_(new localization_mapping::point_range_image)
{
    // subscriber
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/velodyne_points", 100000);
    //imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    //velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel", 1000000);
    //gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);
    //lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, "/imu_link", "velo_link");
    // publisher
    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/pcl_points", "/Laser_Link", 100);
    range_image_pub_ptr_ = std::make_shared<RangeImagePublisher>(nh,"range_image",100);
    point_with_image_pub_ptr_ = std::make_shared<PointWithImagePublisher>(nh,"point_with_image",100);
    //gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/synced_gnss", "/map", "/velo_link", 100);

    //distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();
}

bool DataPretreatFlow::Run() {
    if (!ReadData())
        return false;

    //if (!InitCalibration()) 
    //    return false;

    //if (!InitGNSS())
    //    return false;

    while(HasData()) {
        if (!ValidData())
            continue;
        SplitGround();
        // PassThrough();
        CloudToImage(*cloud_filtered_ptr_);
        // CloudToImage(*current_cloud_data_.cloud_ptr);
        //TransformData();
        PublishData();
    }

    return true;
}

bool DataPretreatFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);

    //static std::deque<IMUData> unsynced_imu_;
    //static std::deque<VelocityData> unsynced_velocity_;
    //static std::deque<GNSSData> unsynced_gnss_;

    //imu_sub_ptr_->ParseData(unsynced_imu_);
    //velocity_sub_ptr_->ParseData(unsynced_velocity_);
    //gnss_sub_ptr_->ParseData(unsynced_gnss_);

    if (cloud_data_buff_.size() == 0)
        return false;

    //double cloud_time = cloud_data_buff_.front().time;
    //bool valid_imu = IMUData::SyncData(unsynced_imu_, imu_data_buff_, cloud_time);
    //bool valid_velocity = VelocityData::SyncData(unsynced_velocity_, velocity_data_buff_, cloud_time);
    //bool valid_gnss = GNSSData::SyncData(unsynced_gnss_, gnss_data_buff_, cloud_time);

    //static bool sensor_inited = false;
    /*
    if (!sensor_inited) {
        if (!valid_imu || !valid_velocity || !valid_gnss) {
            cloud_data_buff_.pop_front();
            return false;
        }
        sensor_inited = true;
    }
    */

    return true;
}
/*
bool DataPretreatFlow::InitCalibration() {
    static bool calibration_received = false;
    if (!calibration_received) {
        if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
            calibration_received = true;
        }
    }

    return calibration_received;
}

bool DataPretreatFlow::InitGNSS() {
    static bool gnss_inited = false;
    if (!gnss_inited) {
        GNSSData gnss_data = gnss_data_buff_.front();
        gnss_data.InitOriginPosition();
        gnss_inited = true;
    }

    return gnss_inited;
}
*/
bool DataPretreatFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    /*
    if (imu_data_buff_.size() == 0)
        return false;
    if (velocity_data_buff_.size() == 0)
        return false;
    if (gnss_data_buff_.size() == 0)
        return false;
    */
    return true;
}

bool DataPretreatFlow::ValidData() {
    while(cloud_data_buff_.size()>1)
    {
        cloud_data_buff_.pop_front();
    }
    current_cloud_data_ = cloud_data_buff_.front();
    //current_imu_data_ = imu_data_buff_.front();
    //current_velocity_data_ = velocity_data_buff_.front();
    //current_gnss_data_ = gnss_data_buff_.front();

    //double diff_imu_time = current_cloud_data_.time - current_imu_data_.time;
    //double diff_velocity_time = current_cloud_data_.time - current_velocity_data_.time;
    //double diff_gnss_time = current_cloud_data_.time - current_gnss_data_.time;
//    if (diff_imu_time < -0.05 || diff_velocity_time < -0.05 || diff_gnss_time < -0.05) {
//        cloud_data_buff_.pop_front();
//        return false;
//    }
//
//    if (diff_imu_time > 0.05) {
//        imu_data_buff_.pop_front();
//        return false;
//    }
//
//    if (diff_velocity_time > 0.05) {
//        velocity_data_buff_.pop_front();
//        return false;
//    }
//
//    if (diff_gnss_time > 0.05) {
//        gnss_data_buff_.pop_front();
//        return false;
//    }

    cloud_data_buff_.pop_front();
//    imu_data_buff_.pop_front();
//    velocity_data_buff_.pop_front();
//    gnss_data_buff_.pop_front();

    return true;
}
/*
bool DataPretreatFlow::TransformData() {
    gnss_pose_ = Eigen::Matrix4f::Identity();

    current_gnss_data_.UpdateXYZ();
    gnss_pose_(0,3) = current_gnss_data_.local_E;
    gnss_pose_(1,3) = current_gnss_data_.local_N;
    gnss_pose_(2,3) = current_gnss_data_.local_U;
    gnss_pose_.block<3,3>(0,0) = current_imu_data_.GetOrientationMatrix();
    gnss_pose_ *= lidar_to_imu_;

    current_velocity_data_.TransformCoordinate(lidar_to_imu_);
    distortion_adjust_ptr_->SetMotionInfo(0.1, current_velocity_data_);
    distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr, current_cloud_data_.cloud_ptr);

    return true;
}
*/
bool DataPretreatFlow::CloudToImage(pcl::PointCloud<pcl::PointXYZ>& cloud_input){
    //set the primer time of transform cloud to image
    clock_t now_time = clock();
    //set parameters
    float angularResolution_x = (float)(0.2f * (M_PI/180.0f));//radius 0.2
    float angularResolution_y = (float)(1.0f * (M_PI/180.0f));//radius 0.2
    float maxAngleWidth = (float)(360.0f * (M_PI/180.0f));//radius 360
    float maxAngleHeight = (float)(32.0f * (M_PI/180.0f));//radius 360
    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f (
            current_cloud_data_.cloud_ptr->sensor_origin_[0],
            current_cloud_data_.cloud_ptr->sensor_origin_[1],
            current_cloud_data_.cloud_ptr->sensor_origin_[2]) *
                    Eigen::Affine3f (current_cloud_data_.cloud_ptr->sensor_orientation_);//collect pose
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;
    float noiseLevel=0.00;
    float minRange = 0.0f;
    int borderSize = 1;
    Eigen::Vector3f  center(0,0,0);
    rangeImage_.createFromPointCloud(*current_cloud_data_.cloud_ptr, angularResolution_x,angularResolution_y, maxAngleWidth,
            maxAngleHeight, sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
    rangeImageGround_.createFromPointCloud(cloud_input, angularResolution_x,angularResolution_y, maxAngleWidth,
            maxAngleHeight, sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
    //save range image
    // std::cout<<"all"<<rangeImage_<<std::endl;
    // std::cout<<"ground"<<rangeImageGround_<<std::endl;
    float *ground_ranges = rangeImageGround_.getRangesArray();
    float *ranges = rangeImage_.getRangesArray();
    // std::vector<std::vector<float>> array_range;
    std::vector<float> array_range(ranges, ranges+rangeImage_.width*rangeImage_.height);
    // std::vector<float> array_range;
    // for(int i = 0; ranges[i] != '\0'; i++)
    // {
    //     array_range[i] = ranges[i];
    // }
    for(int i = rangeImage_.points.size()-rangeImageGround_.width*(rangeImageGround_.height); ranges[i+1] != '\0'; i++)
    {
        if(ground_ranges[i-(rangeImage_.height-rangeImageGround_.height)*rangeImageGround_.width] > 0)
        {
            array_range[i] = -INFINITY;
        }
    }
    point_with_image_ptr_->point_range.data.clear();
    for(int i = 0; i<rangeImage_.width*rangeImage_.height; i++){
        point_with_image_ptr_->point_range.data.push_back(rangeImage_.points[i].x);
        point_with_image_ptr_->point_range.data.push_back(rangeImage_.points[i].y);
        point_with_image_ptr_->point_range.data.push_back(rangeImage_.points[i].z);
        point_with_image_ptr_->point_range.data.push_back(rangeImage_.points[i].range);
    }
    uchar * ground_rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(ground_ranges,rangeImageGround_.width,rangeImageGround_.height);
    cv::Mat cv_range_image_ground(rangeImageGround_.height,rangeImageGround_.width,CV_8UC3,ground_rgb_image);
    cv_range_image_ground_ptr_ = cv_bridge::CvImage(std_msgs::Header(),"rgb8", cv_range_image_ground).toImageMsg();
    // point_with_image_ptr_->image = *cv_range_image_ptr_;
    pcl::io::saveRgbPNGFile("/home/zcy/pictures/img1/" + std::to_string(picture_num_) +".png",
            ground_rgb_image,rangeImageGround_.width,rangeImageGround_.height);//保存至该png文件
    
    uchar * rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(&array_range[0],rangeImage_.width,rangeImage_.height);
    cv::Mat cv_range_image(rangeImage_.height,rangeImage_.width,CV_8UC3,rgb_image);
    cv_range_image_ptr_ = cv_bridge::CvImage(std_msgs::Header(),"rgb8", cv_range_image).toImageMsg();
    // point_with_image_ptr_->image = *cv_range_image_ptr_;
    pcl::io::saveRgbPNGFile("/home/zcy/pictures/img2/" + std::to_string(picture_num_) +".png",
            rgb_image,rangeImage_.width,rangeImage_.height);//保存至该png文件
    
    picture_num_ ++;
    //print cost picture
    clock_t new_time = clock();
    std::cout<<"cost time:"<<double(new_time-now_time)/CLOCKS_PER_SEC<<std::endl;
    return true;
}

bool DataPretreatFlow::SplitGround()
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr  inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    // 距离阈值 单位m
    seg.setDistanceThreshold (0.1);
    seg.setInputCloud (current_cloud_data_.cloud_ptr);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return false;
    }
    // 提取地面
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (current_cloud_data_.cloud_ptr);
    extract.setIndices (inliers);
    // extract.filter(*cloud_filtered_ptr_);
    // 提取除地面外的物体
    extract.setNegative (false);
    extract.filter(*cloud_filtered_ptr_);
    return true;
}

bool DataPretreatFlow::PassThrough()
{
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (current_cloud_data_.cloud_ptr);//设置输入点云
    pass.setFilterFieldName ("x");//设置过滤时所需要点云类型的Z字段
    pass.setFilterLimits (-6,  6);//设置在过滤字段上的范围
    pass.setFilterLimitsNegative (false);//设置保留范围内的还是过滤掉范围内的:算法内部默认false，即保留范围内的，滤掉范围外的；若设为true，则保留范围外的，滤掉范围内的；
    pass.filter (*cloud_filtered_ptr_);//执行过滤，过滤结果在cloud_filtered

    pass.setInputCloud (current_cloud_data_.cloud_ptr);//设置输入点云
    pass.setFilterFieldName ("y");//设置过滤时所需要点云类型的Z字段
    pass.setFilterLimits (-3.5,  3.5);//设置在过滤字段上的范围
    pass.setFilterLimitsNegative (false);//设置保留范围内的还是过滤掉范围内的:算法内部默认false，即保留范围内的，滤掉范围外的；若设为true，则保留范围外的，滤掉范围内的；
    pass.filter (*cloud_filtered_ptr_);//执行过滤，过滤结果在cloud_filtered
}

bool DataPretreatFlow::PublishData() {
    cloud_pub_ptr_->Publish(cloud_filtered_ptr_, current_cloud_data_.time);
    range_image_pub_ptr_->Publish(cv_range_image_ptr_);
    point_with_image_pub_ptr_->Publish(point_with_image_ptr_);
    //gnss_pub_ptr_->Publish(gnss_pose_, current_gnss_data_.time);
    return true;
}
}
