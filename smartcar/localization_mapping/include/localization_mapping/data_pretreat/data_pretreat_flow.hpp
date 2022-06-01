/*
 * @Description: 数据预处理模块，包括时间同步、点云去畸变等
 * @Author: Zhao Chengyue
 * @Date: 2021-09-07 16:07:00
 */
#ifndef LOCALIZATION_MAPPING_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_
#define LOCALIZATION_MAPPING_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/common/float_image_utils.h>//保存深度图像
#include <pcl/io/png_io.h>//保存深度图像
#include <pcl/filters/plane_clipper3D.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
//#include<pcl/visualization/range_image_visualizer.h>
//#include<pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
// subscriber
#include "localization_mapping/subscriber/cloud_subscriber.hpp"
//#include "lidar_localization/subscriber/imu_subscriber.hpp"
//#include "lidar_localization/subscriber/velocity_subscriber.hpp"
//#include "lidar_localization/subscriber/gnss_subscriber.hpp"
//#include "lidar_localization/tf_listener/tf_listener.hpp"
// publisher
#include "localization_mapping/publisher/cloud_publisher.hpp"
#include "localization_mapping/publisher//range_image_publisher.hpp"
#include "localization_mapping/publisher//point_with_image_publisher.hpp"
//#include "lidar_localization/publisher/odometry_publisher.hpp"
// models
//#include "lidar_localization/models/scan_adjust/distortion_adjust.hpp"

namespace localization_mapping {
/*
    boost::shared_ptr<pcl::RangeImage> rangeImagePtr(new pcl::RangeImage);
    pcl::RangeImage& rangeImage_ = *rangeImagePtr;
    pcl::visualization::RangeImageVisualizer range_image_widget_;
*/
class DataPretreatFlow {
  public:
    DataPretreatFlow(ros::NodeHandle& nh);

//    pcl::RangeImage::Ptr rangeImagePtr_;
//    pcl::RangeImage& rangeImage_ = *rangeImagePtr_;
    int picture_num_ = 1;
//    pcl::visualization::CloudViewer viewer_;
//    pcl::visualization::RangeImageVisualizer range_image_widget_;
    bool Run();

  private:
    bool ReadData();
    bool InitCalibration();
    bool InitGNSS();
    bool HasData();
    bool ValidData();
    bool TransformData();
    bool SplitGround();
    bool PassThrough();
    bool CloudToImage(pcl::PointCloud<pcl::PointXYZ>& cloud_input);
    bool PublishData();

  private:
    // subscriber
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    //std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    //std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;
    //std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
    //std::shared_ptr<TFListener> lidar_to_imu_ptr_;
    // publisher
    std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
    std::shared_ptr<RangeImagePublisher> range_image_pub_ptr_;
    std::shared_ptr<PointWithImagePublisher> point_with_image_pub_ptr_;
    //std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;
    // models
    //std::shared_ptr<DistortionAdjust> distortion_adjust_ptr_;

    //Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();

    std::deque<CloudData> cloud_data_buff_;
    //std::deque<IMUData> imu_data_buff_;
    //std::deque<VelocityData> velocity_data_buff_;
    //std::deque<GNSSData> gnss_data_buff_;

    CloudData current_cloud_data_;


    pcl::RangeImage rangeImage_;
    pcl::RangeImage rangeImageGround_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ptr_;
    sensor_msgs::Image::Ptr cv_range_image_ptr_;
    sensor_msgs::Image::Ptr cv_range_image_ground_ptr_;
    localization_mapping::point_range_image::Ptr point_with_image_ptr_;
    //IMUData current_imu_data_;
    //VelocityData current_velocity_data_;
    //GNSSData current_gnss_data_;

    //Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();
};
}

#endif
