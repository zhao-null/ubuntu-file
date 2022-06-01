/*
 * @Description: 在ros中发布深度图以及对应点的坐标
 * @Author: Zhao Chengyue
 * @Date: 2021-09-20 16:49:00
 */

#include "localization_mapping/publisher/point_with_image_publisher.hpp"
//#include "glog/logging.h"


namespace localization_mapping {
PointWithImagePublisher::PointWithImagePublisher(ros::NodeHandle& nh,
                               std::string topic_name,
                               size_t buff_size)
    :nh_(nh){
    publisher_ = nh_.advertise<localization_mapping::point_range_image>(topic_name, buff_size);
}

void PointWithImagePublisher::Publish(localization_mapping::point_range_image::Ptr point_with_image, double time) {
    ros::Time ros_time((float)time);
    PublishData(point_with_image, ros_time);
}

void PointWithImagePublisher::Publish(localization_mapping::point_range_image::Ptr point_with_image) {
    ros::Time time = ros::Time::now();
    PublishData(point_with_image, time);
}

void PointWithImagePublisher::PublishData(localization_mapping::point_range_image::Ptr point_with_image, ros::Time time) {
    publisher_.publish(point_with_image);
}

bool PointWithImagePublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
} // namespace lidar_localization
