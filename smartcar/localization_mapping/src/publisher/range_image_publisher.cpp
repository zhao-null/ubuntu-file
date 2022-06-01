/*
 * @Description: 通过ros发布深度图像
 * @Author: Zhao Chengyue
 * @Date: 2021-09-10 10:51:00
 */

#include "localization_mapping/publisher/range_image_publisher.hpp"
//#include "glog/logging.h"


namespace localization_mapping {
RangeImagePublisher::RangeImagePublisher(ros::NodeHandle& nh,
                               std::string topic_name,
                               size_t buff_size)
    :nh_(nh){
    publisher_ = nh_.advertise<sensor_msgs::Image>(topic_name, buff_size);
}

void RangeImagePublisher::Publish(sensor_msgs::ImagePtr& range_image_ptr_input, double time) {
    ros::Time ros_time((float)time);
    PublishData(range_image_ptr_input, ros_time);
}

void RangeImagePublisher::Publish(sensor_msgs::ImagePtr& range_image_ptr_input) {
    ros::Time time = ros::Time::now();
    PublishData(range_image_ptr_input, time);
}

void RangeImagePublisher::PublishData(sensor_msgs::ImagePtr& range_image_ptr_input, ros::Time time) {
    publisher_.publish(*range_image_ptr_input);
}

bool RangeImagePublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
} // namespace lidar_localization
