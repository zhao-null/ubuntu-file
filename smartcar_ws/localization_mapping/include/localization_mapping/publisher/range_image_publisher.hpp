/*
 * @Description: 在ros中发布深度图像
 * @Author: Zhao Chengyue
 * @Date: 2021-09-10 10:53:00
 */

#ifndef LOCALIZATION_MAPPING_PUBLISHER_RANGE_IMAGE_PUBLISHER_HPP_
#define LOCALIZATION_MAPPING_PUBLISHER_RANGE_IMAGE_PUBLISHER_HPP_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

namespace localization_mapping {
class RangeImagePublisher {
  public:
    RangeImagePublisher(ros::NodeHandle& nh,
                   std::string topic_name,
                   size_t buff_size);
    RangeImagePublisher() = default;

    void Publish(sensor_msgs::ImagePtr& range_image_ptr_input, double time);
    void Publish(sensor_msgs::ImagePtr& range_image_ptr_input);

    bool HasSubscribers();
  
  private:
    void PublishData(sensor_msgs::ImagePtr& range_image_ptr_input, ros::Time time);

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
};
} 
#endif
