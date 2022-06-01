/*
 * @Description: 在ros中发布深度图以及对应点的坐标
 * @Author: Zhao Chengyue
 * @Date: 2021-09-20 16:49:00
 */

#ifndef LOCALIZATION_MAPPING_PUBLISHER_POINT_WITH_IMAGE_PUBLISHER_HPP_
#define LOCALIZATION_MAPPING_PUBLISHER_POINT_WITH_IMAGE_PUBLISHER_HPP_

#include <ros/ros.h>
#include "localization_mapping/point_range_image.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

namespace localization_mapping {
class PointWithImagePublisher {
  public:
    PointWithImagePublisher(ros::NodeHandle& nh,
                   std::string topic_name,
                   size_t buff_size);
    PointWithImagePublisher() = default;

    void Publish(localization_mapping::point_range_image::Ptr point_with_image, double time);
    void Publish(localization_mapping::point_range_image::Ptr point_with_image);

    bool HasSubscribers();
  
  private:
    void PublishData(localization_mapping::point_range_image::Ptr point_with_image, ros::Time time);

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
};
} 
#endif
