/*
 * @Description:
 * @Author: Zhao Chengyue
 * @Date: 2021-09-7 16:00:00
 */
#ifndef LOCALIZATION_MAPPING_SENSOR_DATA_CLOUD_DATA_HPP_
#define LOCALIZATION_MAPPING_SENSOR_DATA_CLOUD_DATA_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace localization_mapping {
class CloudData {
  public:
    using POINT = pcl::PointXYZ;
    using CLOUD = pcl::PointCloud<POINT>;
    using CLOUD_PTR = CLOUD::Ptr;

  public:
    CloudData()
      :cloud_ptr(new CLOUD()) {
    }

  public:
    double time = 0.0;
    CLOUD_PTR cloud_ptr;
};
}

#endif
