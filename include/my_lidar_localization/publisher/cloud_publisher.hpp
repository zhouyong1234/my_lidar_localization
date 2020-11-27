/*
 * @Description: 在ros中发布点云
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 */

#ifndef MY_LIDAR_LOCALIZATION_PUBLISHER_CLOUD_PUBLISHER_HPP_
#define MY_LIDAR_LOCALIZATION_PUBLISHER_CLOUD_PUBLISHER_HPP_

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "my_lidar_localization/sensor_data/cloud_data.hpp"

namespace my_lidar_localization {
class CloudPublisher {
  public:
    CloudPublisher(ros::NodeHandle& nh,
                   std::string topic_name,
                   size_t buff_size,
                   std::string frame_id);
    CloudPublisher() = default;
    void Publish(CloudData::CLOUD_PTR cloud_ptr_input);
  
  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;
};
} 
#endif