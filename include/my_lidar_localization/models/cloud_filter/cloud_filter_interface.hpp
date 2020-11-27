/*
 * @Description: 点云滤波模块的接口
 * @Author: Ren Qian
 * @Date: 2020-02-09 19:29:50
 */
#ifndef MY_LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_
#define MY_LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include "my_lidar_localization/sensor_data/cloud_data.hpp"

namespace my_lidar_localization {
class CloudFilterInterface {
  public:
    virtual ~CloudFilterInterface() = default;

    virtual bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) = 0;
};
}

#endif