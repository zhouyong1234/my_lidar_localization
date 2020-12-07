/*
 * @Description: 不滤波
 * @Author: Ren Qian
 * @Date: 2020-02-09 19:37:49
 */
#ifndef MY_LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_NO_FILTER_HPP_
#define MY_LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_NO_FILTER_HPP_

#include "my_lidar_localization/models/cloud_filter/cloud_filter_interface.hpp"

namespace my_lidar_localization {
class NoFilter: public CloudFilterInterface {
  public:
    NoFilter();

    bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;
};
}
#endif