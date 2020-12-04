/*
 * @Description: 关键帧，在各个模块之间传递数据
 * @Author: Ren Qian
 * @Date: 2020-02-28 19:13:26
 */
#ifndef MY_LIDAR_LOCALIZATION_SENSOR_DATA_KEY_FRAME_HPP_
#define MY_LIDAR_LOCALIZATION_SENSOR_DATA_KEY_FRAME_HPP_

#include <Eigen/Dense>

namespace my_lidar_localization {
class KeyFrame {
  public:
    double time = 0.0;
    unsigned int index = 0;
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

  public:
    Eigen::Quaternionf GetQuaternion();
};
}

#endif