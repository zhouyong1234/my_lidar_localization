/*
 * @Description: 在ros中发布IMU数据
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 */
#ifndef MY_LIDAR_LOCALIZATION_PUBLISHER_IMU_PUBLISHER_HPP_
#define MY_LIDAR_LOCALIZATION_PUBLISHER_IMU_PUBLISHER_HPP_

#include "sensor_msgs/Imu.h"
#include "my_lidar_localization/sensor_data/imu_data.hpp"

namespace my_lidar_localization {
class IMUPublisher {
  public:
    IMUPublisher(ros::NodeHandle& nh,
                 std::string topic_name,
                 size_t buff_size,
                 std::string frame_id);
    IMUPublisher() = default;

    void Publish(IMUData imu_data);

    bool HasSubscribers();

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;
};
} 
#endif