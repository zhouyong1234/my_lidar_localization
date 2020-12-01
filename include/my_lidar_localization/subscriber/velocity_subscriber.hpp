/*
 * @Description: 订阅velocity数据
 * @Author: Ren Qian
 * @Date: 2019-08-19 19:22:17
 */
#ifndef MY_LIDAR_LOCALIZATION_SUBSCRIBER_VELOCITY_SUBSCRIBER_HPP_
#define MY_LIDAR_LOCALIZATION_SUBSCRIBER_VELOCITY_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include "geometry_msgs/TwistStamped.h"
#include "my_lidar_localization/sensor_data/velocity_data.hpp"

namespace my_lidar_localization {
class VelocitySubscriber {
  public:
    VelocitySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    VelocitySubscriber() = default;
    void ParseData(std::deque<VelocityData>& deque_velocity_data);

  private:
    void msg_callback(const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<VelocityData> new_velocity_data_; 
};
}
#endif