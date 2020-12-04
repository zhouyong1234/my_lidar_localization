/*
 * @Description: 订阅odometry数据
 * @Author: Ren Qian
 * @Date: 2019-08-19 19:22:17
 */
#ifndef MY_LIDAR_LOCALIZATION_SUBSCRIBER_ODOMETRY_SUBSCRIBER_HPP_
#define MY_LIDAR_LOCALIZATION_SUBSCRIBER_ODOMETRY_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "my_lidar_localization/sensor_data/pose_data.hpp"

namespace my_lidar_localization {
class OdometrySubscriber {
  public:
    OdometrySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    OdometrySubscriber() = default;
    void ParseData(std::deque<PoseData>& deque_pose_data);

  private:
    void msg_callback(const nav_msgs::OdometryConstPtr& odom_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<PoseData> new_pose_data_; 
};
}
#endif