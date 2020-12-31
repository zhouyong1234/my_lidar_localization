/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2019-03-31 12:58:10
 */
#ifndef MY_LIDAR_LOCALIZATION_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_
#define MY_LIDAR_LOCALIZATION_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include "sensor_msgs/NavSatFix.h"

#include "my_lidar_localization/sensor_data/gnss_data.hpp"

namespace my_lidar_localization {
class GNSSSubscriber {
  public:
    GNSSSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    GNSSSubscriber() = default;
    void ParseData(std::deque<GNSSData>& deque_gnss_data);

  private:
    void msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<GNSSData> new_gnss_data_;

    std::mutex buff_mutex_;
};
}
#endif