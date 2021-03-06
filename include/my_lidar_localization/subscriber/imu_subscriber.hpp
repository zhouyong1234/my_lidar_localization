/*
 *@Description:
 *@Author:
 *@Date:
*/

#ifndef MY_LIDAR_LOCALIZATION_SUBSCRIBER_IMU_SUBSCRIBER_HPP_
#define MY_LIDAR_LOCALIZATION_SUBSCRIBER_IMU_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "my_lidar_localization/sensor_data/imu_data.hpp"

namespace my_lidar_localization
{
class IMUSubscriber{
public:
    IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);

    IMUSubscriber() = default;

    void ParseData(std::deque<IMUData>& deque_imu_data);
private:
    void msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<IMUData> new_imu_data_;
};
}

#endif