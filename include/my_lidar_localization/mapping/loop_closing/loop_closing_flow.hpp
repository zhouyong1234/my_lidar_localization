/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-02-29 03:32:14
 */
#ifndef MY_LIDAR_LOCALIZATION_MAPPING_LOOP_CLOSING_LOOP_CLOSING_FLOW_HPP_
#define MY_LIDAR_LOCALIZATION_MAPPING_LOOP_CLOSING_LOOP_CLOSING_FLOW_HPP_

#include <deque>
#include <ros/ros.h>
// subscriber
#include "my_lidar_localization/subscriber/key_frame_subscriber.hpp"
// publisher
#include "my_lidar_localization/publisher/loop_pose_publisher.hpp"
// loop closing
#include "my_lidar_localization/mapping/loop_closing/loop_closing.hpp"

namespace my_lidar_localization {
class LoopClosingFlow {
  public:
    LoopClosingFlow(ros::NodeHandle& nh);

    bool Run();

  private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool PublishData();

  private:
    // subscriber
    std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_;
    std::shared_ptr<KeyFrameSubscriber> key_gnss_sub_ptr_;
    // publisher
    std::shared_ptr<LoopPosePublisher> loop_pose_pub_ptr_;
    // loop closing
    std::shared_ptr<LoopClosing> loop_closing_ptr_;

    std::deque<KeyFrame> key_frame_buff_;
    std::deque<KeyFrame> key_gnss_buff_;

    KeyFrame current_key_frame_;
    KeyFrame current_key_gnss_;
};
}

#endif