/*
 * @Description: front end 任务管理， 放在类里使代码更清晰
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 */


#include "my_lidar_localization/models/scan_adjust/distortion_adjust.hpp"
#include "glog/logging.h"

namespace my_lidar_localization
{
void DistortionAdjust::SetMotionInfo(float scan_period, VelocityData velocity_data)
{
    scan_period_ = scan_period;
    velocity_ << velocity_data.linear_velocity.x, velocity_data.linear_velocity.y, velocity_data.linear_velocity.z;
    angular_rate_ << velocity_data.angular_velocity.x, velocity_data.angular_velocity.y, velocity_data.angular_velocity.z;
}

bool DistortionAdjust::AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr)
{
    CloudData::CLOUD_PTR origin_cloud_ptr(new CloudData::CLOUD(*input_cloud_ptr));
    output_cloud_ptr->points.clear();

    float orientation_space = 2.0 * M_PI;
    float delete_space = 5.0 * M_PI / 180.0;
    // 初始点云先旋转
    float start_orientation = atan2(origin_cloud_ptr->points[0].y, origin_cloud_ptr->points[0].x); // 起始点方位角 欧拉角
    Eigen::AngleAxisf t_V(start_orientation, Eigen::Vector3f::UnitZ()); // 旋转向量 以z轴为旋转轴
    Eigen::Matrix3f rotate_matrix = t_V.matrix(); //旋转矩阵
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity(); //变换矩阵
    transform_matrix.block<3,3>(0,0) = rotate_matrix.inverse();

    pcl::transformPointCloud(*origin_cloud_ptr, *origin_cloud_ptr, transform_matrix); // 原始点云旋转
    velocity_ = rotate_matrix * velocity_;
    angular_rate_ = rotate_matrix * angular_rate_;

    // 后续点云去除畸变
    for( size_t point_index = 1; point_index < origin_cloud_ptr->points.size(); ++point_index){
        float orientation = atan2(origin_cloud_ptr->points[point_index].y, origin_cloud_ptr->points[point_index].x);

        if(orientation < 0.0)
            orientation += 2.0 * M_PI;
        if(orientation < delete_space || 2.0 * M_PI - orientation < delete_space)
            continue;

        float real_time = fabs(orientation) / orientation_space * scan_period_ - scan_period_ / 2.0;

        Eigen::Vector3f origin_point(origin_cloud_ptr->points[point_index].x,
                                     origin_cloud_ptr->points[point_index].y,
                                     origin_cloud_ptr->points[point_index].z);
        Eigen::Matrix3f current_matrix = UpdateMatrix(real_time); // 根据运行时间获得当前旋转矩阵
        Eigen::Vector3f rotated_point = current_matrix * origin_point; // 旋转矩阵 * 三维点
        Eigen::Vector3f adjusted_point = rotated_point + velocity_ * real_time; // 旋转 + 平移

        CloudData::POINT point;
        point.x = adjusted_point(0);
        point.y = adjusted_point(1);
        point.z = adjusted_point(2);
        output_cloud_ptr->points.push_back(point);
    }
    // 第一次旋转世界坐标系 -> 雷达坐标系
    // 第二次旋转雷达坐标系 -> 世界坐标系
    pcl::transformPointCloud(*output_cloud_ptr, *output_cloud_ptr, transform_matrix.inverse());

    return true;
}

// 获取当前旋转矩阵
Eigen::Matrix3f DistortionAdjust::UpdateMatrix(float real_time)
{
    Eigen::Vector3f angle = angular_rate_ * real_time; // 角度 = 角速度 * 运行时间
    // 旋转向量z-y-x
    Eigen::AngleAxisf t_Vz(angle(2), Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf t_Vy(angle(1), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf t_Vx(angle(0), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf t_V;
    t_V = t_Vz * t_Vy * t_Vx;

    return t_V.matrix();
}
}