/*
 *@Description:
 *@Author:
 *@Date:
*/

#ifndef MY_LIDAR_LOCALIZATION_SENSOR_DATA_IMU_DATA_HPP_
#define MY_LIDAR_LOCALIZATION_SENSOR_DATA_IMU_DATA_HPP_

#include <Eigen/Dense>

namespace my_lidar_localization
{
class IMUData
{
public:
    struct LinearAcceleration{
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct AngularVelocity{
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct Orientation{
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 0.0;
    };

    double time = 0.0;
    LinearAcceleration linear_acceleration;
    AngularVelocity angular_velocity;
    Orientation orientation;
public:
    // 把四元数转成旋转矩阵
    Eigen::Matrix3f GetOrientationMatrix(){
        Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
        Eigen::Matrix3f matrix = q.matrix().cast<float>();

        return matrix;
    }
};
}

#endif
