#ifndef MY_LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_HPP_
#define MY_LIDAR_LOCALIZATION_MODELS_REGISTRATION_ICP_REGISTRATION_HPP_

#include <pcl/registration/icp.h>
#include "my_lidar_localization/models/registration/registration_interface.hpp"

namespace my_lidar_localization
{
class ICPRegistration: public RegistrationInterface{
public:
    ICPRegistration(const YAML::Node& node);
    ICPRegistration(float max_corr_dis, int max_iter, float trans_eps, float euc_eps);

    bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
    bool ScanMatch(const CloudData::CLOUD_PTR& input_source,
                   const Eigen::Matrix4f& predict_pose,
                   CloudData::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
private:
    bool SetRegistrationParam(float max_corr_dis, int max_iter, float trans_eps, float euc_eps);

private:
    pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>::Ptr icp_ptr_; 
};
}

#endif