/*
 * @Description: ICP 匹配模块
 * @Author: Zhou Yong
 * @Date: 2020-11-27 11:40:18
 */

#include "my_lidar_localization/models/registration/icp_registration.hpp"

#include "glog/logging.h"

namespace my_lidar_localization
{
ICPRegistration::ICPRegistration(const YAML::Node& node)
    :icp_ptr_(new pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT>()){
        float max_corr_dis = node["max_corr_dis"].as<float>();
        int max_iter = node["max_iter"].as<int>();
        float trans_eps = node["trans_eps"].as<float>();
        float euc_eps = node["euc_eps"].as<float>();

        SetRegistrationParam(max_corr_dis, max_iter,trans_eps,euc_eps);
}

ICPRegistration::ICPRegistration(float max_corr_dis , int max_iter, float trans_eps, float euc_eps)
{
    SetRegistrationParam(max_corr_dis, max_iter, trans_eps, euc_eps);
}

bool ICPRegistration::SetRegistrationParam(float max_corr_dis, int max_iter, float trans_eps, float euc_eps)
{
    icp_ptr_->setMaxCorrespondenceDistance(max_corr_dis);
    icp_ptr_->setMaximumIterations(max_iter);
    icp_ptr_->setTransformationEpsilon(trans_eps);
    icp_ptr_->setEuclideanFitnessEpsilon(euc_eps);

    LOG(INFO) << "ICP 的匹配参数为：" << std::endl
              << "max_corr_dis: " << max_corr_dis << ", "
              << "max_iter: " << max_iter << ", "
              << "trans_eps: " << trans_eps << ", "
              << "euc_eps: " << euc_eps 
              << std::endl << std::endl;

    return true;
}

bool ICPRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target)
{
    icp_ptr_->setInputTarget(input_target);

    return true;
}

bool ICPRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source, const Eigen::Matrix4f& predict_pose, CloudData::CLOUD_PTR& result_cloud_ptr, Eigen::Matrix4f& result_pose)
{
    icp_ptr_->setInputSource(input_source);
    icp_ptr_->align(*result_cloud_ptr, predict_pose);
    
    result_pose = icp_ptr_->getFinalTransformation();

    return true;
}
}