/*
 *@Description:
 *@Author:
 *@Date:
*/

#include "my_lidar_localization/sensor_data/gnss_data.hpp"
#include "glog/logging.h"

bool my_lidar_localization::GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian my_lidar_localization::GNSSData::geo_converter;

namespace my_lidar_localization
{

// 初始化位置
void GNSSData::InitOriginPosition()
{
    geo_converter.Reset(latitude, longitude, altitude);
    origin_position_inited = true;
}

// 更新位姿
void GNSSData::UpdateXYZ()
{
    if(!origin_position_inited){
        LOG(WARNING) << "GeoConverter has not set origin position";
    }
    geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
}
}