/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-02-06 20:42:23
 */
#include "my_lidar_localization/sensor_data/gnss_data.hpp"

#include "glog/logging.h"

//静态成员变量必须在类外初始化
bool my_lidar_localization::GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian my_lidar_localization::GNSSData::geo_converter;

namespace my_lidar_localization {

void GNSSData::InitOriginPosition() {
    geo_converter.Reset(latitude, longitude, altitude);
    origin_position_inited = true;
}

void GNSSData::UpdateXYZ() {
    if (!origin_position_inited) {
        LOG(WARNING) << "GeoConverter has not set origin position";
    }
    geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
}
}