/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-03-01 18:35:19
 */

#include "my_lidar_localization/models/graph_optimizer/interface_graph_optimizer.hpp"

namespace my_lidar_localization {
void InterfaceGraphOptimizer::SetMaxIterationsNum(int max_iterations_num) {
    max_iterations_num_ = max_iterations_num;
}
}