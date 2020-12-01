/*
 * @Description: 读写文件管理
 * @Author: Ren Qian
 * @Date: 2020-02-24 19:22:53
 */
#ifndef MY_LIDAR_LOCALIZATION_TOOLS_FILE_MANAGER_HPP_
#define MY_LIDAR_LOCALIZATION_TOOLS_FILE_MANAGER_HPP_

#include <string>
#include <iostream>
#include <fstream>

namespace my_lidar_localization {
class FileManager{
  public:
    static bool CreateFile(std::ofstream& ofs, std::string file_path);
    static bool CreateDirectory(std::string directory_path);
};
}

#endif
