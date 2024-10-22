/**
 * @file main.cpp
 * @brief 多线程ASC文件转换为PCD文件
 * @version 0.2
 * @date 2024-08-21
 *
 */

#include <glog/logging.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <fstream>
#include <mutex>
#include <pcl/impl/point_types.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

// 定义点云文件的路径及文件名
const std::string pcd_path = "/home/hzy/bag_map/map/NMGTW_1F/map_PCD/";
const std::string loadName = "121212.asc";  // 输入的ASC文件名
const std::string saveName = "tmp.pcd";     // 输出的PCD文件名

// 全局变量，用于多线程数据存储和访问控制
std::mutex cloud_mutex;  // 用于保护点云数据的互斥量
pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_global(
    new pcl::PointCloud<pcl::PointXYZINormal>());

/**
 * @brief 解析ASC文件的一部分内容，并将解析得到的点加入全局点云对象中
 *
 * @param lines 要解析的文件内容行
 */
void parsePartOfFile(const std::vector<std::string>& lines) {
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_part(
      new pcl::PointCloud<pcl::PointXYZINormal>());

  for (const auto& line : lines) {
    std::istringstream iss(line);
    pcl::PointXYZINormal point;

    // 解析一行数据，读取 x, y, z, intensity, normal_x, normal_y, normal_z
    if (!(iss >> point.x >> point.y >> point.z >> point.intensity >>
          point.normal_x >> point.normal_y >> point.normal_z)) {
      LOG(ERROR) << "解析错误: " << line;
      continue;
    }

    // 将解析成功的点加入局部点云对象中
    cloud_part->points.push_back(point);
  }

  // 加锁保护，并将局部点云数据加入全局点云对象中
  std::lock_guard<std::mutex> lock(cloud_mutex);
  *cloud_global += *cloud_part;
}

int main(int argc, char** argv) {
  // 初始化Google日志
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_colorlogtostderr = true;
  FLAGS_logtostderr = true;

  // 拼接输入输出文件路径
  std::string loadPath = pcd_path + loadName;
  std::string savePath = pcd_path + saveName;

  // 打开ASC文件
  std::ifstream ascFile(loadPath);
  if (!ascFile.is_open()) {
    LOG(ERROR) << "无法打开文件!" << std::endl;
    return -1;
  }

  std::string line;
  std::vector<std::string> lines;  // 存储文件中的每一行内容
  while (std::getline(ascFile, line)) {
    lines.push_back(line);  // 将每一行内容存入lines中
  }
  ascFile.close();

  LOG(INFO) << "文件读取完成，总行数: " << lines.size();

  // 定义线程数，根据CPU核心数设定
  unsigned int num_threads = std::thread::hardware_concurrency();
  if (num_threads == 0)
    num_threads = 4;  // 如果无法检测CPU核心数，默认使用4个线程
  LOG(INFO) << "使用线程数: " << num_threads;

  // 将文件行内容分配给多个线程
  std::vector<std::thread> threads;
  size_t lines_per_thread = lines.size() / num_threads;
  for (unsigned int i = 0; i < num_threads; ++i) {
    size_t start_idx = i * lines_per_thread;
    size_t end_idx =
        (i == num_threads - 1) ? lines.size() : (i + 1) * lines_per_thread;
    std::vector<std::string> part_lines(lines.begin() + start_idx,
                                        lines.begin() + end_idx);

    // 创建并启动线程，每个线程处理一部分文件内容
    threads.emplace_back(parsePartOfFile, part_lines);
  }

  // 等待所有线程处理完毕
  for (auto& thread : threads) {
    thread.join();
  }

  // 设置点云的宽度、高度及其他参数
  cloud_global->width = cloud_global->points.size();
  cloud_global->height = 1;  // 无序点云
  cloud_global->is_dense = false;

  LOG(INFO) << "cloud_global->points.size():" << cloud_global->points.size();

  // 将最终的点云数据保存为PCD文件
  pcl::PCDWriter writer;
  writer.writeBinary(savePath, *cloud_global);
  LOG(INFO) << "点云文件保存成功: " << savePath;

  return 0;
}
