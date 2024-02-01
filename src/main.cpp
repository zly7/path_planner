/**
   \file main.cpp
   \brief Main entry point of the program, starts an instance of Planner
*/

//###################################################
//                      HYBRID A* ALGORITHM
//  AUTHOR:   Karl Kurzer
//  WRITTEN:  2015-03-02
//###################################################

#include <cstring>
#include <iostream>
#include <ros/ros.h>

#include "constants.h"
#include "planner.h"
#include <yaml-cpp/yaml.h>
#include <regex>
#include "constants.h"
#include <dirent.h> 
#include <thread>
#include <chrono>
#include <atomic>
// 全局原子变量用于控制后台线程的执行
std::atomic<bool> keep_running(true);

void flushEverySecond(std::ofstream& out) {
    while (keep_running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        out.flush();
    }
}
//###################################################
//                              COUT STANDARD MESSAGE
//###################################################
/**
   \fn message(const T& msg, T1 val = T1())
   \brief Convenience method to display text
*/
template<typename T, typename T1>
void message(const T& msg, T1 val = T1()) {
  if (!val) {
    std::cout << "### " << msg << std::endl;
  } else {
    std::cout << "### " << msg << val << std::endl;
  }
}

//###################################################
//                                               MAIN
//###################################################
/**
   \fn main(int argc, char** argv)
   \brief Starting the program
   \param argc The standard main argument count
   \param argv The standard main argument value
   \return 0
*/
int main(int argc, char** argv) {
  message("resolution: ", HybridAStar::Constants::each_meter_to_how_many_pixel);

  if (HybridAStar::Constants::manual) {
    message("mode: ", "manual");
  } else {
    message("mode: ", "auto");
  }

  ros::init(argc, argv, "a_star");


  std::string packagePath = "/home/zly/plannerAll/catkin_path_planner";
  std::string yamlPath = packagePath + "/src/path_planner/maps/map.yaml";
  YAML::Node config = YAML::LoadFile(yamlPath);
  std::string unique_number = "-1";
  if (config["image"]) {
    std::string map_image = config["image"].as<std::string>();
    size_t tpcap_pos = map_image.find("TPCAP");
    if (tpcap_pos != std::string::npos) {
      std::regex number_regex("(\\d+)");
      std::smatch match;
      if (std::regex_search(map_image, match, number_regex)) {
          unique_number = match[0].str();
      }
    }
  }
  std::string baseDir = "/home/zly/plannerAll/catkin_path_planner/finalTime/";
  std::string dirPath = baseDir + (HybridAStar::Constants::algorithm == "contour_hybrid_astar" ? "ENHA" :
                                  HybridAStar::Constants::algorithm == "hybrid_astar" ? "HybridA" : "EHHA");
  std::string filePattern = "TPCAP_" + unique_number + "_";
  struct dirent* entry;
  int maxIndex = -1;
  DIR* dir = opendir(dirPath.c_str());
  if (dir == NULL) {
      std::cerr << "Failed to open directory" << std::endl;
      return -1; // 目录打开失败
  }
  while ((entry = readdir(dir)) != NULL) {
      std::string filename(entry->d_name);
      if (filename.find(filePattern) != std::string::npos) { 
          size_t pos = filename.rfind('_'); //last
          if (pos != std::string::npos) {
              int index = std::stoi(filename.substr(pos + 1));
              if (index > maxIndex) {
                  maxIndex = index;
              }
          }
      }
  }
  closedir(dir);
  std::ofstream out;
  std::string outPath = dirPath + "/" + filePattern + std::to_string(maxIndex + 1) + ".txt";
  out.open(outPath, std::ios::out);
  std::thread flush_thread(flushEverySecond, std::ref(out));
  std::streambuf *coutbuf = std::cout.rdbuf();
  std::cout.rdbuf(out.rdbuf());

  HybridAStar::Planner hy;
  hy.plan(); 

  ros::spin();
  // 停止后台线程
  keep_running = false;
  flush_thread.join(); // 等待后台线程完成
  std::cout.rdbuf(coutbuf);
  out.close();
  return 0;
}
