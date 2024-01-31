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
  std::ofstream out("/home/zly/plannerAll/catkin_path_planner/finalTime/ENHA/TPCAP_"+unique_number+".txt", std::ios::app);
  std::streambuf *coutbuf = std::cout.rdbuf();
  std::cout.rdbuf(out.rdbuf());

  HybridAStar::Planner hy;
  hy.plan(); 

  ros::spin();
  std::cout.rdbuf(coutbuf);
  out.close();
  return 0;
}
