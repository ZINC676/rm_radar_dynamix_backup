//
// Created by jg on 22-4-22.
//
#include "lidar.h"
#include <nodelet/loader.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_node");

  rm_radar::Lidar lidar;
  lidar.nh_ = ros::NodeHandle("~");
  lidar.onInit();

  ros::spin();
  return 0;
}